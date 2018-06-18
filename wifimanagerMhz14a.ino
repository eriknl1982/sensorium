#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#import "index.h"

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager


#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>         //MQTT lib

//needed for MH-Z14A
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>   //Hecked lib, because SCL / SDA pins are wrong way around

#include <Si7021.h>

#include <ESP8266mDNS.h>        // Include the mDNS library

SI7021 si7021;

#define INTERVAL 5000
#define MH_Z19_RX 12 //RX pin
#define MH_Z19_TX 13 //TX pin

int resetState = 0;

WiFiClient espClient;
PubSubClient client(espClient);

WiFiManager wifiManager;

ESP8266WebServer server(80);

//Mh-z14 stuff
byte mhzResp[9];    // 9 bytes bytes response
byte mhzCmdReadPPM[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
byte mhzCmdCalibrateZero[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
byte mhzCmdABCEnable[9] = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6};
byte mhzCmdABCDisable[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86};
byte mhzCmdReset[9] = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};
byte mhzCmdMeasurementRange1000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
byte mhzCmdMeasurementRange2000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
byte mhzCmdMeasurementRange3000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
byte mhzCmdMeasurementRange5000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};

int shifts=0,co2ppm;
long previousMillis = 0;

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z14

//custom patrameters for MH-Z14a
char mqtt_server[40];
char mqtt_port[6] ;
char mqtt_username[40];
char mqtt_password[40];
char publish_interval[10];
char mqtt_topic_co2[40];
char mqtt_topic_temperature[40];
char mqtt_topic_humidity[40];

//flag for saving data
bool shouldSaveConfig = false;

//MH-Z14a stuff
byte checksum(byte response[9]){
  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += response[i];
  }
  crc = 255 - crc + 1;
  return crc;
}

//Hande webserver root request
void handleRoot() {
  Serial.println("Handling webserver request");
  
  String configPage = config_page;

  configPage.replace("{v}", "Sensorium configuration");
  configPage.replace("{1}", mqtt_server);
  configPage.replace("{2}", mqtt_port);
  configPage.replace("{3}", mqtt_username);
  configPage.replace("{4}", mqtt_password);
  configPage.replace("{5}", mqtt_topic_co2);
  configPage.replace("{6}", mqtt_topic_temperature);
  configPage.replace("{7}", mqtt_topic_humidity);
  configPage.replace("{8}", publish_interval);
  
  server.send(200, "text/html", configPage);
  
}

//Mh-z14a calibration
void startCalibration() {
  Serial.println("Handling webserver request for startCalibration");
  server.send(200, "text/html", "calibration in progress");

  Serial.println("MH-z14a:Waiting half an hour before calibrating zero"); 
  delay(1800000);
  calibrateZero();
  Serial.println("MH-z14a:Zero was calibrated");
}


void saveSettings() {
  Serial.println("Handling webserver request savesettings");

  //store the updates values in the json config file
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = server.arg("mqtt_server");
    json["mqtt_port"] = server.arg("mqtt_port");
    json["mqtt_username"] = server.arg("mqtt_username");
    json["mqtt_password"] = server.arg("mqtt_password");
    json["publish_interval"] = server.arg("publish_interval");
    json["mqtt_topic_co2"] = server.arg("mqtt_topic_co2");
    json["mqtt_topic_temperature"] = server.arg("mqtt_topic_temperature");
    json["mqtt_topic_humidity"] = server.arg("mqtt_topic_humidity");

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();

    //put updated parameters into memory so they become effective immediately
    server.arg("mqtt_server").toCharArray(mqtt_server,40);
    server.arg("mqtt_port").toCharArray(mqtt_port,40);
    server.arg("mqtt_username").toCharArray(mqtt_username,40);
    server.arg("mqtt_password").toCharArray(mqtt_password,40);
    server.arg("publish_interval").toCharArray(publish_interval,10);
    server.arg("mqtt_topic_co2").toCharArray(mqtt_topic_co2,40);
    server.arg("mqtt_topic_temperature").toCharArray(mqtt_topic_temperature,40);
    server.arg("mqtt_topic_humidity").toCharArray(mqtt_topic_humidity,40);

    //mqtt settings might have changed, let's reconnect
    reconnect();
  
    server.send(200, "text/html", "Settings have been saved and will be used upon the next sensor readout");
}


void disableABC() {
 co2Serial.write(mhzCmdABCDisable, 9);
}

void enableABC() {
 co2Serial.write(mhzCmdABCEnable, 9);
}

void setRange5000() {
 co2Serial.write(mhzCmdMeasurementRange5000, 9);
}

void calibrateZero(){
 co2Serial.write(mhzCmdCalibrateZero, 9);
}

//Function to get CO2 from MH-Z14a
int readCO2() {
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9];
  co2Serial.write(cmd, 9);
  // The serial stream can get out of sync. The response starts with 0xff, try to resync.
  while (co2Serial.available() > 0 && (unsigned char)co2Serial.peek() != 0xFF) {
    co2Serial.read();
    shifts++;
  }

  memset(response, 0, 9);
  co2Serial.readBytes(response, 9);

  if (response[1] != 0x86)
  {
    Serial.println(" Invalid response from co2 sensor!");
    return -1;
  }

  if (response[8] == checksum(response)) {
    int responseHigh = (int) response[2];
    int responseLow = (int) response[3];
    int ppm = (256 * responseHigh) + responseLow;
    return ppm;
  } else {
    Serial.println("CRC error!");
    return -1;
  }
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Connection to previous set wifi failed. Erasing settings before becoming AP");
  SPIFFS.format();
}


void setup() {
  si7021.begin();
  si7021.setHumidityRes(12);
  Serial.begin(115200);
  Serial.println();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_username, json["mqtt_username"]);
          strcpy(mqtt_password, json["mqtt_password"]);
          strcpy(publish_interval, json["publish_interval"]);
          strcpy(mqtt_topic_co2, json["mqtt_topic_co2"]);
          strcpy(mqtt_topic_temperature, json["mqtt_topic_temperature"]);
          strcpy(mqtt_topic_humidity, json["mqtt_topic_humidity"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_username("username", "mqtt username", mqtt_username, 40);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 40);
  WiFiManagerParameter custom_publish_interval("publish_interval", "publish interval", publish_interval, 10);
  WiFiManagerParameter custom_mqtt_topic_co2("topic_co2", "mqtt topic co2", mqtt_topic_co2, 40);
  WiFiManagerParameter custom_mqtt_topic_temperature("topic_temperature", "mqtt topic temperature", mqtt_topic_temperature, 40);
  WiFiManagerParameter custom_mqtt_topic_humidity("topic_humidity", "mqtt topic humidity", mqtt_topic_humidity, 40);

  WiFiManagerParameter custom_text("<p>Fill the folowing values with your home assistant infromation. Username and password are optional</p>");
  wifiManager.addParameter(&custom_text);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.setAPCallback(configModeCallback);
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_publish_interval); 
  wifiManager.addParameter(&custom_mqtt_topic_co2);
  wifiManager.addParameter(&custom_mqtt_topic_temperature);
  wifiManager.addParameter(&custom_mqtt_topic_humidity);


  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Sensorium")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...!");

  //Define url's 
  server.on("/", handleRoot);
  server.on("/saveSettings", saveSettings);
  server.on("/startCalibration", startCalibration);
  server.begin();

  Serial.println("MH-z14a: Disabling ABC");
  disableABC();
  Serial.println("MH-z14a:Setting range to 5000"); 
  setRange5000();
  
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(publish_interval, custom_publish_interval.getValue());
  strcpy(mqtt_topic_co2, custom_mqtt_topic_co2.getValue());
  strcpy(mqtt_topic_temperature, custom_mqtt_topic_temperature.getValue());
  strcpy(mqtt_topic_humidity, custom_mqtt_topic_humidity.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_username"] = mqtt_username;
    json["mqtt_password"] = mqtt_password;
    json["publish_interval"] = publish_interval;
    json["mqtt_topic_co2"] = mqtt_topic_co2;
    json["mqtt_topic_temperature"] = mqtt_topic_temperature;
    json["mqtt_topic_humidity"] = mqtt_topic_humidity;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  client.setServer(mqtt_server, atoi(mqtt_port));

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  //Expose as mdns
  if (!MDNS.begin("sensorium1")) {             // Start the mDNS responder for sensorium.local
    Serial.println("Error setting up MDNS responder!");
  } else {
      Serial.println("mDNS responder started");
      MDNS.addService("http", "tcp", 80);
  }

}

//MQTT reconnect function
void reconnect() {
  client.setServer(mqtt_server, atoi(mqtt_port));
  Serial.print("Attempting MQTT connection to ");
  Serial.print(mqtt_server);
  Serial.print(" on port ");
  Serial.print(mqtt_port);
  Serial.print("...");
  
  if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
   } else {
     Serial.print("failed, rc=");
     Serial.print(client.state());
     Serial.println(" try again in 5 seconds");

    unsigned long previousMillis1 = 0;

     //Go into a loop until we are connected 
     while (!client.connected()) {
         server.handleClient();  //call to handle webserver connection needed, because the while loop will block the processor
         unsigned long currentMillis1 = millis();
         if(currentMillis1 - previousMillis1 >= 5000) {
         previousMillis1 = currentMillis1;
         Serial.print("Attempting MQTT connection...");
          if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
                Serial.println("connected");
             } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
             }
         }
      }
   }
}


//Initialize a reset is pin 12 is low
void resetstate (){
   resetState = digitalRead(12);
   if (resetState == LOW){
    Serial.println("It seems someone wants to go for a reset...");
   //stuff below doesn't work yet, since the ESP built in led is connected to pin 2, but the octocoupler is too...
    
    /*int count = 0;

    //Flash the led for 5 seconds
    while (count < 25) {
     // digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);                       // wait for a second
     // digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
      delay(100);                       // wait for a second
      count ++;
      
    }
  
    resetState = digitalRead(12);
    //See if they still want to go for it
    if (resetState == LOW){
       Serial.println("Let's do it");
    } else {
       Serial.println("They chickened out...");
    }
    */
    SPIFFS.format();
    wifiManager.resetSettings();
    delay(500);
    ESP.restart();     
    
  }
}


long lastMsg = 0;

void loop() {

  //for the webserver
  server.handleClient();

  resetstate();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
 
  long now = millis();
  //send a message every publish_interval (publish_interval in minutes)
  if (now - lastMsg > atoi(publish_interval) * 60 * 1000) {
    lastMsg = now;
    unsigned long currentMillis = millis();
     if (abs(currentMillis - previousMillis) > INTERVAL)
     {
      previousMillis = currentMillis;

      co2ppm=-999;
      co2ppm = readCO2();

      //If no proper co2 value is returned, try again
      while (co2ppm == -1){
        Serial.println("re-Requesting CO2 concentration...");
        co2ppm = readCO2();  
      }
      
      Serial.println(String(mqtt_topic_co2) + ':' + String(co2ppm));
      client.publish(mqtt_topic_co2, String(co2ppm).c_str(), true);

      String humidity = String(si7021.readHumidity(),true);
      Serial.println(String(mqtt_topic_humidity) + ":" + humidity);
      client.publish(mqtt_topic_humidity, humidity.c_str());

      String temperature = String(si7021.readTemp(),true);
      Serial.println(String(mqtt_topic_temperature) + ":" + temperature);
      client.publish(mqtt_topic_temperature, temperature.c_str());
      
     } 
  }
}
