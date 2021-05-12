/**
 *  Derek Parks
 *  
 *  Read a Non-invasive AC Current Sensor (SCT-013) hooked up to a adc (ADS1115) and a ESP-01. Then emit to mqtt.
 *  
 *  Saves wifi and mqtt creds to SPIFFS.
 *  
 *  If wifi creds not found will go into AP mode.
 *  
 *  MIT license for all new code.  
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>          

#include <Adafruit_ADS1015.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "EmonLib.h"

#include <ArduinoJson.h>
#include "AsyncJson.h"
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>


#define VOLTAGE_ADDR 0
#define RESET_ADDR sizeof(float)
#define CAL_ADDR (RESET_ADDR + sizeof(unsigned long))
#define THRES_ADDR (CAL_ADDR + sizeof(float))
#define WIFI_PATH "/wifi.json"
#define MQTT_PATH "/mqtt.json"
#define CONFIG_SSID "CurrentSensorAP"
#define OTA false


AsyncWebServer server(80);
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
EnergyMonitor emon1;

WiFiClient wclient;
StaticJsonDocument<200> doc;

char jsonBuffer[256];
char ssid[32];
char password[32];

//Voltage of power line
float meassuredVoltage = 115;
//Calibration factor for your sensor
float calibrationFac = 0.0947;
//Apperent power before this will get set to zero
float zeroThreshold = 5;

//Watts will be summed until this timeout and then sent out to the mqtt server
unsigned long resetMills = 900000; //15 min

char mqtt_server[17];
unsigned int mqtt_port = 1883;
char mqtt_user[15];
char mqtt_topic[20] = "HOME/POWER";
char mqtt_password[15];
char CLIENT_ID[15];

PubSubClient client(mqtt_server, mqtt_port, wclient);
unsigned long tLast;

boolean isInAPMode = false;
boolean isMQTTFound = false;

unsigned long millsLast;
unsigned long millsSum = 0;
unsigned long measurementCount = 0;
double wattsSum = 0;


int ads1115PinReader(int unused) {
  return ads.readADC_Differential_2_3();
}


/**
 * Called every loop. Maintain wifi and mqtt connections.
 */
void processNet() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    if (client.connected()) {
      client.loop();
    } else {
      if (!isInAPMode && isMQTTFound && client.connect(CLIENT_ID, mqtt_user, mqtt_password)) {
         Serial.println("Connected MQTT");
      }
    }
  } else if (client.connected()) client.disconnect();
}


/**
 * Save a measured voltage
 */
void updateMeasuredVoltage(float newVoltage) {
  meassuredVoltage = newVoltage;
  EEPROM.put(VOLTAGE_ADDR, meassuredVoltage);
  EEPROM.commit();
  
  WebSerial.print("Updating Voltage to:");
  WebSerial.println(meassuredVoltage);
  Serial.print("Updating Voltage to:");
  Serial.println(meassuredVoltage);
}


/**
 * Read measured voltage out of the eeprom
 */
void updateMeasuredVoltageFromEEPROM() {
  float tempMeassuredVoltage = 0.0f;
  EEPROM.get(VOLTAGE_ADDR, tempMeassuredVoltage);

  if(tempMeassuredVoltage > 0.00001f) {
    meassuredVoltage = tempMeassuredVoltage;
  }  
}


/**
 * Save a new calibration factor
 */
void updateCalFactor(float cal) {
  calibrationFac = cal;
  EEPROM.put(CAL_ADDR, calibrationFac);
  EEPROM.commit();
  
  WebSerial.print("Updating Cal factor to:");
  WebSerial.println(calibrationFac);
  Serial.print("Updating Voltage to:");
  Serial.println(calibrationFac);
  ESP.restart();
}


/**
 * Read calibration factor the eeprom
 */
void readCalFactorEEPROM() {
  float tempCalibrationFac = 0.0f;
  EEPROM.get(CAL_ADDR, tempCalibrationFac);

  if(tempCalibrationFac > 0.00001f) {
    calibrationFac = tempCalibrationFac;
  }
  WebSerial.print("Cal factor @Boot:");
  WebSerial.println(calibrationFac);
}


/**
 * Save a new zero threashold
 */
void updateZeroThreshold(float threshold) {
  zeroThreshold = threshold;
  EEPROM.put(THRES_ADDR, zeroThreshold);
  EEPROM.commit();
  
  WebSerial.print("Updating zero threashold to:");
  WebSerial.println(zeroThreshold);
  Serial.print("Updating zero threashold to:");
  Serial.println(zeroThreshold);
  ESP.restart();
}


/**
 * Read zero threashold the eeprom
 */
void readZeroThresholdEEPROM() {
  float tempZeroThreshold = 0.0f;
  EEPROM.get(THRES_ADDR, tempZeroThreshold);

  if(tempZeroThreshold > 0.00001f) {
    zeroThreshold = tempZeroThreshold;
  }
  WebSerial.print("Zero threashold @Boot:");
  WebSerial.println(zeroThreshold);
}


/**
 * Save the reset time to the EEPROM
 */
void updateReset(unsigned long newResetSeconds) {
  resetMills = newResetSeconds;
  EEPROM.put(RESET_ADDR, resetMills);
  EEPROM.commit();
  
  Serial.print("Updating Voltage to:");
  Serial.println(resetMills);
}


/**
 * Read the reset time from the EEPROM
 */
void updateResetFromEEPROM() {
  unsigned long tempResetMills = 0;
  EEPROM.get(RESET_ADDR, tempResetMills);

  if(tempResetMills > 0) {
    resetMills = tempResetMills;
  }
}


/**
 * Read message from webserial
 */
void recvMsg(uint8_t *data, size_t len){
  if(len > 2) {
    if(data[0] == 'V' || data[0] == 'v') {
        String d = "";
        for(int i = data[1]==' '?2:1; i < len; i++) {
          d += char(data[i]);
        }
        updateMeasuredVoltage(d.toFloat());
    }    
  }
}


/**
 * Load wifi cred from SPIFFS
 */
boolean loadWifi() {
  StaticJsonDocument<200> wifiJson;
  Serial.println(F("Loading wifi file"));

  if(!SPIFFS.exists(WIFI_PATH)) {
    Serial.println(F("Creds file not found."));
    return false;
  }
  
  File configFile = SPIFFS.open(WIFI_PATH, "r");

  DeserializationError error = deserializeJson(wifiJson, configFile);
  if (error) {
    Serial.println(F("Error: deserializeJson"));
    Serial.println(error.c_str());
    return false;
  }
  
  strcpy(ssid, wifiJson["ssid"]);
  strcpy(password, wifiJson["pass"]);
  configFile.close();
  return true;
}


/**
 * Load mqtt creds from SPIFFS
 */
boolean loadMqtt() {
  StaticJsonDocument<200> mqttJson;
  Serial.println(F("Loading mqtt file"));

  if(!SPIFFS.exists(MQTT_PATH)) {
    Serial.println(F("Mqtt file not found."));
    return false;
  }
  
  File configFile = SPIFFS.open(MQTT_PATH, "r");

  DeserializationError error = deserializeJson(mqttJson, configFile);
  if (error) {
    Serial.println(F("Error: deserializeJson"));
    Serial.println(error.c_str());
    return false;
  }
  strcpy(mqtt_server, mqttJson["mqtt_server"]);
  mqtt_port = mqttJson["mqtt_port"];
  strcpy(mqtt_user, mqttJson["mqtt_user"]);
  strcpy(mqtt_password, mqttJson["mqtt_password"]);
  strcpy(CLIENT_ID, mqttJson["client_id"]);

  if(mqttJson.containsKey("mqtt_topic")) {
     strcpy(mqtt_topic, mqttJson["mqtt_topic"]);

  }
  
  configFile.close();
  return true;
}


/**
 * Save mqtt creds to SPIFFS
 */
void saveMqtt() {
  StaticJsonDocument<200> mqttJson;
  mqttJson["mqtt_server"] = mqtt_server;
  mqttJson["mqtt_user"] = mqtt_user;
  mqttJson["mqtt_topic"] = mqtt_topic;
  mqttJson["mqtt_password"] = mqtt_password;
  mqttJson["client_id"] = CLIENT_ID;
  mqttJson["mqtt_port"] = mqtt_port;
  serializeJson(mqttJson, jsonBuffer);
  
  File file = SPIFFS.open(MQTT_PATH, "w");
 
  if (!file) {
      Serial.println("Error opening file for writing");
      return;
  }

  int bytesWritten = file.print(jsonBuffer);
   
  if (bytesWritten == 0) {
      Serial.println("File write failed");
      return;
  }
  
  file.close();
  isMQTTFound = true;
}

/**
 * Save wifi creds to SPIFFS
 */
void saveWifi() {
  StaticJsonDocument<200> wifiJson;
  wifiJson["ssid"] = ssid;
  wifiJson["pass"] = password;

  serializeJson(wifiJson, jsonBuffer);
  File file = SPIFFS.open(WIFI_PATH, "w");
 
  if (!file) {
      Serial.println("Error opening file for writing");
      return;
  }

  int bytesWritten = file.print(jsonBuffer);
   
  if (bytesWritten == 0) {
      Serial.println("File write failed");
      return;
  }
  
  file.close();
  ESP.restart();
}


/**
 * Go into AP mode
 */
void apMode() {
  Serial.println("No saved Wifi creds...\nGoing to into AP Mode!");
  isInAPMode = true;
  WiFi.softAP(CONFIG_SSID);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}


/**
 * Go into client (aka station) mode
 */
void clientMode() {
  WiFi.mode(WIFI_STA);
  
  WiFi.begin(ssid, password);
   int failCount = 0;
   while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    failCount++;

    Serial.print("Status:");
    Serial.println(WiFi.status());
    
    if(failCount > 30) {
      SPIFFS.remove(WIFI_PATH);
      ESP.restart();
    }
  }

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


/**
 * Set OTA update method
 */
void otaSetup() {
  ArduinoOTA.onStart([]() {
    Serial.println("Starting updating");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nUpdated ended");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
}


/**
 * List SPIFFS contents
 */
void listRoot() {
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    Serial.print("FS File: ");
    Serial.print(fileName.c_str());
    Serial.print(" size: ");
    Serial.println(fileSize);
  }
}

/**
 * Setup all web server endpoints
 */
void setupServer() {
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
  server.on("/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> mqttJson;

    mqttJson["mqtt_server"] = mqtt_server;
    mqttJson["mqtt_port"] = mqtt_port;
    mqttJson["mqtt_user"] = mqtt_user;
    mqttJson["mqtt_topic"] = mqtt_topic;
    mqttJson["client_id"] = CLIENT_ID;
    
    serializeJson(mqttJson, jsonBuffer);
    request->send(200, "text/json", jsonBuffer);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> wifiStatusJson;

    if(WiFi.status() == WL_CONNECTED) wifiStatusJson["status"] = "Connected";
    else wifiStatusJson["status"] = "Not Connected";
    
    wifiStatusJson["mqttConn"] = client.connected();

    const int mqtt_state = client.state();
    if(mqtt_state == MQTT_CONNECTION_TIMEOUT) wifiStatusJson["mqttState"] = "Connection Timeout";
    else if(mqtt_state == MQTT_CONNECTION_LOST) wifiStatusJson["mqttState"] = "Connection Lost";
    else if(mqtt_state == MQTT_CONNECT_FAILED) wifiStatusJson["mqttState"] = "Connected Failed";
    else if(mqtt_state == MQTT_DISCONNECTED) wifiStatusJson["mqttState"] = "Disconnected";
    else if(mqtt_state == MQTT_CONNECTED) wifiStatusJson["mqttState"] = "Connected";
    else if(mqtt_state == MQTT_CONNECT_BAD_PROTOCOL) wifiStatusJson["mqttState"] = "Bad Protocol";
    else if(mqtt_state == MQTT_CONNECT_BAD_CLIENT_ID) wifiStatusJson["mqttState"] = "Bad Client ID";
    else if(mqtt_state == MQTT_CONNECT_UNAVAILABLE) wifiStatusJson["mqttState"] = "Unavailable";
    else if(mqtt_state == MQTT_CONNECT_BAD_CREDENTIALS) wifiStatusJson["mqttState"] = "Bad Credentials";
    else if(mqtt_state == MQTT_CONNECT_UNAUTHORIZED) wifiStatusJson["mqttState"] = "Unauthorized";
    
    wifiStatusJson["SSID"] = WiFi.SSID();
    wifiStatusJson["IP"] = WiFi.localIP().toString();
    wifiStatusJson["APIP"] = WiFi.softAPIP().toString();
    
    serializeJson(wifiStatusJson, jsonBuffer);
    request->send(200, "text/json", jsonBuffer);
  });

  server.on("/values", HTTP_GET, [](AsyncWebServerRequest *request) {
    doc["voltage"] = meassuredVoltage;
    doc["cal_factor"] = calibrationFac;
    doc["zero_thres"] = zeroThreshold;
    doc["update_s"] = resetMills/1000;
    serializeJson(doc, jsonBuffer);
    request->send(200, "text/json", jsonBuffer);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    boolean updated = false;
    if (request->hasParam("voltage")) {
      AsyncWebParameter* pPayload = request->getParam("voltage");
      String value = pPayload->value();

      updateMeasuredVoltage(value.toFloat());
      updated = true;
    } 

    if (request->hasParam("cal")) {
      AsyncWebParameter* pPayload = request->getParam("cal");
      String value = pPayload->value();
      updateCalFactor(value.toFloat());
      updated = true;
    } 

    if (request->hasParam("zero_thres")) {
      AsyncWebParameter* pPayload = request->getParam("zero_thres");
      String value = pPayload->value();
      updateZeroThreshold(value.toFloat());
      updated = true;
    } 
    
    if (request->hasParam("update_s")) {
      AsyncWebParameter* pPayload = request->getParam("update_s");
      String value = pPayload->value();
      updateReset(value.toInt() * 1000);
      updated = true;
    } 

    if(updated) request->send(200, "text/json", "{'success':True}");
    else request->send(200, "text/json", "{'success':False}");
  });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/json");
    StaticJsonDocument<400> wifiListJson;
    const int networksFound = WiFi.scanComplete();
    if(networksFound == -2){
      WiFi.scanNetworks(true);
      wifiListJson["error"] = "again";
      serializeJson(wifiListJson, *response);
      request->send(response);
    } else if(networksFound) {
      Serial.print("Networks Found: ");
      Serial.println(networksFound);
      JsonArray scan = wifiListJson.createNestedArray("list");
      for (int i = 0; i < networksFound; ++i) {
        JsonObject item = scan.createNestedObject();
        item["ssid"] = WiFi.SSID(i);
        item["rssi"] = WiFi.RSSI(i);
        item["enc"] = WiFi.encryptionType(i);
      }

      serializeJson(wifiListJson, *response);
      request->send(response);
      WiFi.scanDelete();
      if(WiFi.scanComplete() == -2) {
        WiFi.scanNetworks(true);
      }
    }
  });


  
  AsyncCallbackJsonWebHandler* handlerMqtt = new AsyncCallbackJsonWebHandler("/mqtt", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<200> data;

    if (json.is<JsonObject>()) {
      data = json.as<JsonObject>();

      strcpy(mqtt_server, data["mqtt_server"]);
      mqtt_port = data["mqtt_port"];
      strcpy(mqtt_user, data["mqtt_user"]);
      strcpy(mqtt_topic, data["mqtt_topic"]);
      strcpy(mqtt_password, data["mqtt_password"]);
      strcpy(CLIENT_ID, data["client_id"]);
      data.clear();
 
      saveMqtt();
      request->send(200, "text/json", "{'success':True}");
    } else {
      request->send(200, "text/json", "{'success':False}");
    }
  });
  
  server.addHandler(handlerMqtt);
  AsyncCallbackJsonWebHandler* handlerWifi = new AsyncCallbackJsonWebHandler("/connect", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<200> data;

    if (json.is<JsonObject>()) {
      data = json.as<JsonObject>();

      strcpy(ssid, data["ssid"]);
      strcpy(password, data["password"]);
      
      data.clear();
      saveWifi();
      request->send(200, "text/json", "{'success':True}");
    } else {
      request->send(200, "text/json", "{'success':False}");
    }
  });
  
  server.addHandler(handlerWifi);
  server.begin();
}


void setup() {
  Serial.begin(74880);

  // initialize filesystem
  SPIFFS.begin();
  
  if(!loadWifi()) { //Check if found wifi creds
    apMode();
  } else {
    clientMode();
    isMQTTFound = loadMqtt();
  }

  if(OTA) otaSetup();
  
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);

  setupServer();
  processNet();

  Wire.begin(VOLTAGE_ADDR, 2);
  ads.begin();
  ads.setGain(GAIN_FOUR);

  EEPROM.begin(512);
  updateMeasuredVoltageFromEEPROM();
  updateResetFromEEPROM();
  readCalFactorEEPROM();

  emon1.inputPinReader = ads1115PinReader;
  emon1.current(-9999, calibrationFac);
  delay(3000); //delay for a bit to let everything calm down
  
  tLast = millis();
}


void webOutJSON(double apparentPower, double kwh) {
  doc["ApparentPower"] = apparentPower;
  doc["WattsSum"] = wattsSum;
  doc["MillsSum"] = millsSum;
  doc["kwh"] = kwh;

  serializeJson(doc, jsonBuffer);
  WebSerial.println(jsonBuffer);
}


void mqttJSON(double apparentPower, double kwh) {
  doc["ApparentPower"] = apparentPower;
  doc["WattsSum"] = wattsSum;
  doc["MillsSum"] = millsSum;
  doc["kwh"] = kwh;
  
  const size_t n = serializeJson(doc, jsonBuffer);
  client.publish(mqtt_topic, jsonBuffer, n);
  WebSerial.println("MQTT Update");
}


void loop(void) {
  const double Irms = emon1.calcIrms(300);
  
  const unsigned long t = millis();
  const unsigned long dMills = t - tLast;
  millsSum += dMills;
  double apparentPower = Irms * meassuredVoltage; //Watts

  if(apparentPower < zeroThreshold) {
    apparentPower = 0;
  }
  
  wattsSum += apparentPower;
  measurementCount++;
  const double kwh = (wattsSum / measurementCount / 1000.0) * (millsSum / 3600000.0);

  webOutJSON(apparentPower, kwh);
  
  processNet();
  if(millsSum > resetMills) {
    mqttJSON(apparentPower, kwh);

    millsSum = 0;
    wattsSum = 0;
    measurementCount = 0;
  }

  tLast = t;
  if(OTA) ArduinoOTA.handle();
}
