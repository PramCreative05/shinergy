/**
 * Shinergy 1.5 Beta Demo
 * @author: Bagas Pram (pram.hao@gapp.nthu.edu.tw)
 * June 2024； July 23 Fixed Bug
**/

/* [#] Libraries for Data Structure*/
#include <cppQueue.h>
#include <ArduinoJson.h>

/* [#] Libraries for AsyncWebServer */
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <HTTPClient.h>
#include <TridentTD_LineNotify.h>

/* [#] LIbraries for OneWire */
#include "OneWireNg_CurrentPlatform.h"
#include <DallasTemperature.h>
#include "DHT.h"

/* [#] Libraries for SD Card */
#include "FS.h"
#include "SD.h"
#include "SPI.h"

/* [#] Libraries for I2C Devices*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_SHT20.h"

/*------------------------------ SENSOR & WIRING CONFIG PARAMETER ------------------------------*/
#define BUTTON_PIN 16
#define BUZZER_PIN 17
#define PWM_PIN 14
#define DS_PIN 4
#define DHT_PIN 32
#define LED_PIN 25
#define POT_PIN 33

// LCD Parameter
int lcdCol = 16;
int lcdRow = 2;

// One Wire Device Instances
const int oneWireBus = DS_PIN;     
OneWire oneWire(oneWireBus);
DallasTemperature sensorsDS(&oneWire);
DHT dht(DHT_PIN, DHT11);

// I2C Device Instances
LiquidCrystal_I2C lcd(0x3F, lcdCol, lcdRow);
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

/*------------------------------ NETWORK CONFIG PARAMETER ------------------------------*/
// 8080 async; 8888 softAp
#define LINE_TOKEN "qZQOgs1RRfGQCyllEII7171YoblnOz6zQ1EOQ0ACWZP"
String hostname = "Shinergy1.5";
WiFiMulti wifiMulti;
AsyncWebServer server(80); // Create AsyncWebServer object on port 80

/*------------------------------ SD CARD UTILS ------------------------------*/
/**
 * USAGE:
 * c_str() to convert to const char *
 * writeFile(SD, "/hello.txt", "Hello ");
 * appendFile(SD, "/hello.txt", "World!\n");
*/

void writeFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println(F("$> [!] Failed to open file for writing"));
        return;
    }
    if(file.print(message)){
        Serial.printf("$> New file: %s\n", path);
        Serial.println("$> [v] Header Added");
    } else {
        Serial.println("$> [!] Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println(F("$> [!] Failed to open file for appending"));
        return;
    }
    if(file.print(message)){
        Serial.println("$> [v] New Record Appended");
    } else {
        Serial.println("$> [!] Append failed");
    }
    file.close();
}

/*------------------------------ PROGRAM UTILS ------------------------------*/
// [&] System Health Flag Status
bool ena_sensorsDS = true;
bool ena_sensorsSHT = true;
bool ena_sensorsAM = true;

bool status_sensorsDS = false;
bool status_sensorsSHT = false;
bool status_sensorsAM = false;
bool status_SD =  false;
bool status_online = false;
bool status_phone = false;

IPAddress controlAP;
String controlAPString; 

// [&] Global Tracker Variable
unsigned long timerDelay = 120000;
unsigned long lastTime = 0;
unsigned long lastBlink = 0;
unsigned long blinkDelay = 2000;
unsigned long timerNotif = 120000;
unsigned long lastNotif = 0;
int netTimeOut = 60000;
String filename;
String startTime;
int lastPot = 0;

float sampling_DS = 0.0;
float sampling_SHT_Temp = 0.0;
float sampling_SHT_Hum = 0.0;
float sampling_AM_Temp = 0.0;
float sampling_AM_Hum = 0.0;
int counter = 0;

// [&] PWM Control Variable
const int pwm_pin = PWM_PIN;
const int frequency = 500;
const int pwm_channel = 0;
const int resolution = 8;

String slider_value = "0";
const char* input_parameter = "value";

// [&] Data Structures
typedef struct sensorRec {
  unsigned long timeStamp_ = 0;
  float temp1_ = 0.0;
  float temp2_ = 0.0;
  float temp3_ = 0.0;
  float hum2_ = 0.0;
  float hum3_ = 0.0;
} Record;

#define queueSize_ 15
cppQueue queueBatch(sizeof(Record), queueSize_, FIFO, false);
cppQueue queueUpload(sizeof(Record), queueSize_, FIFO, false);

// Wrap Up Sampling Data
String getSensorDataJson() {
  DynamicJsonDocument doc(1024);

  doc["t1"] = String(sampling_DS); 
  doc["t2"] = String(sampling_SHT_Temp);
  doc["t3"] = String(sampling_AM_Temp);
  doc["h2"] = String(sampling_SHT_Hum);
  doc["h3"] = String(sampling_AM_Hum);

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Wrap Up System Health
String getSystemJson() {
  DynamicJsonDocument doc(1024);

  doc["on"] = String(status_online); 
  doc["sd"] = String(status_SD);  
  doc["ip"] = controlAPString;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// [&] Utils
float readTempDS() {
  sensorsDS.requestTemperatures(); // sends command for all devices on the bus to perform a temperature conversion
  float tempC = sensorsDS.getTempCByIndex(0);
  if(tempC == DEVICE_DISCONNECTED_C) {   // Error code for no readings DEVICE_DISCONNECTED_C = -127
    Serial.print(F("$> [!] DS disconnected"));
    return -1.0;
  }
  
  Serial.println("DS: " + String(tempC) + " ºC");
  return tempC;
}

float readTempSHT() {
  float tempC = sht20.readTemperature();
  if(tempC == ERROR_I2C_TIMEOUT || tempC == ERROR_BAD_CRC){
    return -1.0;
  }
  Serial.println("SHT: " + String(tempC) + " ºC");
  return tempC;
}

float readHumSHT() {
  float hum = sht20.readHumidity();
  if(hum == ERROR_I2C_TIMEOUT || hum == ERROR_BAD_CRC){
    return -1.0;
  }
  Serial.println("SHT: " + String(hum) + " %");
  return hum;
}

float readTempAM() {
  float tempC = dht.readTemperature();
  if(isnan(tempC)) {
    return -1.0;
  }
  Serial.println("AM: " + String(tempC) + " ºC");
  return tempC;
}

float readHumAM() {
  float hum = dht.readHumidity();
  if(isnan(hum)) {
    return -1.0;
  }
  Serial.println("AM: " + String(hum) + " %");
  return hum;
}


void data_sampling() {
  lcd.setCursor(0,1);
  lcd.print("Getting Data:");
  Record set_record;
  for(int i = 0; i < queueSize_; i++) {
    if(status_sensorsDS) {
      float temp1 = readTempDS();
      set_record.temp1_ = temp1;
    }

    if(status_sensorsSHT) {
      float temp2 = readTempSHT();
      float hum2 = readHumSHT();
      set_record.temp2_ = temp2;
      set_record.hum2_ = hum2;
    }

    if(status_sensorsAM) {
      float temp3 = readTempAM();
      float hum3 = readHumAM();
      set_record.temp3_ = temp3;
      set_record.hum3_ = hum3;
    }
    

    set_record.timeStamp_ = millis() / 1000;
    queueBatch.push(&set_record);
    
    lcd.setCursor(14,1);
    lcd.print(i+1);
    beep();
    delay(3000);
  }
}

void batch_record() {
  lcd.setCursor(0,1);
  lcd.print("Data Process...");
  // [1] Init Tracker Variable
  float sumDS = 0.0;
  int countDS = 0;

  float sumTempSHT = 0.0;
  int countTempSHT = 0;
  float sumHumSHT = 0.0;
  int countHumSHT = 0;

  float sumTempAM = 0.0;
  int countTempAM = 0;
  float sumHumAM = 0.0;
  int countHumAM = 0;

  String params;
  
  // [2] Dump to SD and Filtering Queue for Upload (avoid corrupt data from SD card I/O)
  queueUpload.clean();
  while(!queueBatch.isEmpty()) {
    Record get_record;
    Record set_record;
    queueBatch.pop(&get_record);
    
    float temp1 = get_record.temp1_;
    if(temp1 > 0 && temp1 < 130.0) {
      sumDS += temp1;
      countDS++;
      set_record.temp1_ = temp1;
    }
  
    float temp2 = get_record.temp2_;
    if(temp2 > 0 && temp2 < 130.0) {
      sumTempSHT += temp2;
      countTempSHT++;
      set_record.temp2_ = temp2;
    }
  
    float hum2 = get_record.hum2_;
    if(hum2 > 0 && hum2 <= 100.0) {
      sumHumSHT += hum2;
      countHumSHT++;
      set_record.hum2_ = hum2;
    }
  
    float temp3 = get_record.temp3_;
    if(temp3 > 0 && temp3 < 130.0) {
      sumTempAM += temp3;
      countTempAM++;
      set_record.temp3_ = temp3;
    }
  
    float hum3 = get_record.hum3_;
    if(hum3 > 0 && hum3 <= 100.0) {
      sumHumAM += hum3;
      countHumAM++;
      set_record.hum3_ = hum3;
    }
    
    queueUpload.push(&set_record);
    
    if(status_SD) {
      // Save Format time, t1, t2, t3, h2, h3
      String newRecord = "\n" + String(set_record.timeStamp_) 
                          + ", " + String(set_record.temp1_)
                          + ", " + String(set_record.temp2_)
                          + ", " + String(set_record.temp3_)
                          + ", " + String(set_record.hum2_)
                          + ", " + String(set_record.hum3_); 
      appendFile(SD, filename.c_str(), newRecord.c_str());
    }

    beep();
  }
  queueBatch.clean();


  // [3] Calculating average and update global tracker
  sampling_DS = sumDS / countDS;
  sampling_SHT_Temp = sumTempSHT / countTempSHT;
  sampling_SHT_Hum = sumHumSHT / countHumSHT;
  sampling_AM_Temp = sumTempAM / countTempAM;
  sampling_AM_Hum = sumHumAM / countHumAM;

  Serial.print(F("AvgSam High Temp = "));
  Serial.println(sampling_DS);
  Serial.print(F("AvgSam Box Temp = "));
  Serial.println(sampling_SHT_Temp);
  Serial.print(F("AvgSam Env Temp = "));
  Serial.println(sampling_AM_Temp);
  Serial.print(F("AvgSam Box Hum = "));
  Serial.println(sampling_SHT_Hum);
  Serial.print(F("AvgSam Env Hum = "));
  Serial.println(sampling_AM_Hum);

  // [4] Upload to Sheets
  if(status_online) {
    Record get_record;
    while(!queueUpload.isEmpty()) {
      digitalWrite(LED_PIN, HIGH);
      queueUpload.pop(&get_record);
      // Upload to sheets
      HTTPClient http;
      String url = "https://script.google.com/macros/s/AKfycby_14tpACr7-govGJqmVkKD9IL68NfSiPf7uFJcV41FL8fevuU3SwINb7dyacrFHkXH/exec?";
      params = "t1=" + String(sampling_DS) + "&t2=" + String(sampling_SHT_Temp) + "&t3=" + String(sampling_AM_Temp) + "&h2=" + String(sampling_SHT_Hum) + "&h3=" + String(sampling_AM_Hum);
      url += params;

      http.begin(url.c_str());
      int httpCode = http.GET();
      
      if (httpCode > 0) {
        Serial.println(F("$> [v] Upload Success"));
      } else {
        Serial.println(F("$> [!] Upload Failed"));
      }
      http.end();
      beep();
    }
    digitalWrite(LED_PIN, LOW);
  } else {
    queueUpload.clean();
  }
  
  // [5] LINE NOTIFY
  Serial.println(F("$> [v] Finish Batching, Now notify..."));
  LINE.notify("\nCollector Temp: " + String(sampling_DS) + " C" + "\nBox: " + String(sampling_SHT_Temp) + " C   " + String(sampling_SHT_Hum) + "%");
}

void beep(){
  digitalWrite(BUZZER_PIN, HIGH); 
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
}

int WiFiStatus;
String Get_WiFiStatus(int Status){
    switch(Status){
        case WL_IDLE_STATUS:
          return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
          return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
          return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
          return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
          return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
          return "WL_CONNECTED";
        case WL_DISCONNECTED:
          return "WL_DISCONNECTED";
        default:
            return "UNKNOWN";
    }
}

String IpAddress2String(const IPAddress& ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

// Callback funtion for WiFi Event Handler
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  status_online =  true;
  Serial.println(F("$> [v] Connected To The WiFi Network"));
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print(F("$> [v] Local Shinergy Online IP: "));
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  status_online =  false;
  Serial.println(info.wifi_sta_disconnected.reason);
  //  wifiMulti.run();
}

void WiFistartAP(WiFiEvent_t event, WiFiEventInfo_t info){
  controlAP = WiFi.softAPIP();
  controlAPString = IpAddress2String(controlAP);
  Serial.print(F("$> [v] AP IP: "));
  Serial.println(controlAPString);
}

void displayStat() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("IP:");
  lcd.print(controlAPString);
  lcd.setCursor(0,1);
  lcd.print("Col:");
  lcd.print(sampling_DS);
  lcd.print(char(223));
  lcd.print("C");

  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("IP:");
  lcd.print(controlAPString);
  lcd.setCursor(0,1);
  lcd.print("B:");
  lcd.print(round(sampling_SHT_Temp));
  lcd.print(char(223));
  lcd.print("C ");
  lcd.print(round(sampling_SHT_Hum));
  lcd.print("%");
}

void setup(){
  /*------------------------------ PROGRAM INIT AND TEST ------------------------------*/
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // input pull-up mode for button
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT); 
  pinMode(LED_PIN, OUTPUT);
  randomSeed(analogRead(34));

  digitalWrite(LED_PIN, LOW);

  ledcSetup(pwm_channel, frequency, resolution);
  ledcAttachPin(pwm_pin, pwm_channel);
  ledcWrite(pwm_channel, slider_value.toInt());

  lcd.init();                 
  lcd.backlight();

  /*---------- WELCOME DISPLAY ----------*/
  lcd.setCursor(0, 0);
  lcd.print("<** SHINERGY **>");

  int openDur;
  for(int tempo = 5; tempo > 1; tempo--){
    openDur = tempo*100;
    digitalWrite(BUZZER_PIN, HIGH);
    delay(openDur);
    digitalWrite(BUZZER_PIN, LOW);
    delay(openDur);
  }
  delay(500);
  beep();
  lcd.setCursor(0,1);
  lcd.print("AGRI ");
  delay(500);
  beep();
  lcd.print("SOLAR ");
  delay(500);
  beep();
  lcd.print("DRYER");
  
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  
  /*------------------------------ SENSORS INIT AND TEST ------------------------------*/
  if(ena_sensorsDS) {
    sensorsDS.begin();
    if(sensorsDS.getDS18Count() > 0) {
      status_sensorsDS = true;
      Serial.println(F("$> [v] High Temp DS Sensor is connected"));
      beep();
    } else {
      Serial.println(F("$> [！] No High Temp Sensor (DS)"));
      status_sensorsDS = false;
//      ESP.restart();
    }
  }
  
  if(ena_sensorsSHT) {
    // !!! NEED WDT TIME OUT HANDLER
    sht20.initSHT20();
    delay(100);
    sht20.checkSHT20();
    beep();
    status_sensorsSHT = true;
  }
  
  if(ena_sensorsAM) {
    dht.begin();
    float humd = dht.readHumidity();
    float temp = dht.readTemperature();
    
    if (!isnan(humd) || !isnan(temp) ) {
      status_sensorsAM = true;
      Serial.println(F("$> [v] Env Sensor is connected"));
      beep();
    } else {
      Serial.println(F("$> [!] No Env Sensor (AM)"));
      status_sensorsDS = false;
//      ESP.restart();
    }

  }

  /*------------------------------ NETWORK INIT AND TEST ------------------------------*/
  // [1] Connect to a network and become soft AP
  // Add list of WiFi networks
  wifiMulti.addAP("SmartComm", "smartcomm_pass");
  wifiMulti.addAP("shinergy", "shinergy88");
  wifiMulti.addAP("HSNL-714", "nf5731063");
  

  WiFi.mode(WIFI_MODE_APSTA);
  
  WiFi.onEvent(WiFistartAP, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_START);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.softAP("Shinergy", "tcs8888");
  
  unsigned long int startConnTime = millis();
  WiFiStatus = wifiMulti.run();
  while(WiFiStatus != WL_CONNECTED){
      delay(250);
      Serial.print(F("$> [!] Searching Network"));
      WiFiStatus = WiFi.status();
      Serial.println(Get_WiFiStatus(WiFiStatus));
      
      if((millis() - startConnTime) > netTimeOut) {
        while(digitalRead(BUTTON_PIN) == HIGH) {
          // Continue?
          // Press \/
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("No WiFi!");
          lcd.setCursor(0,1);
          lcd.print("Offline?");
          delay(150);
          
          // If user does not prompt to continue
          if((millis() - startConnTime) > (2*netTimeOut)) {
            ESP.restart();
          }
        }
        // If prompted by pressing button
        status_online = false;
        beep();
      }
  }

  // [2] Get time via Wifi Client or Random
  if(status_online) {
    HTTPClient http;
    String url = "https://www.timeapi.io/api/Time/current/zone?timeZone=Asia/Singapore";
    http.begin(url.c_str());
    int httpCode = http.GET();
    
    if (httpCode > 0) {
      String payload = http.getString();
      // Parse JSON
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print(F("$> [!] deserializeJson() failed: "));
        Serial.println(error.f_str());
        startTime = random(0, 1000);
      } else {
        String year = doc["year"];
        String mon = doc["month"];
        String day = doc["day"];
        String hour = doc["hour"];
        String mins = doc["minute"];
        String value = doc["dateTime"];
        startTime = year + mon + day + hour + mins;
        Serial.print("$> [v] Time: ");
        Serial.println(value);
      }
    } else {
      Serial.println(F("$> [!] Error on HTTP request"));
      startTime = random(0, 1000);
    }

    http.end();
  } else {
    startTime = random(0, 1000);
  }

  // [3] Set LINE Token
  LINE.setToken(LINE_TOKEN);
  

  /*------------------------------ SD CARD AND FILE SYSTEM INIT AND TEST ------------------------------*/
  // [1] Check File System
  if(!SPIFFS.begin()){
    Serial.println(F("$> [!] An Error has occurred while mounting SPIFFS"));
    ESP.restart();
    return;
  }

  // [2] Check SD Module
  if(!SD.begin()){
      Serial.println(F("$> [!] Card Mount Failed"));
      return;
  } else {
    // Continue?
    // Press \/
    
    status_SD = false;
  }

  // [3] Check SD Card
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println(F("$> [!] No SD card attached"));
      // Continue?
      // Press \/
      
      status_SD = false;
  } else {
    status_SD = true;
  }

  /**
   * Variable Configuration
   * t1 = DS; t2 h2 = Box; t3 h3 = Env;
  **/
  if(status_SD) {
    filename = "/" + startTime + ".csv";
    writeFile(SD, filename.c_str(), "time, t1, t2, t3, h2, h3");
  }
  
  /*------------------------------ SERVER INIT AND TEST ------------------------------*/
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });

  server.serveStatic("/", SPIFFS, "/");

  //Send JSON Sampling
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    String jsonResponse = getSensorDataJson();
    request->send(200, "application/json", jsonResponse);
  });

  //Send System Check
  server.on("/system", HTTP_GET, [](AsyncWebServerRequest *request){
    String jsonResponse = getSystemJson();
    request->send(200, "application/json", jsonResponse);
  });
  //Update Motor PWM Value through HTTP Request
  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String message;
    if (request->hasParam(input_parameter)) {
      message = request->getParam(input_parameter)->value();
      slider_value = message;
      Serial.print(F("$> [>>] Speed ="));
      Serial.println(slider_value);
      ledcWrite(pwm_channel, slider_value.toInt());
    }
    else {
      message = "No message sent";
    }
    Serial.println(message);
    request->send(200, "text/plain", "Updating Speed!");
  });
  
  server.begin();
//  data_sampling();
}
 
void loop(){
  if(digitalRead(BUTTON_PIN) == LOW) {
    Serial.println(F("$> [v] Set Mode"));
    delay(500);
    int dutyCycle = 0;
    while(1) {
      int potReading = analogRead(POT_PIN);
      potReading = map(potReading, 0, 4095, 0, 100);
      dutyCycle = 210 - 1.5 * potReading;
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Set Speed:");
      lcd.setCursor(0,1);
      lcd.print(potReading);
      lcd.print("%");

      if(digitalRead(BUTTON_PIN) == LOW) break;
      delay(100);
    }
    ledcWrite(pwm_channel, dutyCycle);
  } 
  
  if ((millis() - lastTime) > timerDelay) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Status:");
    
    data_sampling();
    lastTime = millis();
  }
  
  if(!queueBatch.isEmpty()){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Status:");
     
    batch_record();
  }

  if((millis() - lastBlink) > blinkDelay) {
    displayStat();
    lastBlink = millis();  
  }
  
  delay(50);
}
