#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <time.h>
#include <FS.h>
#include <LittleFS.h>
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <CertStoreBearSSL.h>
#include <ESP8266httpUpdate.h>

#define APP_BUILD "webreathe-standard-1.0"
#define APSSID  "we-breathe.org"
#define APPSK  "12345678"
#define APPHOST "https://monitor.we-breathe.org"
#define USING_HTTPS 

#define WIFI_MODE_AP 0
#define WIFI_MODE_CLIENT  1
#define WIFI_MAX_AP_TIMEOUT 30000
#define WIFI_MAX_RETRIES 3
#define REPORT_INTERVAL 30000

byte currentWiFiAP = 0;
byte currentWiFiRetryCycle = 0;
byte WIFI_MODE = WIFI_MODE_AP;

#define PMRXPin 13
#define PMTXPin 15

String ssids[6];
String psks[6];
String mac = "00:00:00:00:00:00";
unsigned long request_count = 0;

Adafruit_BMP280 bmp;
//ClosedCube_HDC1080 hdc;
SoftwareSerial pmSerial(PMRXPin, PMTXPin);

BearSSL::CertStore certStore;
#ifdef USING_HTTPS
BearSSL::WiFiClientSecure client;
#else
WiFiClient client;
#endif
HTTPClient https;

unsigned long lastWiFi = 0;
unsigned long lastPrint = 0;
unsigned long current = 0;
unsigned long lastReport = 0;
unsigned long lastNTPSync = 0;
boolean NTPSynced = false;
String uuid = "";
struct {
  unsigned long datetime = -1;

  float temperature;
  float humidity;
  float temperature_2nd;
  float pressure;
  float altitude;
  float pm25;
  unsigned long request_count;

  time_t timestamp;
  long lastUpdate = -1;

} measurements;

ESP8266WebServer server(80);

const int buttonPin = 0;     // the number of the pushbutton pin
volatile bool pressed = false;
volatile unsigned long lastPressStart = 0;

String urlencode(String str)
{
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      //encodedString+=code2;
    }
    yield();
  }
  return encodedString;
}

ICACHE_RAM_ATTR void ISR() {
  if (digitalRead(buttonPin) == LOW) {
    pressed = true;
    lastPressStart = millis();
  }
  if (digitalRead(buttonPin) == HIGH) {
    pressed = false;
  }
}

void syncWithNTP(bool from_sntp) {
  lastNTPSync = millis();
  NTPSynced = true;
  Serial.println("The NTP server was called!");
  struct tm timeinfo;
  time_t now = time(nullptr);
  gmtime_r(&now, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}

struct {
  String content;
  int code = 0;
} HTTPSResponse;

String HTTPSPOSTRequest(String url,  String data, String contentType){
  Serial.print("POST Request to send to ");Serial.println(url);
  if (https.begin(client, url)) { 
    https.addHeader("Content-Type", contentType);
    https.addHeader("User-Agent", "ESP8266");
    int httpsCode = https.POST(data);
    if (httpsCode > 0) {
      HTTPSResponse.code = httpsCode;  
      if (httpsCode == HTTP_CODE_OK) {
        HTTPSResponse.content = https.getString();
        Serial.print("Request finished OK. Response:"); Serial.println(HTTPSResponse.content);
      }
    } else { //error posting
      Serial.print("ERROR HTTPSPOSTRequest:Failed to POST data");
      HTTPSResponse.content = "";
    }
  } else {
    Serial.print("ERROR HTTPSPOSTRequest:failed to connect to server");
    HTTPSResponse.content = "";
  }
  https.end();
  return HTTPSResponse.content;
}

void initHDC(){
  //Configure HDC1080
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  Wire.write(0x90);
  Wire.write(0x00);
  Wire.endTransmission();

}

boolean readHDCSensor(double* temperature, double* humidity)
{
  //holds 2 bytes of data from I2C Line
  uint8_t Byte[4];

  //holds the total contents of the temp register
  uint16_t temp;

  //holds the total contents of the humidity register
  uint16_t humid;
  
  //Point to device 0x40 (Address for HDC1080)
  Wire.beginTransmission(0x40);
  //Point to register 0x00 (Temperature Register)
  Wire.write(0x00);
  //Relinquish master control of I2C line
  //pointing to the temp register triggers a conversion
  Wire.endTransmission();
  
  //delay to allow for sufficient conversion time
  delay(20);
  
  //Request four bytes from registers
  Wire.requestFrom(0x40, 4);

  delay(1);
  
  //If the 4 bytes were returned sucessfully
  if (4 <= Wire.available())
  {
    //take reading
    //Byte[0] holds upper byte of temp reading
    Byte[0] = Wire.read();
    //Byte[1] holds lower byte of temp reading
    Byte[1] = Wire.read();
    
    //Byte[3] holds upper byte of humidity reading
    Byte[3] = Wire.read();
    //Byte[4] holds lower byte of humidity reading
    Byte[4] = Wire.read();

    //Combine the two bytes to make one 16 bit int
    temp = (((unsigned int)Byte[0] <<8 | Byte[1]));

    //Temp(C) = reading/(2^16)*165(C) - 40(C)
    *temperature = (double)(temp)/(65536)*165-40;

   //Combine the two bytes to make one 16 bit int
    humid = (((unsigned int)Byte[3] <<8 | Byte[4]));

    //Humidity(%) = reading/(2^16)*100%
    *humidity = (double)(humid)/(65536)*100;
    return true;
  }
  return false;
}

void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes at %d...\n", cur, total, millis());
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void updateFirmware(String newVersion){
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    t_httpUpdate_return ret = ESPhttpUpdate.update(client, String(APPHOST)+"/firmware/"+newVersion);
    // Or:
    // t_httpUpdate_return ret = ESPhttpUpdate.update(client, "server", 80, "file.bin");

    switch (ret) {
      case HTTP_UPDATE_FAILED: Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str()); break;

      case HTTP_UPDATE_NO_UPDATES: Serial.println("HTTP_UPDATE_NO_UPDATES"); break;

      case HTTP_UPDATE_OK: Serial.println("HTTP_UPDATE_OK"); break;
    }

}

DynamicJsonDocument json(1024);
struct tm timeinfo;
time_t now;

void sendDataToWeBreathe() {
  request_count++;
  measurements.request_count=request_count;
  // If you need an HTTP request with a content type: application/json, use the following:
  json.clear();
  json["device"] = uuid;
  json["temperature"] = measurements.temperature;
  json["humidity"] = measurements.humidity ;
  json["temperature_2nd"] = measurements.temperature_2nd ;
  json["pressure"] = measurements.pressure ;
  json["altitude"] = measurements.altitude;
  json["pm25"] = measurements.pm25;
  json["requests"] = measurements.request_count;

  now = time(nullptr);
  gmtime_r(&now, &timeinfo);

  char yearStr[7];
  itoa(timeinfo.tm_year + 1900,yearStr,10);
  char monthStr[3];
  itoa(timeinfo.tm_mon+1,monthStr,10);
  char dayStr[3];
  itoa(timeinfo.tm_mday,dayStr,10);
  char hourStr[3];
  itoa(timeinfo.tm_hour,hourStr,10);
  char minStr[3];
  itoa(timeinfo.tm_min,minStr,10);
  char secStr[3];
  itoa(timeinfo.tm_sec,secStr,10);
  char timeStr[32]="";
  strcat(timeStr,yearStr);
  strcat(timeStr,"-");
  if (timeinfo.tm_mon + 1 < 10)
    strcat(timeStr,"0");
  strcat(timeStr,monthStr);
  strcat(timeStr,"-");
  if (timeinfo.tm_mday < 10)
    strcat(timeStr, "0");
  strcat(timeStr,dayStr);
  strcat(timeStr, " ");
  if (timeinfo.tm_hour < 10)
    strcat(timeStr, "0");
  strcat(timeStr,hourStr);
  strcat(timeStr,":");
  if (timeinfo.tm_min < 10)
    strcat(timeStr, "0");
  strcat(timeStr,minStr);
  strcat(timeStr,":");
  if (timeinfo.tm_sec < 10)
    strcat(timeStr, "0");
  strcat(timeStr,secStr);
  strcat(timeStr,".000");
  json["timestamp"] = timeStr;
  
  char js[1024] = "";
  serializeJson(json, js);

  String r = HTTPSPOSTRequest(String(APPHOST)+"/device/postJson", "data="+urlencode(js), "application/x-www-form-urlencoded");  
  DynamicJsonDocument resp(1024); 
  DeserializationError err = deserializeJson(resp, r);
  if(!err){
    if(resp.containsKey("firmware") &&resp["firmware"]!=nullptr && resp["firmware"]!=String(APP_BUILD) ){
      Serial.print("Updating firmware. Current version:");
      Serial.print(String(APP_BUILD));
      Serial.print(" Returned version:");
      Serial.println((String)resp["firmware"]);
      updateFirmware((String)resp["firmware"]);
    }
  }
  
}


void handleRoot() {
  String s = "<html><body><h1>You are connected we-breathe.org station.</h1><form action=\"/config\" method=\"post\"><h2>WiFi access points available to connect to:</h2><table>";
  for (int i = 1; i <= 5; i++) {
    s += "<tr><td>SSID " + String(i) + "</td><td>Password " + String(i) + "</td></tr>";
    s += "<tr><td><input type=\"text\" name=\"ssid" + String(i) + "\" size=\"20\"></td><td><input type=\"text\" name=\"pw" + String(i) + "\" size=\"20\"></td></tr>";
  }
  s += "</table><input type=\"submit\" value=\"Set WiFi\"></form><table><h2>Station current configuration</h2>";
  s += "<form action=\"/reset\" method=\"post\"><table><tr><td>UUID:</td><td>" + uuid + "&nbsp;</td></tr></table><input type=\"submit\" value=\"Reset config\"></form></body></html>";
  server.send(200, "text/html", s);
}

void handleConfigReset() {
  Serial.println("Config reset requested");
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else {
    LittleFS.remove("/config");
    LittleFS.remove("/wifi");
    String message = "Data reset\n";
    server.send(200, "text/plain", message);
  }
}

void handleWiFiSetup() {
  Serial.println("WiFi setup requested");
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else {
    String message = "POST form was:\n";
    StaticJsonDocument<512> doc;

    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
      if (server.arg(i) != "" && server.argName(i).indexOf("ssid") == 0) {
        String ids = server.argName(i).substring(4);
        int id = ids.toInt();
        if (!isnan(id))
//          strcpy(ssids[id],server.arg(i));
        ssids[id] = server.arg(i);
      }
      if (server.arg(i) != "" && server.argName(i).indexOf("pw") == 0) {
        String ids = server.argName(i).substring(2);
        int id = ids.toInt();
        if (!isnan(id))
//          strcpy(psks[id],server.arg(i));
          psks[id]=server.arg(i);
      }
    }
    LittleFS.remove("/wifi");
    bool wifiFound = false;
    for (int i = 1; i <= 5; i++) {
      String s = "ssid" + String(i);
      String p = "psk" + String(i);
      if (ssids[i] != "") {
        doc[s] = ssids[i];
        wifiFound = true;
        if (psks[i] == "")
          doc[p] = "";
        else
          doc[p] = psks[i];
        Serial.print((String)doc[s]); Serial.print(" - "); Serial.print((String)doc[p]);
      }
    }
    if (wifiFound) {
      File f = LittleFS.open("/wifi", "w");
      if (serializeJson(doc, f) == 0) {
        Serial.println(F("Failed to write config"));
        server.send(500, "text/plain", "Failed to write config");
      } else {
        server.send(200, "text/plain", message);
      }
      f.close();
      delay(3000);
      ESP.reset();
    }
  }
}

void setup() {

  //Serial init
  Serial.begin(115200);
  delay(2000);
  Wire.begin();
  Wire.setClock(100000L);
  ESPhttpUpdate.setClientTimeout(10000);
  Serial.print("Station startup:");
  Serial.println(APP_BUILD);
  WiFi.persistent(false);


  //button press setup
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), ISR, CHANGE);

  //filesystem initialization
  if (LittleFS.begin()) {
    Serial.println("Filesystem initialized");
  } else {
    Serial.println("ERROR:Filesystem cannot be initialized");
    delay(3000);
    ESP.reset();
  }

  //certificate init
  int numCerts = certStore.initCertStore(LittleFS, PSTR("/certs.idx"), PSTR("/certs.ar"));
  Serial.print(F("Number of CA certs read: "));
  Serial.println(numCerts);
  if(numCerts == 0) {
    Serial.println(F("ERROR: No certs found. Filesystem don't contains required files"));
    delay(10000);
    ESP.reset();
  }
#ifdef USING_HTTPS
  client.setCertStore(&certStore);
#endif

  //pressure sensor init
  int status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("ERROR:Pressure sensor not found"));
    delay(3000);
    ESP.reset();
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //temp&humidity sensor
  initHDC();
  //hdc.begin(0x40);
    //Serial.println(F("ERROR:Couldn't find temperature&humidity sensor!"));
    //delay(3000);
    //ESP.reset();
  //}

  //particulate matter sensor init
  pinMode(PMRXPin, INPUT);
  pinMode(PMTXPin, OUTPUT);
  pmSerial.begin(9600);
  Serial.println("Particulate Matter sensor initialized");
  
  //read mac address
  mac = WiFi.macAddress();

  //WIFI init
  WIFI_MODE = WIFI_MODE_AP;
  bool wifiDetected = false;
  //LittleFS.remove("/wifi");
  if (LittleFS.exists("/wifi")) {
    File f = LittleFS.open("/wifi", "r");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, f);
    if (error) {
      Serial.println(F("Failed to read config file, using default configuration"));
      LittleFS.remove("/wifi");
      delay(3000);
      ESP.reset();
    }

    for (int i = 1; i <= 5; i++) {
      String ssid = "ssid";
      ssid = ssid + String(i);
      String psk = "psk";
      psk = psk + String(i);
      if (doc.containsKey(ssid)) {
        if (doc.containsKey(psk)) {
          if (!wifiDetected)
            WiFi.mode(WIFI_STA);
          wifiDetected = true;
          WIFI_MODE = WIFI_MODE_CLIENT;
          ssids[i - 1] = (String)doc[ssid];
          psks[i - 1] = (String)doc[psk];
          Serial.print("Added SSID:"); Serial.print((String)doc[ssid]); Serial.print(" PSK:"); Serial.println("*********");//String(doc[psk]);
        }
      }
    }
  }

  if (WIFI_MODE == WIFI_MODE_AP) {
    Serial.println("Station in AP mode ... connect to we-breathe.org AP and access configuration");
    String ssidap = String(APSSID)+" "+String(mac);
    boolean resap = WiFi.softAP(ssidap, APPSK);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.on("/", handleRoot);
    server.on("/config", handleWiFiSetup);
    server.on("/reset", handleConfigReset);
    server.begin();
    Serial.println("HTTP server started");
  } else {
    WiFi.begin(ssids[currentWiFiAP], psks[currentWiFiAP]);
    settimeofday_cb(syncWithNTP);
    configTime(0, 0, "ro.pool.ntp.org", "ro.pool.ntp.org");  // UTC
  }

  if (LittleFS.exists("/config")) {
    File f = LittleFS.open("/config", "r");
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, f);
    if (error) {
      Serial.println(F("Unable to read config file"));
      LittleFS.remove("/config");
      delay(3000);
      ESP.reset();
    }

    if (doc.containsKey("uuid")) {
      uuid = (String)doc["uuid"];
      Serial.println("Device has uuid saved:" + uuid);
    }
    f.close();

  } else { // the file is not there
    Serial.println("Config file don't exists");
  }
  Serial.println("Finished starting");
}


void tryRegistration() {
  mac.replace(":", "%3A");
  String httpData = "name=we-breathe.org%20device%20" + mac+"&firmwareUpdate=true&firmwareVersion="+APP_BUILD;

  String path = "/device/register";
  
  Serial.println("URL post:" + String(APPHOST) + path + " data:" + httpData);
  if (https.begin(client, String(APPHOST) + path+"?"+httpData)) {
    //https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    https.addHeader("User-Agent", "ESP8266"); 
    if(String(APPHOST).startsWith("https"))
      https.addHeader("Host", String(APPHOST).substring(8));
    else 
      https.addHeader("Host", String(APPHOST).substring(7));

    int httpsCode = https.GET();
    if (httpsCode > 0) {
      Serial.println(httpsCode);
      if (httpsCode == HTTP_CODE_OK) {
        String tmpuuid = https.getString();
        Serial.println("UUID received:" + tmpuuid);
        StaticJsonDocument<128> doc;
        doc["uuid"] = tmpuuid;
        File f = LittleFS.open("/config", "w");

        if (serializeJson(doc, f) == 0) {
          Serial.println(F("ERROR:Unable to write config file."));
          delay(3000);
          ESP.reset();
        } else {
          Serial.println("Device registered with UUID:" + tmpuuid);
          uuid = tmpuuid;
          f.close();
          delay(3000);
          ESP.reset();
        }
      }
    } else { //error posting
      Serial.print("ERROR:Failed to POST data");
      delay(3000);
      ESP.reset();
    }
  } else {
    Serial.print("failed to connect to server");
    delay(3000);
    ESP.reset();
  }
  https.end();
  client.stop();
}

void readSensors() {
  double tempv, humidv;
  if(readHDCSensor(&tempv, &humidv)){
    measurements.temperature = tempv;
    measurements.humidity = humidv;
        
  } else {
    Serial.print("Unable to read HDC1080 "); Serial.println(millis());
  }
//  double tmpv = hdc.readTemperature();
//  if(hdc.validRead)
//    measurements.temperature = tmpv;
  //else
    //hdc.reset();
//  tmpv = hdc.readHumidity();
//  if(hdc.validRead)
//    measurements.humidity = tmpv;
  //else
    //hdc.reset();
  measurements.temperature_2nd = bmp.readTemperature();
  measurements.pressure = bmp.readPressure();
  measurements.altitude = bmp.readAltitude(1021);
}

void printMeasurements() {
  lastPrint = millis();
  Serial.print("Temperature: "); Serial.println(measurements.temperature);
  Serial.print("Humidity: "); Serial.println(measurements.humidity);
  Serial.print(F("Temperature 2: "));
  Serial.println(measurements.temperature_2nd);
  Serial.print(F("Pressure: "));
  Serial.println(measurements.pressure);
  Serial.print(F("Approx altitude: "));
  Serial.println(measurements.altitude);
  Serial.print("PM2.5:");
  Serial.println(measurements.pm25);
  Serial.print("Requests:");
  Serial.println(measurements.request_count);
  Serial.println();
  if (WIFI_MODE == WIFI_MODE_AP) {
    Serial.println("Station is in AP mode. Connect to we-breathe.org AP and access configuration.");
  }
}

void readPM() {
  while (pmSerial.available()) {
    int i = 0;
    int bytes[9];
    while (pmSerial.available() && i < 9) {
      bytes[i] = pmSerial.read();
      i++;
    }
    if (i == 9) {
      if (bytes[0] == 255 && bytes[1] == 24 && bytes[2] == 0) {
        float pm = bytes[4];
        if (bytes[4] > 99)
          pm = bytes[4] / 1000.0;
        else if (bytes[4] > 9)
          pm = bytes[4] / 100.0;
        else
          pm = bytes[4] / 10.0;
        pm = pm + bytes[3];
        measurements.pm25 = pm;
      }
    }
  }
}


void resetData() {
  LittleFS.remove("/config");
  LittleFS.remove("/wifi");
  Serial.println("Data reset requested");
  delay(1000);
  ESP.reset();
}

void loop() {
  if (WIFI_MODE == WIFI_MODE_AP) { //AP MODE
    //in case it's in AP mode process requests
    server.handleClient();
  } else { // NORMAL MODE
    current = millis();
    //read serial if available
    //Read PM first increasing chances of no interrupts when hdc is read  
    if (pmSerial.available()) {
      readPM();
    }
    //read i2c sensor values
    noInterrupts();
    readSensors();
    interrupts();
    if (current > lastPrint + REPORT_INTERVAL) {
      printMeasurements();
    }
    if (pressed && millis() > lastPressStart + 5000) {
      resetData();
    }
    if (WiFi.status() != WL_CONNECTED) { // not connected to wifi yet
      if (current > lastWiFi + WIFI_MAX_AP_TIMEOUT) {
        Serial.print("WiFi AP SSID:"); Serial.print(ssids[currentWiFiAP]); Serial.println(" timed out.");
        lastWiFi = current;
        currentWiFiAP++;
        if (currentWiFiAP == 5) {
          currentWiFiRetryCycle++;
          currentWiFiAP = 0;
          Serial.print("All WiFi AP's tried. Cycle:"); Serial.println(currentWiFiRetryCycle + 1);
        }
        WiFi.begin(ssids[currentWiFiAP], psks[currentWiFiAP]);
      }
    } else { //we have wifi
      lastWiFi = current;
      if (uuid == "" && NTPSynced) {
        tryRegistration();
      } else {
        //send data
        if (NTPSynced && current < lastNTPSync + 12 * 3600 * 1000 && current > lastReport + REPORT_INTERVAL) {
          lastReport = current;
          sendDataToWeBreathe();
        }
      }
    }
  }
}


