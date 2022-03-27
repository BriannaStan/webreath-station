#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>

#include <LiquidCrystal_I2C.h> 
#include <sps30.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>

#include <ArduinoJson.h>

#include <LittleFS.h>
#include <time.h>

// Replace with your network credentials
const char *ssid     = "WIFIID";
const char *password = "WIFIPW";
String serverName = "http://monitor.we-breathe.org";
String uuid="";
WiFiClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

Adafruit_BME280 bme;
Adafruit_CCS811 ccs;
LiquidCrystal_I2C lcd(0x27, 20, 4);

#define SEALEVELPRESSURE_HPA (1032)
#define auto_clean_days 4
#define REPORT_INTERVAL 30000
#define DISPLAY_INTERVAL 5000 

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

String urlPostRequest(String url, String data, String contentType="application/x-www-form-urlencoded"){  
  String payload = "";
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;

      // Your Domain name with URL path or IP address with path
      http.begin(client, url.c_str());
      // Specify content-type header
      http.addHeader("Content-Type", contentType);
      
      // Send HTTP POST request
      int httpResponseCode = http.POST(data);
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    } else {
      Serial.println("Not connected to internet...");
    }
    return payload;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  lcd.init();
  lcd.backlight();
  
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  timeClient.begin();
  timeClient.setTimeOffset(0);
  timeClient.update();

    //initialize filesystem
  if(LittleFS.begin()){
    Serial.println("Filesystem mounted");
    //LittleFS.remove("/register.txt");
    // registered.txt will be the file that will keep the uuid of the device
    //if this file exists we read it into global var uuid;
    if (LittleFS.exists("/register.txt")){
      Serial.println("File register.txt exists");
      File f = LittleFS.open("/register.txt","r");
      if(f){
        uuid = "";
        while(f.available())
          uuid += f.readString();
        f.close();
        Serial.println("Device has uuid saved:"+uuid);
      } else {
        Serial.println("Problem opening file register.txt");
      }
  
    } else { // the file is not there
      Serial.println("The file don't exists");
      String mac = WiFi.macAddress();
      mac.replace(":","%3A");
      String httpData = "name=Device%20"+mac;
      Serial.println("URL post:"+serverName+"/device/register"+" data:"+httpData);

      uuid = urlPostRequest(serverName+"/device/register", httpData);
      if(uuid=="") {
        
      } else {
        File f = LittleFS.open("/register.txt","w+");
        f.print(uuid);
        f.close();
        Serial.println("Device registered with uuid:"+uuid);
      }
    }
    LittleFS.end(); // we don't need fs anymore
  }else{
    Serial.println("An Error has occurred while mounting LittleFS");
    while(1) delay(500);
  }

  
  //bme280
  if (!bme.begin(0x77)) {
    Serial.println("BME280 error starting! Please check your wiring.");
    while (1) delay(500);
  }
  Serial.println("BME280 started.");
  
  //ccs811
  if(!ccs.begin(0x5b)){    
    Serial.println("CCS811 error starting! Please check your wiring.");
    while (1) delay(500);
  }
  while(!ccs.available())
    delay(10);
  Serial.println("CCS811 started.");

  //sps30 probe
  while (sps30_probe() != 0) {
    Serial.print("SPS30 sensor probing failed\n");
    while (1) delay(500);
  }
  #ifndef PLOTTER_FORMAT
    Serial.print("SPS30 sensor probing successful\n");
  #endif /* PLOTTER_FORMAT */
  int ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    Serial.print("SPS30 error setting the auto-clean interval: ");
    Serial.println(ret);
  }
  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("SPS30 error starting measurement\n");
    while(1) delay(500);
  }
  #ifndef PLOTTER_FORMAT
  Serial.print("SPS30 measurements started\n");
  #endif /* PLOTTER_FORMAT */
  #ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
    Serial.print("Your Arduino hardware has a limitation that only\n");
    Serial.print("  allows reading the mass concentrations. For more\n");
    Serial.print("  information, please check\n");
    Serial.print("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
    Serial.print("\n");
    delay(2000);
  #endif
  
  delay(1000);
  
}

struct {
  int eCO2=-1;
  int TVOC = -1;
  unsigned long datetime = -1;
  struct sps30_measurement sps30;
  float temperature;
  float pressure;
  float altitude;
  float humidity;
  long lastUpdate = -1;
} measurements;

void printMeasurements(){
    Serial.print("CO2: ");
    Serial.println(measurements.eCO2);
    Serial.print("TVOC: ");
    Serial.println(measurements.TVOC);
    Serial.print("Temperature: ");
    Serial.println(measurements.temperature);
    Serial.print("Humidity: ");
    Serial.println(measurements.humidity);
    Serial.print("Pressure: ");
    Serial.println(measurements.pressure);
    Serial.print("Altitude: ");
    Serial.println(measurements.altitude);
    Serial.print("PM  1.0: ");
    Serial.println(measurements.sps30.mc_1p0);
    Serial.print("PM  2.5: ");
    Serial.println(measurements.sps30.mc_2p5);
    Serial.print("PM  4.0: ");
    Serial.println(measurements.sps30.mc_4p0);
    Serial.print("PM 10.0: ");
    Serial.println(measurements.sps30.mc_10p0);
    Serial.print("NC  0.5: ");
    Serial.println(measurements.sps30.nc_0p5);
    Serial.print("NC  1.0: ");
    Serial.println(measurements.sps30.nc_1p0);
    Serial.print("NC  2.5: ");
    Serial.println(measurements.sps30.nc_2p5);
    Serial.print("NC  4.0: ");
    Serial.println(measurements.sps30.nc_4p0);
    Serial.print("NC 10.0: ");
    Serial.println(measurements.sps30.nc_10p0);  
    Serial.print("Typical partical size: ");
    Serial.println(measurements.sps30.typical_particle_size);
    //measurements.datetime = millis();
}

void reportData(){
  timeClient.update();
  measurements.datetime = timeClient.getEpochTime();
}

int display=0;
int lastDisplay = -1;
void lcdDisplayRefresh(){
  lcd.clear();
  if(display==0){  
    display=1;
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(measurements.temperature);
    lcd.print("C H: ");
    lcd.print(measurements.humidity);
    lcd.print("%");
    lcd.setCursor(0,1);
    lcd.print("Pressure: ");
    lcd.print(measurements.pressure);
    lcd.print("hPa");
    lcd.setCursor(0,2);
    lcd.print("CO2: ");
    lcd.print(measurements.eCO2);
    lcd.print("ppm");
    lcd.setCursor(0,3);
    lcd.print("TVOC: ");
    lcd.print(measurements.TVOC);
    lcd.print("index");
  } else if(display==1){
    lcd.setCursor(0,0);
    lcd.print("PM 1.0: ");
    lcd.print(measurements.sps30.mc_1p0);
    lcd.setCursor(0,1);
    lcd.print("PM 2.5: ");
    lcd.print(measurements.sps30.mc_2p5);
    lcd.setCursor(0,2);
    lcd.print("PM 4.0: ");
    lcd.print(measurements.sps30.mc_4p0);
    lcd.setCursor(0,3);
    lcd.print("PM10.0: ");
    lcd.print(measurements.sps30.mc_10p0);
    display=2;
  }else if(display==2){
    lcd.setCursor(0,0);
    lcd.print("NC 1.0: ");
    lcd.print(measurements.sps30.nc_1p0);
    lcd.setCursor(0,1);
    lcd.print("NC 2.5: ");
    lcd.print(measurements.sps30.nc_2p5);
    lcd.setCursor(0,2);
    lcd.print("NC 4.0: ");
    lcd.print(measurements.sps30.nc_4p0);
    lcd.setCursor(0,3);
    lcd.print("NC10.0: ");
    lcd.print(measurements.sps30.nc_10p0);
    display=3;
  } else {
    lcd.setCursor(0,0);
    lcd.print("NC 0.5: ");
    lcd.print(measurements.sps30.nc_0p5);
    lcd.setCursor(0,1);
    lcd.print("Part. size: ");
    lcd.print(measurements.sps30.typical_particle_size);
    lcd.setCursor(0,2);
    lcd.print("Alt: ");
    lcd.print(measurements.altitude);
    lcd.setCursor(0,3);
    lcd.print("IP: ");
    lcd.print(WiFi.localIP());
    display=0;
  }
  lastDisplay=millis();
  
}

void sendDataToWeBreathe(){
      HTTPClient http;
      DynamicJsonDocument json(1024);
      
      // If you need an HTTP request with a content type: application/json, use the following:
      json["device"]=uuid;
      json["pm1.0"]=measurements.sps30.mc_1p0;
      json["pm2.5"]=measurements.sps30.mc_2p5;
      json["pm4.0"]=measurements.sps30.mc_4p0;
      json["pm10.0"]=measurements.sps30.mc_10p0;
      json["nc0.5"]=measurements.sps30.nc_0p5;
      json["nc1.0"]=measurements.sps30.nc_1p0;
      json["nc2.5"]=measurements.sps30.nc_2p5;
      json["nc4.0"]=measurements.sps30.nc_4p0;
      json["nc10.0"]=measurements.sps30.nc_10p0;
      json["temperature"]=measurements.temperature;
      json["pressure"] = measurements.pressure;
      json["altitude"]=measurements.altitude;
      json["humidity"]=measurements.humidity ;
      json["eco2"]=measurements.eCO2;
      json["tvoc"]=measurements.TVOC;


      time_t dt = timeClient.getEpochTime();
      String timeStr = timeClient.getFormattedTime();

      Serial.print("Epoch time:");
      Serial.println(dt);
      
      struct tm *ptm = gmtime ((time_t *)&dt); 
      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon+1;
      int currentYear = ptm->tm_year+1900;

      String yearStr = "";
      if (currentYear<10) yearStr+="000"+String(currentYear);
      else if (currentYear<100) yearStr+="00"+String(currentYear);
      else if (currentYear<1000) yearStr+="0"+String(currentYear);
      else yearStr = String(currentYear);

      String monthStr = "";
      if (currentMonth<10) monthStr+="0"+String(currentMonth);
      else monthStr = String(currentMonth);

      String dayStr = "";
      if (monthDay<10) dayStr+="0"+String(monthDay);
      else dayStr = String(monthDay);

      json["timestamp"] = yearStr + "-" + monthStr + "-" + dayStr+" "+timeStr+".000";
      String jsonData;
      serializeJson(json, jsonData);
      String httpData="data="+urlencode(jsonData);
      Serial.print("Json to post:");
      Serial.println(jsonData);
      Serial.print("Data to post:");
      Serial.println(httpData);
      String response = urlPostRequest(serverName+"/device/postJson",httpData);
           
      //Serial.print("HTTP Response: "+response);
}

void loop() {
  //start measurements
  
   if (ccs.available()) {
    if (!ccs.readData()) {
      measurements.eCO2 = ccs.geteCO2();
      measurements.TVOC = ccs.getTVOC();
    }
    else {
      measurements.eCO2 = -1;
      measurements.TVOC=-1;
      Serial.println("CCS811 error reading data!");
    }
  } else {
      measurements.eCO2 = -1;
      measurements.TVOC=-1;
      Serial.println("CCS811 error! Not avilable!");
  }

  int ret = sps30_read_measurement(&measurements.sps30);
  if (ret < 0) {
    measurements.sps30.mc_1p0 = -1;
    measurements.sps30.mc_2p5 = -1;
    measurements.sps30.mc_4p0 = -1;
    measurements.sps30.mc_10p0 = -1;
    Serial.print("SPS30 error reading measurement\n");
  } 

  measurements.temperature = bme.readTemperature();
  measurements.pressure = bme.readPressure()/100.0F;
  measurements.altitude =  bme.readAltitude(SEALEVELPRESSURE_HPA);
  measurements.humidity = bme.readHumidity();
  if(millis()-lastDisplay>=DISPLAY_INTERVAL){
    lcdDisplayRefresh();
  }

  if(millis()-measurements.lastUpdate>=REPORT_INTERVAL){
    reportData();  
    printMeasurements();
    sendDataToWeBreathe();
    measurements.lastUpdate=millis();
  }
  delay(1000);
}
