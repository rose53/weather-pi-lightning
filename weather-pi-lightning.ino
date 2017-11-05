#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_HTU21DF.h>
#include "AS3935.h"
#include "icons.h"

#define IRQ_PIN 4
#define OLED_RESET 0

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

volatile bool readyForHTUUpdate = true;

Ticker htuUpdateTicker;
Ticker currentDisplayTicker;
Ticker disturberTicker;

const char* ssid         = "Utgard";
const char* password     = "franziundrose";
const char* mqttServer   = "192.168.0.42";
const char* mqttUser     = "weatherpi";
const char* mqttPassword = "weatherpi";

const String place = "LIGHTNING";
const String sensordata = "sensordata";
const String typeDistance = "DISTANCE";

unsigned long          previousMillis  = 0;     




WiFiClient espClient;
PubSubClient client(espClient);

char json[256];
DynamicJsonBuffer jsonBuffer;
JsonObject& obj = jsonBuffer.createObject();

// used to store the data that should be displayed
struct displayData_t {
    float temperature;
    float humidity;
    bool  detected;
    int   distance;
    bool  disturber;
    bool  wifiConnected;
    bool  mqttConnected;
};    

volatile displayData_t displayData = { 0.0, 0.0, false, 1, false, false, false};

const int    displaysCount      = 2;
volatile int currentDisplayIndx = 0;

const int icon_wifi_pos_x = 0;  
const int icon_wifi_pos_y = 1; 
const int icon_data_pos_x = icon_wifi_pos_x;  
const int icon_data_pos_y = icon_wifi_pos_y + 22;  
const int icon_disturber_pos_x = icon_wifi_pos_x;  
const int icon_disturber_pos_y = icon_data_pos_y + 22;  
const int data_pos_x = 30;

void displayTemperature() {
    display.setFont(&FreeSans24pt7b);
    display.setCursor(data_pos_x,(display.height() + 24) / 2);
    display.setTextSize(1);
    display.print(displayData.temperature,1);
    display.drawBitmap(icon_data_pos_x,icon_data_pos_y, temperature_icon_bmp, temperature_icon_width, temperature_icon_height, WHITE); 
    display.setFont();
}

void displayHumidity() {
    display.setFont(&FreeSans24pt7b);
    display.setCursor(data_pos_x,(display.height() + 24) / 2);
    display.setTextSize(1);
    display.print(displayData.humidity,0);
    display.drawBitmap(icon_data_pos_x,icon_data_pos_y, humidity_icon_bmp, humidity_icon_width, humidity_icon_height, WHITE); 
    display.setFont();
}

void displayLightning() {
}
    


void (*displays[displaysCount])(void)= {displayTemperature, displayHumidity};

void updateDisplay() {
    
    display.clearDisplay();
    
    if (displayData.wifiConnected) {        
        display.drawBitmap(icon_wifi_pos_x,icon_wifi_pos_y, wifi_icon_bmp, wifi_icon_width, wifi_icon_height, WHITE);    
    } else {
        display.drawBitmap(icon_wifi_pos_x,icon_wifi_pos_y, no_wifi_icon_bmp, no_wifi_icon_width, no_wifi_icon_height, WHITE);    
    }    

    if (displayData.disturber) {        
        display.drawBitmap(icon_disturber_pos_x,icon_disturber_pos_y, disturber_icon_bmp, disturber_icon_width, disturber_icon_height, WHITE);    
    } 
    
    (*displays[currentDisplayIndx])();
    
    display.display();   
}

void readHTUData() {
    displayData.temperature = htu.readTemperature();
    displayData.humidity    = htu.readHumidity();
}

/**************************************************************************/
/*
    Try to connect to the WIFI, after 10 tries, we do a deep sleep
*/
/**************************************************************************/
boolean wifiConnect(void) {
  WiFi.begin(ssid, password);
  // Wait for connection
  int tries = 10;
  boolean retVal = true;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tries--;
    if (tries <= 0) {
      // got no connection to WiFi, going to sleep
      return false;
    }
  }
  return retVal;
}

/**************************************************************************/
/*
    Try to connect to the MQTT broker, after 10 tries, we return false
*/
/**************************************************************************/
boolean mqttConnect(void) {
  client.setServer(mqttServer, 1883);

  int  tries = 10;
  displayData.mqttConnected = false;
  while (!client.connected()) {
    tries--;
    if (tries == 0) {
      return false;
    }
    if (!client.connect(place.c_str(), mqttUser, mqttPassword)) {
      delay(5000);
    }
  }
  displayData.mqttConnected = true;
  return true;
}

void sendAS3935Distance(JsonObject& jsonObject, int distance) {
  
  String topic = sensordata + "/" + place + "/" + typeDistance;
  topic.toLowerCase();
  
  jsonObject.set("sensor", "AS3935");
  jsonObject.set("type", typeDistance);
  jsonObject.set("distance", distance);

  jsonObject.printTo(json, sizeof(json));
  client.publish(topic.c_str(), json);
  jsonObject.remove("distance");
  jsonObject.remove("type");
  jsonObject.remove("sensor");
  delay(250);
}
void setup() {

    //I2C
    Wire.begin();
  
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
    display.clearDisplay();
    display.display();

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("AS3935");

    display.display();
  
    htu.begin();

    // initialize the AS3935
    mod1016.init(IRQ_PIN);    
    mod1016.setTuneCaps(5);  // 
    mod1016.setIndoors();
    //mod1016.setOutdoors();
    mod1016.setNoiseFloor(3);
    //mod1016.setMinimumLightnings(MIN_NUM_LGHT_5);
    
    display.print("TUNE        :");
    display.println(mod1016.getTuneCaps(), HEX);
    display.print("IN/OUT      :");
    display.println(mod1016.getAFE(), BIN);
    display.print("NOISEFLOOR  :");
    display.println(mod1016.getNoiseFloor(), HEX);
    display.print("MIN_NUM_LGHT:");
    display.println(mod1016.getMinimumLightnings(),HEX);

    display.display();
    pinMode(IRQ_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
    mod1016.getIRQ();

    display.print("connect to WIFI...");
    display.display();
    // connect to Wifi
    displayData.wifiConnected = wifiConnect();
    updateDisplay();
    display.setTextSize(1);
    obj.set("place", place);

    htuUpdateTicker.attach(30, setReadyForHTUUpdate);
    currentDisplayTicker.attach(2,nextDisplay);
}

/**
 * Functions used by the different ticker
 */
void setReadyForHTUUpdate() {
    readyForHTUUpdate = true;  
}

void nextDisplay() {
    currentDisplayIndx = (currentDisplayIndx + 1) % displaysCount;
}

void resetDisturber() {
    displayData.disturber = false;
}

void loop() {
    unsigned long currentMillis = millis();
    if (displayData.detected) {
      translateIRQ(mod1016.getIRQ());
      displayData.detected = false;
      previousMillis = currentMillis;    
    }
    client.loop();

  
    if (currentMillis - previousMillis >= 2000) {
        updateDisplay();
        previousMillis = currentMillis;    
    }

    if (readyForHTUUpdate) {
        readHTUData();
        readyForHTUUpdate = false;    
    }

}

// gets called from the AS3935 interrupt 
void alert() {
  displayData.detected = true;
}

void translateIRQ(byte irq) {
    displayData.disturber = false;
    switch(irq) {
      case 1:
        display.setCursor(0,30);
        display.println("                   ");
        display.println("NOISE DETECTED    ");
        break;
      case 4:
        disturberTicker.detach();
        displayData.disturber = true;
        disturberTicker.once(10, resetDisturber);
        break;
      case 8: 
        display.setCursor(0,30);
        display.println("                   ");
        display.println("LIGHTNING DETECTED");
        printDistance();
        break;
    }
    display.display();
}

void printDistance() {
  displayData.distance = mod1016.calculateDistance();
  if (mqttConnect()) {
    sendAS3935Distance(obj,displayData.distance);
  }
  if (displayData.distance == -1)
    display.println("Lightning out of range");
  else if (displayData.distance == 1)
    display.println("Distance not in table");
  else if (displayData.distance == 0)
    display.println("Lightning overhead");
  else {
    display.print("Lightning ~");
    display.print(displayData.distance);
    display.println("km away\n");  
  }


}



