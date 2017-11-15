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
#include "credentials.h"

#define IRQ_PIN 4
#define OLED_RESET 0

enum Event {
  NOTHING,
  NOISE,
  DISTURBER,
  LIGHTNING
};

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

volatile bool readyForHTUUpdate = true;

Ticker htuUpdateTicker;
Ticker currentDisplayTicker;
Ticker irqDisplayTicker;


const char* place         = "LIGHTNING";
const char* sensordata    = "sensordata";
const char* typeLightning = "LIGHTNING";

const char* topic = "sensordata/lightning/lightning";

unsigned long          previousMillis  = 0;     


WiFiClient espClient;
PubSubClient client(espClient);

char json[256];

// used to store the data that should be displayed
struct displayData_t {
    float temperature;
    float humidity;
    bool  detected;
    int   distance;
    Event event;
    bool  wifiConnected;
    bool  mqttConnected;
};    

volatile displayData_t displayData = { 0.0, 0.0, false, 1, NOTHING, false, false};

const int    displaysCount      = 2;
volatile int currentDisplayIndx = 0;

const int icon_wifi_pos_x = 0;  
const int icon_wifi_pos_y = 1; 
const int icon_data_pos_x = icon_wifi_pos_x;  
const int icon_data_pos_y = icon_wifi_pos_y + 22;  
const int icon_disturber_pos_x = icon_wifi_pos_x;  
const int icon_disturber_pos_y = icon_data_pos_y + 22; 
const int icon_noise_pos_x = icon_wifi_pos_x;  
const int icon_noise_pos_y = icon_data_pos_y + 22;
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
    display.setFont(&FreeSans24pt7b);
    display.setCursor(data_pos_x,(display.height() + 24) / 2);
    display.setTextSize(1);
    display.print(displayData.distance);
    display.drawBitmap(icon_data_pos_x,icon_data_pos_y, lightning_icon_bmp, lightning_icon_width, lightning_icon_height, WHITE); 
    display.setFont();    

/*
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
  */
}
    


void (*displays[displaysCount])(void)= {displayTemperature, displayHumidity};

void updateDisplay() {
    
    display.clearDisplay();
    
    if (displayData.wifiConnected) {        
        display.drawBitmap(icon_wifi_pos_x,icon_wifi_pos_y, wifi_icon_bmp, wifi_icon_width, wifi_icon_height, WHITE);    
    } else {
        display.drawBitmap(icon_wifi_pos_x,icon_wifi_pos_y, no_wifi_icon_bmp, no_wifi_icon_width, no_wifi_icon_height, WHITE);    
    }    

    if (displayData.event == NOISE) {        
        display.drawBitmap(icon_noise_pos_x,icon_noise_pos_y, noise_icon_bmp, noise_icon_width, noise_icon_height, WHITE);    
    } else if (displayData.event == DISTURBER) {        
        display.drawBitmap(icon_disturber_pos_x,icon_disturber_pos_y, disturber_icon_bmp, disturber_icon_width, disturber_icon_height, WHITE);    
    } 

    if (displayData.event == LIGHTNING) {
        displayLightning();
    } else {
        (*displays[currentDisplayIndx])();    
    }
    
    
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
    if (!client.connect(place, mqttUser, mqttPassword)) {
      delay(5000);
    }
  }
  displayData.mqttConnected = true;
  return true;
}

void sendAS3935Distance(int distance) {

    StaticJsonBuffer<256> jsonBuffer;
    JsonObject&           root = jsonBuffer.createObject();

    root.set("place", place);
    root.set("sensor", "AS3935");
    root.set("type", typeLightning);
    root.set("distance", distance);

    root.printTo(json, sizeof(json));
    client.publish(topic, json);
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

void resetIRQDisplayData() {
    displayData.event = NOTHING;
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
    displayData.event = NOTHING;
    irqDisplayTicker.detach();
    switch(irq) {
      case 1:        
        displayData.event = NOISE;
        break;
      case 4:
        displayData.event = DISTURBER;
        break;
      case 8: 
        displayData.event = LIGHTNING;
        displayData.distance = mod1016.calculateDistance();
        if (mqttConnect()) {
            sendAS3935Distance(displayData.distance);
        }        
        break;
    }
    irqDisplayTicker.once(10, resetIRQDisplayData);
    updateDisplay();
}


