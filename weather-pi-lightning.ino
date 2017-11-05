#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <AS3935.h>

#define IRQ_PIN 2

volatile bool detected = false;

const char* ssid         = "xxxxxxxx";
const char* password     = "xxxxxxxx";
const char* mqttServer   = "xxxxxxxx";
const char* mqttUser     = "xxxxxxxx";
const char* mqttPassword = "xxxxxxxx";

const String place = "LIGHTNING";
const String sensordata = "sensordata";
const String typeDistance = "DISTANCE";


char json[256];

WiFiClient espClient;
PubSubClient client(espClient);

DynamicJsonBuffer jsonBuffer;
JsonObject& obj = jsonBuffer.createObject();
    
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
  while (!client.connected()) {
    tries--;
    if (tries == 0) {
      return false;
    }
    if (!client.connect(place.c_str(), mqttUser, mqttPassword)) {
      delay(5000);
    }
  }
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
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Welcome to the MOD-1016 (AS3935) Lightning Sensor test sketch!");
  Serial.println("Embedded Adventures (www.embeddedadventures.com)\n");

  //I2C
  Wire.begin();
  mod1016.init(IRQ_PIN);
  
 
  //Tune Caps, Set AFE, Set Noise Floor
  //autoTuneCaps(IRQ_PIN);
  
  mod1016.setTuneCaps(5);
  mod1016.setOutdoors();
  mod1016.setNoiseFloor(5);
  
  
  Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");

  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
  mod1016.getIRQ();

  Serial.println("connect to WIFI...");
  // connect to Wifi
  wifiConnect();

  obj.set("place", place);
}

void loop() {
  if (detected) {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }
  client.loop();
}

void alert() {
  detected = true;
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE DETECTED");
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        break;
      case 8: 
        Serial.println("LIGHTNING DETECTED");
        printDistance();
        break;
    }
}

void printDistance() {
  int distance = mod1016.calculateDistance();
  if (mqttConnect()) {
    sendAS3935Distance(obj,distance);
  }
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");  
  }
}

