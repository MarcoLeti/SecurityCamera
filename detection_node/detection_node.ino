#include <WiFi.h>
#include <PubSubClient.h>
#include "credentials.h"
#define PIRPIN 19
#define BUTTON_PIN  21

// Define a static IP address for this Node
IPAddress staticIP(192, 168, 1, 160); 
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
PubSubClient client(espClient);
bool prev_motion_detected;

void setup() {
  Serial.begin(9600);
  pinMode(PIRPIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  connectToWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
 
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
}

void loop() {
  if (!client.connected()) {
    Serial.println("MQTT not connected, attempting to reconnect...");
    if (client.connect("DetectionClient", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("MQTT reconnected");
    } else {
      Serial.print("MQTT reconnection failed, state ");
      Serial.println(client.state());
    }
  }

  int pir_state = digitalRead(PIRPIN);
  int button_state = digitalRead(BUTTON_PIN);
  bool motion_detected = pir_state == 1 && button_state == 1;
  if(motion_detected == true && motion_detected != prev_motion_detected) {
    Serial.print("sending 1...");
    client.publish("motion", "motion detected");
    prev_motion_detected = motion_detected;
  } else if (motion_detected == false && motion_detected != prev_motion_detected) {
    Serial.print("sending 0...");
    client.publish("motion", "no motions");
    prev_motion_detected = motion_detected;
  }
}

void connectToWiFi() {
  delay(1000);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("WiFi Connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}