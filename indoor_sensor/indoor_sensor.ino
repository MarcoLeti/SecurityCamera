#include <WiFi.h>
#include <PubSubClient.h>
#include "credentials.h"
#define PIRPIN 19

// Define a static IP address for this Node
IPAddress staticIP(192, 168, 1, 150); 
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);
  pinMode(PIRPIN, INPUT);
  connectToWiFi();
}

void loop() {
  int pir_state = digitalRead(PIRPIN);
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
  if(pir_state == 1) {
    client.publish("motion", "motion detected");
  }
  delay(5000);
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