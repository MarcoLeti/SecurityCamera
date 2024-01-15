#include <WiFi.h>
#include <PubSubClient.h>
#include "credentials.h"
#define PIRPIN 19
#define BUZZER_PIN 18

bool personDetected = false;

// Define a static IP address for this Node
IPAddress staticIP(192, 168, 1, 150); 
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);
  pinMode(PIRPIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);  // Configure LEDC channel 0: frequency=5000 Hz, resolution=8-bit
  ledcAttachPin(BUZZER_PIN, 0);  // Attach the LEDC channel to the GPIO pin
  connectToWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("AlarmClient", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT broker");
      // Subscribe to the desired topic
      client.subscribe("detection");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  client.loop();
  int pir_state = digitalRead(PIRPIN);
  // pir_state == 1 && 
  if(personDetected == true) {
    tone(BUZZER_PIN, 1000);
    Serial.print("playing...");
  } else {
    noTone(BUZZER_PIN);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  String receivedPayload = "";
  Serial.print("Payload: ");
  for (int i = 0; i < length; i++) {
    receivedPayload += (char)payload[i];
  }
  Serial.print(receivedPayload);
  Serial.println();
  if(receivedPayload == "person_detected") {
    personDetected = true;
  } else {
    personDetected = false;
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