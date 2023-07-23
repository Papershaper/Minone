#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"   // Include sensitive credentials (gitignored)

// Define the onboard LED pin (ESP32 uses GPIO 2 for many boards)
#define LED_BUILTIN 2

// ===== Global Objects =====
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ===== Function Prototypes =====
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Connect to WiFi
  setupWiFi();

  // Set MQTT server and callback
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  // Reconnect to MQTT broker if needed
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  // Process incoming MQTT messages and keep connection alive
  mqttClient.loop();

  // ===== Heartbeat LED Blink (non-blocking) =====
  static unsigned long lastHeartbeat = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeat >= 2000) { // Every 2 seconds
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);  // Brief blink
    digitalWrite(LED_BUILTIN, LOW);
    lastHeartbeat = currentMillis;
  }
}

// ===== WiFi Setup Function =====
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);

  // Set WiFi to station mode and initiate connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT Reconnection Function =====
void reconnectMQTT() {
  // Loop until reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect (without username/password, but you can add these parameters)
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, subscribe to a topic (example: "minone/commands")
      mqttClient.subscribe("minone/commands");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

// ===== MQTT Callback Function =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic [");
  Serial.print(topic);
  Serial.print("]: ");

  // Convert payload to a string for easy handling
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // For demonstration: flash the LED on message receipt
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);  // Note: in production code, consider a non-blocking approach
  digitalWrite(LED_BUILTIN, LOW);

  // You can expand this callback to handle specific commands
}
