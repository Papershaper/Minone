#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "secrets.h"   // Sensitive credentials (gitignored)

// Define LED pin and sensor pins
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define TRIG_PIN 5      
#define ECHO_PIN 18     

// Global objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Telemetry and heartbeat timing variables
unsigned long telemetryInterval = 2000;  // Default telemetry interval (ms)
unsigned long lastTelemetry = 0;
unsigned long lastHeartbeat = 0;

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
long readUltrasonicDistance(int triggerPin, int echoPin);
long getAverageDistance(int samples);
void handleMQTTCommands(String command);
void setupOTA();

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup OTA update capability
  setupOTA();
  
  // Configure MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  // Ensure MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Handle OTA updates
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();

  // Non-blocking LED heartbeat: blink briefly every 2 seconds
  if (currentMillis - lastHeartbeat >= 2000) {
    digitalWrite(LED_BUILTIN, HIGH);
    lastHeartbeat = currentMillis;
  } else if (currentMillis - lastHeartbeat >= 100) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Publish telemetry at defined intervals
  if (currentMillis - lastTelemetry >= telemetryInterval) {
    // Average multiple sensor readings
    long distance = getAverageDistance(5);
    int rssi = WiFi.RSSI();
    unsigned long uptime = currentMillis / 1000;  // Uptime in seconds

    // Create JSON formatted telemetry using ArduinoJson
    StaticJsonDocument<200> doc;
    doc["distance_cm"] = distance;
    doc["rssi"] = rssi;
    doc["uptime_sec"] = uptime;
    
    String payload;
    serializeJson(doc, payload);
    
    mqttClient.publish("minone/telemetry", payload.c_str());
    Serial.print("Telemetry published: ");
    Serial.println(payload);
    
    lastTelemetry = currentMillis;
  }
}

// ===== WiFi Setup =====
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT Reconnection =====
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      mqttClient.subscribe("minone/commands");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

// ===== MQTT Callback =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic [");
  Serial.print(topic);
  Serial.print("]: ");
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Handle command messages on "minone/commands"
  handleMQTTCommands(message);
}

// ===== Command Handler =====
void handleMQTTCommands(String command) {
  // Example: "SET_INTERVAL:3000" adjusts telemetry interval to 3000 ms
  if (command.startsWith("SET_INTERVAL:")) {
    telemetryInterval = command.substring(13).toInt();
    Serial.print("Telemetry interval set to ");
    Serial.print(telemetryInterval);
    Serial.println(" ms");
  }
  // Additional commands can be added here.
}

// ===== Ultrasonic Sensor Reading =====
long readUltrasonicDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distanceCm = (duration * 0.034) / 2;
  return distanceCm;
}

// ===== Average Sensor Reading =====
long getAverageDistance(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
    delay(50);  // Small delay between readings
  }
  return sum / samples;
}

// ===== OTA Setup =====
void setupOTA() {
  ArduinoOTA.setHostname("Minone-ESP32");
  ArduinoOTA.onStart([](){
    Serial.println("Start updating firmware...");
  });
  ArduinoOTA.onEnd([](){
    Serial.println("\nOTA Update complete.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
    Serial.printf("OTA Progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
