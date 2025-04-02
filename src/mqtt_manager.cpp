#include "mqtt_manager.h"
#include "secrets.h"   // For MQTT credentials
#include <Arduino.h>

MQTTManager::MQTTManager(PubSubClient &client) : mqttClient(client) { }

void MQTTManager::begin() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(MQTTManager::mqttCallback);
    mqttClient.setBufferSize(16000);  // Adjust if needed
}

void MQTTManager::maintain() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
}

void MQTTManager::publishTelemetry(const char* payload) {
    mqttClient.publish("PolyMap/minone/telemetry", payload);
    Serial.print("Telemetry published: ");
    Serial.println(payload);
}

void MQTTManager::publishMap(const uint8_t* occupancyGrid, size_t size) {
    mqttClient.publish("PolyMap/minone/local_map/blob", occupancyGrid, size);
}

void MQTTManager::reconnect() {
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
            Serial.println(" - retrying in 5 seconds");
            delay(5000);
        }
    }
}

void MQTTManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic [");
    Serial.print(topic);
    Serial.print("]: ");
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);
    // Here, you can call a function to handle the command (e.g., handleMQTTCommands)
}
