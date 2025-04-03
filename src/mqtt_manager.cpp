#include "mqtt_manager.h"
#include "globals.h"
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
            mqttClient.subscribe("PolyMap/minone/cmd/#");  //SUBSCRIBE to all commands
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
    
    if (strstr(topic, "/cmd/state") != nullptr) {
        // Minimal parse
        if (message == "start_manual") {
            startManualFlag = true;
        }
        else if (message == "start_auto") {
            startAutoFlag = true;
        }
        else if (message == "pause") {
            pauseFlag = true;
        }
        else if (message == "resume") {
            resumeFlag = true;
        }
      }
      else if (strstr(topic, "/cmd/manual") != nullptr) {
        // Create a new ManualTaskItem
        ManualTaskItem task;
        // Optionally, parse out a command type from the message 
        // but you decided to store 'commandType' or 'rawMessage' for later:
        // For example, if you want a quick guess:
        task.commandType = "unknown"; 
        // Or parse a small substring, etc.
        
        task.rawMessage = message;
        task.timestamp  = millis();

        // Push the command into the global queue
        manualTaskQueue.push(task);
      }
}
