#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <PubSubClient.h>

class MQTTManager {
public:
    // Constructor takes a reference to a PubSubClient
    MQTTManager(PubSubClient &client);
    // Initializes the MQTT settings
    void begin();
    // Call this in your main loop to maintain the connection and process messages
    void maintain();
    // Publish telemetry data (or any other payload) to a specified topic
    void publishTelemetry(const char* payload);
    // Publish the occupancy grid map
    void publishMap(const uint8_t* occupancyGrid, size_t size);

private:
    PubSubClient &mqttClient;
    // Internal reconnection logic
    void reconnect();
    // Static callback to handle incoming MQTT messages
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
};

// forward
void networkMaintain();

#endif // MQTT_MANAGER_H
