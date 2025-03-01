#include <Arduino.h>
#include <ESP32Servo.h>
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
// Occupancy map
#define MAP_WIDTH 120
#define MAP_HEIGHT 120
#define UNKNOWN 255  //unexplored- frontier
#define FREE 0
#define OCCUPIED 1
#define ROBOT 2

// Global objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Telemetry and heartbeat timing variables
unsigned long telemetryInterval = 2000;  // Default telemetry interval (ms)
unsigned long lastTelemetry = 0;
unsigned long lastHeartbeat = 0;

// Create the occupancy grid as a 2D array
uint8_t occupancyGrid[MAP_HEIGHT][MAP_WIDTH];
int robotX = MAP_WIDTH / 2;  // 60 - center
int robotY = MAP_HEIGHT / 2; // 60 - center
const int CELL_SIZE_CM = 10;

// Servo and sweep settings
Servo sweepServo;
const int SERVO_PIN = 4;           // Change to your servo control pin
const int SERVO_CENTER = 90;       // Servo center position (forward)
const int SWEEP_MIN = 45;          // Minimum servo angle (left sweep limit)
const int SWEEP_MAX = 135;         // Maximum servo angle (right sweep limit)
const int SWEEP_STEP = 5;          // Angle increment in degrees
const int SERVO_DELAY_MS = 15;     // Delay for servo to settle

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
long readUltrasonicDistance(int triggerPin, int echoPin);
void handleMQTTCommands(String command);
void setupOTA();
void initializeMap();
void updateCell(int gridX, int gridY, uint8_t value);
void sweepAndUpdateMap();
void publishMap();

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

  // Initialize the occupancy grid
  initializeMap();
  
  // Attach the servo
  sweepServo.attach(SERVO_PIN);
  sweepServo.write(SERVO_CENTER);  // Center the servo
  
  // Log the starting position
  Serial.print("Robot starting position: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.println(")");
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

  // Perform a sweep and update the map
  sweepAndUpdateMap();
  
  // Publish telemetry at defined intervals
  if (currentMillis - lastTelemetry >= telemetryInterval) {
    // kinda obsolete now with the sweep
    long distance = readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
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

    // Publish the current map over MQTT
    publishMap();
    
    lastTelemetry = currentMillis;
  }
}

// ==================================
void initializeMap() {
  for (int i = 0; i < MAP_HEIGHT; i++) {
    for (int j = 0; j < MAP_WIDTH; j++) {
      occupancyGrid[i][j] = UNKNOWN;
    }
  }
}

void updateCell(int gridX, int gridY, uint8_t value) {
  if (gridX >= 0 && gridX < MAP_WIDTH && gridY >= 0 && gridY < MAP_HEIGHT) {
    occupancyGrid[gridY][gridX] = value;
  }
}

// Sweep function: Moves the servo, reads distance, and updates the map.
void sweepAndUpdateMap() {
  // Sweep from SWEEP_MIN to SWEEP_MAX
  for (int angle = SWEEP_MIN; angle <= SWEEP_MAX; angle += SWEEP_STEP) {
    sweepServo.write(angle);
    delay(SERVO_DELAY_MS);
    
    // Get the sensor reading in centimeters
    long distance_cm = readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
    
    // Calculate the relative angle (with 0° being forward)
    float relativeAngle = angle - SERVO_CENTER;
    float angle_rad = relativeAngle * (PI / 180.0);
    
    // Convert the measured distance to number of cells
    int cellsAway = distance_cm / CELL_SIZE_CM;
    
    // Compute the grid coordinates of the detected obstacle
    // Assuming robot's forward (0° relative) corresponds to increasing X.
    int obstacleX = robotX + (int)(cellsAway * cos(angle_rad));
    int obstacleY = robotY + (int)(cellsAway * sin(angle_rad));
    
    // Update the detected cell as OCCUPIED
    updateCell(obstacleX, obstacleY, OCCUPIED);
    
    // Optionally, mark cells along the beam as FREE.
    // A simple way is to step from the robot's position to the obstacle.
    for (int i = 1; i < cellsAway; i++) {
      int freeX = robotX + (int)(i * cos(angle_rad));
      int freeY = robotY + (int)(i * sin(angle_rad));
      updateCell(freeX, freeY, FREE);
    }
  }
}

// Publish the occupancy grid over MQTT
void publishMap() {
  // Mark the robot's current location on the grid
  updateCell(robotX, robotY, ROBOT);
  
  // Publish the raw binary map data
  // topic:  "PolyMap/{self.robot_id}/slam"
  mqttClient.publish("minone/local_map", (const uint8_t*)occupancyGrid, sizeof(occupancyGrid));
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
