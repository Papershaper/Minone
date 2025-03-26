#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "motors.h" 
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
// Define robot agent states
enum AgentState { STATE_IDLE, STATE_SWEEP, STATE_MOVE };
// Global variable to hold the current agent state
AgentState currentAgentState = STATE_IDLE;

// DEBUG
const bool DEBUG = false;

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
const int SERVO_PIN = 4;           // initialized with strong type
const int SERVO_CENTER = 90;       // Servo center position (forward)
const int SWEEP_MIN = 45;          // Minimum servo angle (left sweep limit)
const int SWEEP_MAX = 135;         // Maximum servo angle (right sweep limit)
const int SWEEP_STEP = 5;          // Angle increment in degrees
const int SERVO_DELAY_MS = 30;     // Delay for servo to settle
const int SENSOR_MAX_DIST = 200;   // maximum distance given tof

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
long readUltrasonicDistance(int triggerPin, int echoPin);
void handleMQTTCommands(String command);
void setupOTA();
void initializeMap();
void updateCell(int gridX, int gridY, uint8_t value);
void sweepAndUpdateMap();  //function prototype just for grins
void publishMap();
void maintainMQTT();
void processOTA();
void handleHeartbeat();
void publishTelemetry();
void updateTelemetry();
void updateRobotAgent();  // Now includes the sensor sweep and movement logic


//-------- SETUP ---------------------------//

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  setupWiFi();  // Connect to WiFi
  setupOTA(); // Setup OTA update capability
  
  // Configure MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(16000);   // Override the buffersize for the local_map

  // Allocate all available timers explicitly
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize motors and encoders
  setupMotors();
  setupEncoders();

  // Initialize the occupancy grid
  initializeMap();
  
  // Attach the servo
  sweepServo.attach(SERVO_PIN);
  delay(200);  //optional - probably not needed, with Pin order.
  sweepServo.write(SERVO_CENTER);  // Center the servo
  
  // Log the starting position
  Serial.print("Robot starting position: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.println(")");
}

// ----- Main Loop ------------------ //
void loop() {
  maintainMQTT();       // Process MQTT messages and ensure connectivity
  processOTA();         // Handle OTA updates
  handleHeartbeat();    // Blink LED for system heartbeat
  updateOdometry();     // keep the odometry up to date
  updateTelemetry();    // Publish telemetry and map at intervals
  updateRobotAgent();   // Call the robot decision-making stub
}

// ----- Task Function: Robot Agent with Sensor Sweep and Movement -----
// The robot agent performs a sensor sweep, then drives forward 40 cm and repeats.
// This function is non-blocking and uses a state machine.
void updateRobotAgent() {
  // Remove the local enum declaration since it's now global:
  // static AgentState agentState = STATE_SWEEP;
  // Instead, use the global variable: currentAgentState

  // Variables for sensor sweep state
  static unsigned long lastSweepTime = 0;
  static int currentSweepAngle = SWEEP_MIN;  // Start at the left sweep limit

  // Variables for move state
  static float moveStartPosX = posX_cm;
  static float moveStartPosY = posY_cm;
  const float MOVE_DISTANCE_CM = 40.0;  // Target move distance

  // Variable for idle state
  static unsigned long idleStartTime = 0;

  unsigned long now = millis();

  switch (currentAgentState) {
    case STATE_IDLE: {
      // On first entry into IDLE, record the start time.
      if (idleStartTime == 0) {
        idleStartTime = now;
        Serial.println("Entering IDLE state: pausing for 60 seconds.");
      }
      // Remain idle until 20 seconds have elapsed.
      if (now - idleStartTime >= 20000) {  // 60,000 ms = 60 sec
        idleStartTime = 0;  // Reset for next idle period
        currentAgentState = STATE_SWEEP;
        Serial.println("Idle complete. Switching to SWEEP state.");
      }
      break;
    }

    case STATE_SWEEP: {
      // Perform the full, blocking sensor sweep.
      Serial.println("Starting blocking sensor sweep...");
      sweepAndUpdateMap();  // This function blocks while sweeping
      // Center the servo after sweeping
      sweepServo.write(SERVO_CENTER);
      // Record the current odometry position as the starting point for movement
      moveStartPosX = posX_cm;
      moveStartPosY = posY_cm;
      // Transition to MOVE state
      // currentAgentState = STATE_MOVE;
      currentAgentState = STATE_IDLE;
      Serial.println("Sensor sweep complete, switching to next state.");
      break;
    }
    
    case STATE_MOVE: {
      // Command robot to move forward at a fixed speed (adjust as needed)
      setMotorSpeed(100, 100);

      // Calculate distance moved using odometry (assuming updateOdometry() is called regularly)
      float dx = posX_cm - moveStartPosX;
      float dy = posY_cm - moveStartPosY;
      float distanceMoved = sqrt(dx * dx + dy * dy);

      //DEBUG
      if (DEBUG){
        Serial.print("MOVE - dx: ");
        Serial.print(dx);
        Serial.print(" dy: ");
        Serial.print(dy);
        Serial.print(" distance: ");
        Serial.println(distanceMoved);
      }

      // If moved 40 cm or more, stop and return to sweeping
      if (distanceMoved >= MOVE_DISTANCE_CM) {
        setMotorSpeed(0, 0);  // Stop the motors
        currentAgentState = STATE_SWEEP;  // Switch back to sensor sweep
        lastSweepTime = now;      // Reset sweep timing
      }
      break;
    }
  }
}

// ----- Task Function: Maintain MQTT Connection ----- 
void maintainMQTT() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
}

// ----- Task Function: Handle OTA Updates -----
void processOTA() {
  ArduinoOTA.handle();
}

// ----- Task Function: Non-blocking LED Heartbeat -----
// This version blinks the LED briefly every 2 seconds.
void handleHeartbeat() {
  static unsigned long lastBlink = 0;
  unsigned long now = millis();
  if (now - lastBlink >= 2000) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);  // Briefly turn on LED (can be adjusted)
    digitalWrite(LED_BUILTIN, LOW);
    lastBlink = now;
  }
}

// ----- Task Function: Sensor Sweep and Map Update ----- 
void performSensorSweep() {
  // Executes a full servo sweep and updates the occupancy grid
  sweepAndUpdateMap();
}

// ----- Task Function: Publish Telemetry -----
// This function publishes telemetry data and the local map.
void publishTelemetry() {
  // Example telemetry: you might update this later to include position, pose, and error statuses.
  long distance = readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
  int rssi = WiFi.RSSI();
  unsigned long uptime = millis() / 1000;  // Uptime in seconds

  StaticJsonDocument<200> doc;
  doc["distance_cm"] = distance;  // To be updated with position/pose later
  doc["rssi"] = rssi;
  doc["uptime_sec"] = uptime;
  
  switch (currentAgentState) {
    case STATE_IDLE:
      doc["agent_state"] = "idle";
      break;
    case STATE_SWEEP:
      doc["agent_state"] = "sweep";
      break;
    case STATE_MOVE:
      doc["agent_state"] = "move";
      break;
  }

  String payload;
  serializeJson(doc, payload);
  
  mqttClient.publish("minone/telemetry", payload.c_str());
  Serial.print("Telemetry pub: ");
  Serial.println(payload);

  // Also publish the current map over MQTT
  publishMap();
}

// ----- Task Function: Update Telemetry Periodically -----
void updateTelemetry() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemetry >= telemetryInterval) {
    publishTelemetry();
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
  // move to start and settle.
  sweepServo.write(SWEEP_MIN);
  delay(200);

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
    
    //DEBUG
    int read_angle = sweepServo.read(); // returns current pulse width as an angle between 0 and 180 degrees
    int read_pulse = sweepServo.readMicroseconds(); 
    Serial.print("sweep: angle:");
    Serial.print(angle);
    Serial.print(" read_angle: ");
    Serial.print(read_angle);
    Serial.print(" pulse: ");
    Serial.print(read_pulse);
    Serial.print(" dist: ");
    Serial.print(distance_cm);
    Serial.println();


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
  // Delay after a complete sweep to allow time for the robot to move
  sweepServo.write(SERVO_CENTER);  // Center the servo
  delay(500);
}

// Publish the occupancy grid over MQTT
void publishMap() {
  // Mark the robot's current location on the grid
  updateCell(robotX, robotY, ROBOT);

  // Publish the raw binary map data  120x120 ~ 14.4 kb
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
  // Use a timeout of 30000 microseconds (30 ms)
  long duration = pulseIn(echoPin, HIGH, 30000);
  // If no echo is received, treat as out-of-range (200 cm)
  if (duration == 0) {
    return SENSOR_MAX_DIST;
  }

  long distanceCm = (duration * 0.034) / 2;
  if(distanceCm > SENSOR_MAX_DIST) {
    distanceCm = SENSOR_MAX_DIST;
  }
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
