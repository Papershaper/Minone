#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "globals.h"
#include "mqtt_manager.h"
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
MQTTManager mqttManager(mqttClient);

// Define robot agent states
enum RobotState { STANDBY, MANUAL, AUTONOMOUS, PAUSED, ERROR };
// Global variable to hold the current agent state
RobotState currentRobotState = STANDBY;

// --- Global Agent State for Automated Mode ---
enum AgentState { STATE_IDLE, STATE_SWEEP, STATE_MOVE };
AgentState autoAgentState = STATE_IDLE;

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
const int SERVO_DELAY_MS = 40;     // Delay for servo to settle, 38ms timeout for sensor
const int SENSOR_MAX_DIST = 200;   // maximum distance given tof

// Function prototypes
void setupWiFi();
long readUltrasonicDistance(int triggerPin, int echoPin);
void handleMQTTCommands(String command);  //future plans
void setupOTA();
void initializeMap();
void updateCell(int gridX, int gridY, uint8_t value);
void markLineFree(int x0, int y0, int x1, int y1);
void sweepAndUpdateMap();  //function prototype just for grins
void publishMap();
void processOTA();
void handleHeartbeat();
void publishTelemetry();
void updateTelemetry();
void updateRobotAgent();  // Now includes the sensor sweep and movement logic
void processIncomingCommands();
void updateManualCommands();
void handleErrorState();


//-------- SETUP ---------------------------//

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  setupWiFi();  // Connect to WiFi
  setupOTA(); // Setup OTA update capability
  mqttManager.begin();  // Start the MQTT conneciton

  // Initialize motors and encoders
  setupMotors();
  setupEncoders();

  // Initialize the occupancy grid
  initializeMap();
  
  // Attach the servo
  // Allocate all available timers explicitly
  ESP32PWM::allocateTimer(1);
  int channel = sweepServo.attach(SERVO_PIN);
  Serial.print("[timer 1 allocated] Servo Channel: ");
  Serial.println(channel);
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
  mqttManager.maintain();       // Process MQTT messages and ensure connectivity
  processOTA();         // Handle OTA updates
  handleHeartbeat();    // Blink LED for system heartbeat
  updateOdometry();     // keep the odometry up to date
  updateTelemetry();    // Publish telemetry and map at intervals
  processIncomingCommands();  //new MQTT commands or other inputs

  // non-blocking robot controls
  updateMove();

  //updateRobotState();  //Review current high level state; room for new modes -Docking, charging
  switch(currentRobotState) {
    case MANUAL:
      updateManualCommands();
      break;
    case AUTONOMOUS:
      updateRobotAgent();
      break;
    case STANDBY:
      //wait for action
      break;
    case PAUSED:
      break;
    case ERROR:
      handleErrorState();
      break;
  }
}

void updateManualCommands() {
  // check command que and execute
  if (!manualTaskQueue.empty()) {
    ManualTaskItem cmd = manualTaskQueue.front();
    manualTaskQueue.pop();
    
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, cmd.rawMessage);
    if (!err) {
      // Read 'action' field
      const char* action = doc["action"] | "";
      if (strcmp(action, "move") == 0) {
        float distance = doc["distance_cm"] | 0.0;
        int speed      = doc["speed"] | 200;
        // Now do something with it, e.g.
        // enqueueManualCommand("move", distance, speed);
        Serial.print("Got 'move' command, distance: ");
        Serial.print(distance);
        Serial.print(" speed: ");
        Serial.println(speed);
      }
      else if (strcmp(action, "turn") == 0) {
        float angle = doc["angle_deg"] | 0.0;
        // ...
        Serial.print("Got 'turn' command, angle: ");
        Serial.println(angle);
      }
      else {
        Serial.println("Unknown action in JSON");
      }
    }
    else {
      Serial.println("JSON parse error or raw command - handle as needed");
    }
  }
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
  const float MOVE_DISTANCE_CM = 10.0;  // Target move distance

  // Variable for idle state
  static unsigned long idleStartTime = 0;

  unsigned long now = millis();

  switch (autoAgentState) {
    case STATE_IDLE: {
      // On first entry into IDLE, record the start time.
      if (idleStartTime == 0) {
        idleStartTime = now;
        Serial.println("Entering IDLE state: pausing for 5 seconds.");
      }
      // Remain idle until 20 seconds have elapsed.
      if (now - idleStartTime >= 5000) {  // 60,000 ms = 60 sec
        idleStartTime = 0;  // Reset for next idle period
        autoAgentState = STATE_SWEEP;
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
      autoAgentState = STATE_MOVE;
      // currentAgentState = STATE_IDLE;
      Serial.println("Sensor sweep complete, switching to next state.");
      break;
    }
    
    case STATE_MOVE: {
      // Command the robot to START a MOVE -- this is temporary
      // Command robot to move forward 0-255, must be >200
      startMove(MOVE_DISTANCE_CM, 200, 10000);  //move 10cm, at 200, timeout 10s

      autoAgentState = STATE_IDLE;  // nonblcking swith to itdle form now.
      break;
    }
  }
}

void processIncomingCommands() {
  // check flags and take action as needed; change state levels
  if (startManualFlag) {
    startManualFlag = false;
    currentRobotState = MANUAL;
  }

  if (startAutoFlag) {
    startAutoFlag = false;
    currentRobotState = AUTONOMOUS;
  }

  if (pauseFlag) {
    pauseFlag = false;
    currentRobotState = PAUSED;
  }

  if (resumeFlag) {
    resumeFlag = false;
    currentRobotState = AUTONOMOUS;
  }

  if (errorFlag) {
    errorFlag = false;
  }

}

void handleErrorState() {
  // Stop motors, lock out commands or do a safe shutdown
  setMotorSpeed(0, 0);
  // Possibly blink an LED or broadcast a special MQTT message
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
  // Add odometry information
  doc["posX_cm"] = posX_cm;
  doc["posY_cm"] = posY_cm;
  doc["orientation_rad"] = orientation_rad;
  
  switch (currentRobotState) {  //STANDBY, MANUAL, AUTONOMOUS, PAUSED, ERROR 
    case STANDBY:
      doc["robot_state"] = "STANDBY";
      break;
    case MANUAL:
      doc["robot_state"] = "MANUAL";
      break;
    case AUTONOMOUS:
      doc["robot_state"] = "AUTONOMOUS";
      break;
    case PAUSED:
      doc["robot_state"] = "PAUSED";
      break;
    case ERROR:
      doc["robot_state"] = "ERROR";
      break;
  }

  switch (autoAgentState) {
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
  
  mqttManager.publishTelemetry(payload.c_str());
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
      occupancyGrid[i][j] = UNKNOWN;  //Initialize as UNKNOWN
    }
  }
}

void updateCell(int gridX, int gridY, uint8_t value) {
  if (gridX >= 0 && gridX < MAP_WIDTH && gridY >= 0 && gridY < MAP_HEIGHT) {
    occupancyGrid[gridY][gridX] = value;
  }
}

void markLineFree(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
  int err = dx + dy, e2; // error value e_xy

  while (true) {
    updateCell(x0, y0, FREE);  // mark current cell as FREE
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
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
    
    // Calculate the global map angle
    float global_angle = orientation_rad + (angle - SERVO_CENTER) * (PI / 180.0);
    
    // Convert the measured distance to number of cells
    int cellsAway = distance_cm / CELL_SIZE_CM;

    // Compute the grid coordinates of the detected obstacle
    // Assuming robot's forward (0° relative) corresponds to increasing X.
    int obstacleX = robotX + (int)(cellsAway * cos(global_angle));
    int obstacleY = robotY + (int)(cellsAway * sin(global_angle));
    
    // Mark all cells along the line from the robot to the obstacle as FREE
    markLineFree(robotX, robotY, obstacleX, obstacleY);
    
    // Update the detected cell as OCCUPIED unless it is at max
    if (distance_cm < SENSOR_MAX_DIST) {
      updateCell(obstacleX, obstacleY, OCCUPIED);
    } else {
      updateCell(obstacleX, obstacleY, FREE);  // Either free or unkown
    }
    

  }
  // Delay after a complete sweep to allow time for the last reading
  delay(200);
  sweepServo.write(SERVO_CENTER);  // Center the servo
  delay(200);
}

// Publish the occupancy grid over MQTT
void publishMap() {
  // Mark the robot's current location on the grid
  updateCell(robotX, robotY, ROBOT);

  // Publish the raw binary map data  120x120 ~ 14.4 kb
  // topic:  "PolyMap/{self.robot_id}/slam"
  mqttManager.publishMap((const uint8_t*)occupancyGrid, sizeof(occupancyGrid));
  
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


// ===== Command Handler =====  //  DEPRECATED?? or hold for future commands
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
