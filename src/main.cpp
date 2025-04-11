#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <queue>
#include "globals.h"
#include "mqtt_manager.h"
#include "motors.h"
#include "sensor.h"
#include "secrets.h"   // Sensitive credentials (gitignored)

// Define LED pin and sensor pins
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif



// Global objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
MQTTManager mqttManager(mqttClient);

// Define robot agent states
enum RobotState { STANDBY, MANUAL, AUTONOMOUS, PAUSED, ERROR };
// Global variable to hold the current agent state
RobotState currentRobotState = STANDBY;

// Task Type definition for non-blocking task execution
enum TaskType {TASK_NONE, TASK_MOVE, TASK_TURN, TASK_SCAN };  //room for more

// Task Wrapper, for queue and execution
struct RobotTask {
  TaskType type;
  MoveCommand moveCmd;  // for TASK_MOVE
  TurnCommand turnCmd;  // for TASK_TURN
  ScanCommand scanCmd;  // for TASK_SCAN
  // Additional commands can go here
  // only 'activate' the relevant command in the TaskType
};

//set up the queue
std::queue<RobotTask> taskQueue;
RobotTask* activeTask = nullptr;

// --- Global Agent State for Automated Mode ---  DEPRECATED, Reuse for Frontier Search
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


// Function prototypes
void setupWiFi();
void handleMQTTCommands(String command);  //future plans
void setupOTA();
void initializeMap();
void updateCell(int gridX, int gridY, uint8_t value);
void markLineFree(int x0, int y0, int x1, int y1);
void publishMap();
void processOTA();
void handleHeartbeat();
void publishTelemetry();
void updateTelemetry();
void updateRobotAgent();  // Now includes the sensor sweep and movement logic
void processIncomingCommands();
void updateManualCommands();
void handleErrorState();
void updateTaskRunner();
bool updateRobotTask(RobotTask &task); //activate the RobotTask wrapper pattern
void enqueueMoveTask(float distanceCm, int speed, unsigned long timeoutMs);
void enqueueTurnTask(float angleDeg, int speed, unsigned long timeoutMs);
void enqueueScanTask(float startAngle, float endAngle, int speed, unsigned long timeoutMs);


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
  //processOTA();         // Handle OTA updates
  handleHeartbeat();    // Blink LED for system heartbeat
  updateOdometry();     // keep the odometry up to date
  updateTelemetry();    // Publish telemetry and map at intervals
  processIncomingCommands();  //new MQTT commands or other inputs
  updateManualCommands();  //process any new commands

  switch(currentRobotState) {
    case MANUAL:
      updateTaskRunner();
      break;
    case AUTONOMOUS:
      updateRobotAgent();
      updateTaskRunner();
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

void updateTaskRunner() {
  if (!activeTask) {
    if (!taskQueue.empty()) {
      // We only set the pointer to the front item
      // so we can update it in place
      // (Alternatively, you can pop and store it, but then you must
      // push it back if it's not done. Another approach is to copy it locally.)
      activeTask = &taskQueue.front();
    } else {
      return; // no tasks
    }
  }

  bool done = updateRobotTask(*activeTask);
  if (done) {
    // Completed or aborted => remove from queue
    taskQueue.pop();
    activeTask = nullptr;
  }
}

bool updateRobotTask(RobotTask &task) {
  switch (task.type) {
    case TASK_MOVE:
      return updateMoveTask(task.moveCmd);
    case TASK_TURN:
      return updateTurnTask(task.turnCmd);
    case TASK_SCAN:
      return updateScanTask(task.scanCmd);
    default:
      // no-op or immediately done
      return true;
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
        unsigned long timeout = doc["timeout"] | 10000; // default 10s

        // Instead of directly moving, we create a task:
        enqueueMoveTask(distance, speed, timeout);
      }
      else if (strcmp(action, "turn") == 0) {
        float angle   = doc["angle_deg"] | 0.0;
        int speed     = doc["speed"]     | 200;
        unsigned long timeout = doc["timeout"] | 10000;

        enqueueTurnTask(angle, speed, timeout);
      }
      else if (strcmp(action, "scan") == 0) {
        float startAngle   = doc["start_angle"] | 45.0;
        float endAngle   = doc["end_angle"] | 135.0;
        int speed     = doc["speed"]     | 40;   //pause in ms
        unsigned long timeout = doc["timeout"] | 5000;

        enqueueScanTask(startAngle, endAngle, speed, timeout);
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


// ----- Task Function for Autonomous Robot Agent -----
void updateRobotAgent() {
  static unsigned long idleStartTime = 0;
  unsigned long now = millis();

  if (taskQueue.empty()){
    // for Automation, if the taskQueue is empty queu up these tasks:

    enqueueScanTask(40, 140, 20, 5000);
    enqueueScanTask(40, 140, 20, 5000);
    enqueueTurnTask(180, 200, 10000);
    enqueueMoveTask(20.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout
    enqueueTurnTask(180, 200, 3000);
    enqueueMoveTask(40.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout
    enqueueTurnTask(180, 200, 3000);
    enqueueMoveTask(40.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout
    enqueueTurnTask(180, 200, 3000);
    enqueueMoveTask(40.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout
    enqueueTurnTask(180, 200, 3000);
    enqueueMoveTask(40.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout
    enqueueTurnTask(180, 200, 10000);
    enqueueScanTask(40, 140, 20, 5000);
    
  }

  // switch (autoAgentState) {
  //   case STATE_IDLE: {
  //     // On first entry into IDLE, record the start time.
  //     if (idleStartTime == 0) {
  //       idleStartTime = now;
  //       Serial.println("Entering IDLE state: pausing for 5 seconds.");
  //     }
  //     if (now - idleStartTime >= 5000) {  // 60,000 ms = 60 sec
  //       idleStartTime = 0;  // Reset for next idle period
  //       autoAgentState = STATE_SWEEP;
  //       Serial.println("Idle complete. Switching to SWEEP state.");
  //     }
  //     break;
  //   }

  //   case STATE_SWEEP: {
  //     // Perform the full, blocking sensor sweep.
  //     Serial.println("Blocking sensor sweep...");
  //     sweepAndUpdateMap();  // This function blocks while sweeping
  //     autoAgentState = STATE_MOVE;
  //     // currentAgentState = STATE_IDLE;
  //     break;
  //   }
    
  //   case STATE_MOVE: {
  //     Serial.println("Enqueueing a move task from agent...");
  //     enqueueMoveTask(10.0f, 200, 10000);  // e.g., 10 cm, speed=200, 10s timeout

  //     autoAgentState = STATE_IDLE;  // nonblcking swith to itdle form now.
  //     break;
  //   }
  // }
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
  long distance = readUltrasonicDistance();
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

// ---------------
//  Helper FUNCTIONS
// ---------------
void enqueueMoveTask(float distanceCm, int speed, unsigned long timeoutMs) {
  RobotTask task;
  task.type = TASK_MOVE;
  // Initialize the MoveCommand sub-struct
  task.moveCmd.moveState      = MOVE_IDLE;   // Start from IDLE
  task.moveCmd.targetDistance = distanceCm;
  task.moveCmd.speed          = speed;
  task.moveCmd.timeout        = timeoutMs;
  // Other fields like startX, startY, startTime will be set in the update function

  taskQueue.push(task);
}

void enqueueTurnTask(float angleDeg, int speed, unsigned long timeoutMs) {
  RobotTask task;
  task.type = TASK_TURN;
  task.turnCmd.turnState    = TURN_IDLE;
  task.turnCmd.targetAngle  = angleDeg;
  task.turnCmd.speed        = speed;
  task.turnCmd.timeout      = timeoutMs;
  // etc.

  taskQueue.push(task);
}

void enqueueScanTask(float startAngle, float endAngle, int speed, unsigned long timeoutMs) {
  RobotTask task;
  task.type = TASK_SCAN;
  task.scanCmd.scanState    = SCAN_IDLE;
  task.scanCmd.startAngle  = startAngle;
  task.scanCmd.endAngle    = endAngle;
  task.scanCmd.speed        = speed;
  task.scanCmd.timeout      = timeoutMs;
  // etc.

  taskQueue.push(task);
}