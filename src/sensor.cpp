#include "sensor.h"
#include "globals.h"
#include "maps.h"
#include "motors.h"
#include "mqtt_manager.h"

// global instances
Servo sweepServo;
ScanCommand currentScan;

// --- to prevent too much blocking during the scan ---- //
static inline void waitWithComms(uint32_t wait_ms)
{
    uint32_t t0 = millis();
    while (millis() - t0 < wait_ms) {
        networkMaintain();    // keep the broker alive
        delay(1);                  // 1 ms sleep → feeds Wi-Fi/crypto tasks
    }
}

bool updateScanTask(ScanCommand &cmd) {


    switch (cmd.scanState) {
    
        case SCAN_IDLE:
            // Transition to INIT
            cmd.scanState = SCAN_INIT;
            return false;
    
        case SCAN_INIT:
            sweepServo.write(SERVO_CENTER);
            cmd.startTime = millis();
            cmd.scanState = SCAN_IN_PROGRESS;
            return false;
    
        case SCAN_IN_PROGRESS: 
            Serial.println("Blocking SCAN_IN_PROGRESS...");
            sweepAndUpdateMap(cmd.startAngle, cmd.endAngle, cmd.speed);  // This function blocks while sweeping
            cmd.scanState = SCAN_COMPLETE;
            return false;
        
        case SCAN_COMPLETE:
            sweepServo.write(SERVO_CENTER);
        return true;
    
        case SCAN_ABORT:
            Serial.println("SCAN ABORT");
        return true;  // done, but aborted
    }
    return true; // default to done if something unexpected
    
}

// Sweep function: Moves the servo, reads distance, and updates the map.
void sweepAndUpdateMap(int sweepMin, int sweepMax, int servoDelay) {
  if ((sweepMin > sweepMax) || (sweepMin < SWEEP_MIN) || (sweepMax > SWEEP_MAX)) {
    return;
  }
  // move to start and settle.
  sweepServo.write(sweepMin);
  waitWithComms(200);

  // Sweep from SWEEP_MIN to SWEEP_MAX
  for (int angle = sweepMin; angle <= sweepMax; angle += SWEEP_STEP) {
    sweepServo.write(angle);
    waitWithComms(servoDelay);  //delay to make sure servo in correct position before sensor poll.
    
    // Get the sensor reading in centimeters
    long distance_cm = readUltrasonicDistance();
    
    // Calculate the global map angle, servo angle and robot angle.
    // orientation_rad is the robot's current global heading
    // angle is the servo angle relative to the robot's forward (SERVO_CENTER)
    // float relative_angle_rad = (angle - SERVO_CENTER) * (M_PI / 180.0); // Convert servo angle to radians relative to robot forward
    // float global_angle_rad = orientation_rad + relative_angle_rad; // Robot's global heading + relative sensor angle
    // TODO:  account for the servo offset from the Center of the robot (10cm)
    // TODO:  account for the sensor offset from servo center (4cm)
    // float global_angle = orientation_rad - (angle - SERVO_CENTER) * (PI / 180.0);  //DEPRICATED
    float theta_robot   = orientation_rad;                          // robot body heading
    float theta_servo   = (angle - SERVO_CENTER) * PI / 180.0;      // ± servo sweep
    float theta_global  = theta_robot - theta_servo;                // **CW vs CCW sign fixed**


    // Compute the grid coordinates of the detected obstacle, convert to cells
    // Assuming robot's forward (0° relative) corresponds to increasing X.
    //float dx_cm = distance_cm * cos(global_angle);
    //float dy_cm = distance_cm * sin(global_angle);

    /* position of the servo pivot in world coords */
    float servoX = posX_cm + SERVO_OFFSET_CM * cos(theta_robot);
    float servoY = posY_cm + SERVO_OFFSET_CM * sin(theta_robot);

    /* position of the ultrasonic sensor from the scan pivot point */
    float startX_cm = servoX + SENSOR_OFFSET_CM * cos(theta_global);
    float startY_cm = servoY + SENSOR_OFFSET_CM * sin(theta_global);

    /* endpoint (hit or max) */
    float endX_cm   = startX_cm + distance_cm * cos(theta_global);
    float endY_cm   = startY_cm + distance_cm * sin(theta_global);

    // Perform the probabilistic ray update on the float grid
    probabilisticRayUpdate(startX_cm, startY_cm, endX_cm, endY_cm, (float)distance_cm);

    // int obstacleX = robotX + (int)(dx_cm / CELL_SIZE_CM);  //depricted?
    // int obstacleY = robotY + (int)(dy_cm / CELL_SIZE_CM);  //depricted?

  }
  // Delay after a complete sweep to allow time for the last reading
  waitWithComms(200);
  sweepServo.write(SERVO_CENTER);  // Center the servo
}

// ===== Ultrasonic Sensor Reading =====
long readUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Use a timeout of 30000 microseconds (30 ms)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
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