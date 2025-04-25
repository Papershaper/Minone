#include "sensor.h"
#include "globals.h"
#include "maps.h"
#include "motors.h"

// global instances
Servo sweepServo;
ScanCommand currentScan;  

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
  delay(200);

  // Sweep from SWEEP_MIN to SWEEP_MAX
  for (int angle = sweepMin; angle <= sweepMax; angle += SWEEP_STEP) {
    sweepServo.write(angle);
    delay(servoDelay);  //delay to make sure servo in correct position before sensor poll.
    
    // Get the sensor reading in centimeters
    long distance_cm = readUltrasonicDistance();
    
    // Calculate the global map angle
    // orientation_rad is the robot's current global heading
    // angle is the servo angle relative to the robot's forward (SERVO_CENTER)
    // float relative_angle_rad = (angle - SERVO_CENTER) * (M_PI / 180.0); // Convert servo angle to radians relative to robot forward
    // float global_angle_rad = orientation_rad + relative_angle_rad; // Robot's global heading + relative sensor angle
    float global_angle = orientation_rad - (angle - SERVO_CENTER) * (PI / 180.0);

    // Compute the grid coordinates of the detected obstacle, convert to cells
    // Assuming robot's forward (0° relative) corresponds to increasing X.
    float dx_cm = distance_cm * cos(global_angle);
    float dy_cm = distance_cm * sin(global_angle);

    float endX_cm = posX_cm + dx_cm;
    float endY_cm = posY_cm + dy_cm;

    int obstacleX = robotX + (int)(dx_cm / CELL_SIZE_CM);
    int obstacleY = robotY + (int)(dy_cm / CELL_SIZE_CM);

    // Perform the probabilistic ray update on the float grid
    probabilisticRayUpdate(posX_cm, posY_cm, endX_cm, endY_cm, (float)distance_cm);

  }
  // Delay after a complete sweep to allow time for the last reading
  delay(200);
  sweepServo.write(SERVO_CENTER);  // Center the servo
  delay(200);
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