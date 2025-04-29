#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define TRIG_PIN 5      
#define ECHO_PIN 18

const int SERVO_PIN = 4;           // initialized with strong type
const int SERVO_CENTER = 90;       // Servo center position (forward)
const int SWEEP_MIN = 10;          // Minimum servo angle (left sweep limit)
const int SWEEP_MAX = 170;         // Maximum servo angle (right sweep limit)
const int SWEEP_STEP = 5;          // Angle increment in degrees
const int SERVO_DELAY_MS = 40;     // Delay for servo to settle, 38ms timeout for sensor
const int SENSOR_MAX_DIST = 150;   // maximum distance given tof (between 140-200)
//----- MINONE SETTINGS ------?If for other robots?
const float SERVO_OFFSET_CM   = 10.0f;   // Servo is 10cm forward of the robot CENTER
const float SENSOR_OFFSET_CM = 4.0f; // the sensor is 4cm from the servo pivot point
const float ROBOT_RADIUS_CM  = 12.5f;   // 25 cm wheel base Ø ≈ 12.5 cm radius


enum ScanState { SCAN_IDLE, SCAN_INIT, SCAN_IN_PROGRESS, SCAN_COMPLETE, SCAN_ABORT };

struct ScanCommand {
  ScanState scanState;
  int startAngle;
  int endAngle;
  int speed;
  unsigned long startTime;
  unsigned long timeout; // max allowed time in milliseconds
};

// global declariations
extern Servo sweepServo;
extern ScanCommand currentScan;  

//Function prototypes
void sweepAndUpdateMap(int sweepMin, int sweepMax, int servoDelay);
bool updateScanTask(ScanCommand &cmd);
long readUltrasonicDistance();

#endif