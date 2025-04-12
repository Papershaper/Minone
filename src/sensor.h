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
const int SENSOR_MAX_DIST = 170;   // maximum distance given tof (between 140-200)


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