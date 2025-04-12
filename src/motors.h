#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// --- Motor Pin Definitions ---
#define LEFT_MOTOR_EN 19      // PWM enable pin for left motor (was 12)
#define LEFT_MOTOR_IN1 13     // Direction control pin 1 for left motor
#define LEFT_MOTOR_IN2 14     // Direction control pin 2 for left motor

#define RIGHT_MOTOR_EN 22     // PWM enable pin for right motor (was 15)
#define RIGHT_MOTOR_IN1 16    // Direction control pin 1 for right motor
#define RIGHT_MOTOR_IN2 17    // Direction control pin for right motor

// --- Encoder Pin Definitions ---
#define LEFT_ENCODER_PIN 32
#define RIGHT_ENCODER_PIN 33

enum MoveState { MOVE_IDLE, MOVE_INIT, MOVE_IN_PROGRESS, MOVE_COMPLETE, MOVE_ABORT };

struct MoveCommand {
  MoveState moveState;
  float targetDistance; // in cm
  int direction;
  int speed;            // motor speed
  unsigned long startTime;
  unsigned long timeout; // max allowed time in milliseconds
  float startX;
  float startY;
  float startOrientation;
};
extern MoveCommand currentMove;

// TURN -- use the sam Finite State Machine for Scan.
enum TurnState { TURN_IDLE, TURN_INIT, TURN_IN_PROGRESS, TURN_COMPLETE, TURN_ABORT };

struct TurnCommand {
  TurnState turnState;
  float targetAngle;
  float targetAngle_rad;
  float startAngle_rad;
  int speed;
  unsigned long startTime;
  unsigned long timeout;
};

// --- Odometry Parameters ---
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

extern float posX_cm;         // Robot X position in cm
extern float posY_cm;         // Robot Y position in cm
extern float orientation_rad; // Robot orientation in radians

// --- Constants for Odometry Calculation ---
extern const float WHEEL_DIAMETER_CM;
extern const int   ENCODER_PULSES_PER_REV;
extern const float WHEEL_BASE_CM;

// --- PWM Parameters ---
extern const int PWM_FREQUENCY;
extern const int PWM_RESOLUTION;

// --- Function Prototypes ---
void setupMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);
void setupEncoders();
float normalizeAngle(float angle);
void updateOdometry();
void startMove(float distance, int speed, unsigned long timeout_ms);
bool updateMoveTask(MoveCommand &cmd);
bool updateTurnTask(TurnCommand &cmd);

#endif // MOTORS_H
