#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// --- Motor Pin Definitions ---
#define LEFT_MOTOR_EN 12      // PWM enable pin for left motor
#define LEFT_MOTOR_IN1 13     // Direction control pin 1 for left motor
#define LEFT_MOTOR_IN2 14     // Direction control pin 2 for left motor

#define RIGHT_MOTOR_EN 15     // PWM enable pin for right motor
#define RIGHT_MOTOR_IN1 16    // Direction control pin 1 for right motor
#define RIGHT_MOTOR_IN2 17    // Direction control pin for right motor

// --- Encoder Pin Definitions ---
#define LEFT_ENCODER_PIN 32
#define RIGHT_ENCODER_PIN 33

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
void updateOdometry();
void handleMotorCommand(String command);

#endif // MOTORS_H
