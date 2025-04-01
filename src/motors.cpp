#include "motors.h"

// --- Global Variable Definitions ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

float posX_cm = 0;         // Initialize these to your starting position
float posY_cm = 0;
float orientation_rad = 0.0;

// --- Odometry Calculation Constants ---
const float WHEEL_DIAMETER_CM = 7.2;      // 72 mm. Example: 5 cm diameter wheel
const int   ENCODER_PULSES_PER_REV = 509;   // actual 508.8
const float WHEEL_BASE_CM = 25.0;           // Distance between the wheels in cm

// --- PWM Setup Parameters ---
const int PWM_FREQUENCY = 5000; // 5kHz PWM frequency
const int PWM_RESOLUTION = 8;   // 8-bit resolution (0-255)

// --- Motor Setup ---
void setupMotors() {
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  digitalWrite(LEFT_MOTOR_EN, LOW);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);

  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  digitalWrite(RIGHT_MOTOR_EN, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  // Configure LEDC channels for PWM
  // Left Motor on channel 0
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_EN, 0);

  // Right Motor on channel 1
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_MOTOR_EN, 1);
}

// --- Motor Speed Control ---
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  leftSpeed = constrain(leftSpeed, 0, 255);
  ledcWrite(0, leftSpeed);

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  rightSpeed = constrain(rightSpeed, 0, 255);
  ledcWrite(1, rightSpeed);
}

// --- Encoder Interrupt Service Routines ---
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

// --- Encoder Setup ---
void setupEncoders() {
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);
}

// --- Odometry Update ---
// Call this function periodically (e.g., in loop()) to update the robot's position.
void updateOdometry() {
  static long lastLeftCount = 0;
  static long lastRightCount = 0;

  // Safely copy encoder counts
  noInterrupts();
  long currentLeft = leftEncoderCount;
  long currentRight = rightEncoderCount;
  interrupts();

  long deltaLeft = currentLeft - lastLeftCount;
  long deltaRight = currentRight - lastRightCount;
  lastLeftCount = currentLeft;
  lastRightCount = currentRight;

  // Calculate distance travelled by each wheel (in cm)
  float leftDistance = (deltaLeft * (PI * WHEEL_DIAMETER_CM)) / ENCODER_PULSES_PER_REV;
  float rightDistance = (deltaRight * (PI * WHEEL_DIAMETER_CM)) / ENCODER_PULSES_PER_REV;

  // Compute average displacement and change in orientation
  float deltaDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE_CM;

  // Update orientation (radians)
  orientation_rad += deltaTheta;

  // Update position (in cm)
  posX_cm += deltaDistance * cos(orientation_rad);
  posY_cm += deltaDistance * sin(orientation_rad);
}

// --- Example Motor Command Handler ---
// Expected MQTT command format: "MOTOR:<left_speed>,<right_speed>"
// For example, "MOTOR:100,-100" drives the left motor forward at 100 and right motor in reverse at 100.
void handleMotorCommand(String command) {
  if (command.startsWith("MOTOR:")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      int leftSpeed = command.substring(6, commaIndex).toInt();
      int rightSpeed = command.substring(commaIndex + 1).toInt();
      setMotorSpeed(leftSpeed, rightSpeed);
    }
  }
}
