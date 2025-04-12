#include <cmath>
#include "motors.h"

// --- Global Variable Definitions ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
int leftMotorDirection = 1;
int rightMotorDirection = 1;

float posX_cm = 0;         // Initialize these to your starting position
float posY_cm = 0;
float orientation_rad = 0.0;

// Define the global move state and command.
MoveState moveState = MOVE_IDLE;
MoveCommand currentMove;

// --- Odometry Calculation Constants ---
const float WHEEL_DIAMETER_CM = 7.2;      // 72 mm. Example: 5 cm diameter wheel
const int   ENCODER_PULSES_PER_REV = 509;   // actual 508.8
const float WHEEL_BASE_CM = 25.0;           // Distance between the wheels in cm
const float ANGLE_TOLERANCE = 0.01;    //for tracking turns

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
    leftMotorDirection = 1;
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    leftMotorDirection = -1;
    leftSpeed = -leftSpeed;
  }
  leftSpeed = constrain(leftSpeed, 0, 255);
  ledcWrite(0, leftSpeed);

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    rightMotorDirection = 1;
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    rightMotorDirection = -1;
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
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);
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

  // Incorporate motor direction into the pulse counts
  deltaLeft *= leftMotorDirection;
  deltaRight *= rightMotorDirection;

  // Calculate distance travelled by each wheel (in cm)
  float leftDistance = (deltaLeft * (PI * WHEEL_DIAMETER_CM)) / ENCODER_PULSES_PER_REV;
  float rightDistance = (deltaRight * (PI * WHEEL_DIAMETER_CM)) / ENCODER_PULSES_PER_REV;

  // Compute average displacement and change in orientation
  float deltaDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE_CM;

  // Update orientation (radians)
  orientation_rad += deltaTheta;
  orientation_rad = normalizeAngle(orientation_rad);

  // Update position (in cm)
  posX_cm += deltaDistance * cos(orientation_rad);
  posY_cm += deltaDistance * sin(orientation_rad);
}

float normalizeAngle(float angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// --- Movement Handlers = non-blocking ---
// ========================================

void startMove(float distance, int speed, unsigned long timeout_ms) {
  currentMove.targetDistance = distance;
  currentMove.direction = (distance > 0.0f) - (distance < 0.0f);  // should return -1, 1, or 0
  currentMove.speed = speed;
  currentMove.timeout = timeout_ms;
  currentMove.startTime = millis();
  currentMove.startX = posX_cm;
  currentMove.startY = posY_cm;
  currentMove.startOrientation = orientation_rad;
  currentMove.moveState = MOVE_INIT;
}

bool updateMoveTask(MoveCommand &cmd) {
  switch (cmd.moveState) {

    case MOVE_IDLE:
    // Transition to INIT
    cmd.moveState = MOVE_INIT;
    return false;

    case MOVE_INIT:
    // 1) Capture start X, Y 
    cmd.startX = posX_cm;  
    cmd.startY = posY_cm;
    cmd.direction = (cmd.targetDistance > 0.0f) - (cmd.targetDistance < 0.0f);
    cmd.startOrientation = orientation_rad;
    cmd.startTime = millis();
    // 2) Initialize motors
    setMotorSpeed(cmd.speed * cmd.direction, cmd.speed * cmd.direction); 
    // 3) Transition to IN_PROGRESS
    cmd.moveState = MOVE_IN_PROGRESS;
    return false;

    case MOVE_IN_PROGRESS: {
    // Check how far we've traveled  -- CHECK DEPRECATED
    float dx = posX_cm - cmd.startX;
    float dy = posY_cm - cmd.startY;
    float dist = dx * cos(cmd.startOrientation) + dy * sin(cmd.startOrientation);  
    unsigned long elapsedTime = millis() - cmd.startTime;
    
    // Check if distance is reached
    if (fabs(dist) >= fabs(cmd.targetDistance)) {
        // Stop motors
        setMotorSpeed(0,0);
        cmd.moveState = MOVE_COMPLETE;
        return false;
    }
    // Check for timeout
    if (millis() - cmd.startTime > cmd.timeout) {
        // Possibly abort
        cmd.moveState = MOVE_ABORT;
        return false;
    }
    return false; // still in progress
    }

    case MOVE_COMPLETE:
    // Possibly do some finalization
    // Then we declare 'true' => task is done
    return true;

    case MOVE_ABORT:
    // Stop motors, log error, etc.
    setMotorSpeed(0, 0);
    return true;  // done, but aborted
  }
  return true; // default to done if something unexpected
}

bool updateTurnTask(TurnCommand &cmd) {
  // Follows ROS standard where positive turn angle is counterclockwise (left)
  switch (cmd.turnState) {

    case TURN_IDLE:
    // Transition to INIT
    cmd.turnState = TURN_INIT;
    return false;

    case TURN_INIT:
    cmd.targetAngle_rad = cmd.targetAngle * (M_PI / 180.0);
    // 1) Capture start X, Y 
    cmd.startAngle_rad = orientation_rad;
    cmd.startTime = millis();
    // 2) Initialize motors in correct direction
    if (cmd.targetAngle_rad > 0) {
      setMotorSpeed(-cmd.speed, cmd.speed);
    } else {
      setMotorSpeed(cmd.speed, -cmd.speed);
    }
     
    // 3) Transition to IN_PROGRESS
    cmd.turnState = TURN_IN_PROGRESS;
    return false;

    case TURN_IN_PROGRESS: {
      float turnedAngle = orientation_rad - cmd.startAngle_rad;
      // Check if angle is reached
      if (fabs(turnedAngle) >= fabs(cmd.targetAngle_rad) - ANGLE_TOLERANCE) {
          // Stop motors
          setMotorSpeed(0,0);
          cmd.turnState = TURN_COMPLETE;
          return false;
      }
      // Check for timeout
      if (millis() - cmd.startTime > cmd.timeout) {
          cmd.turnState = TURN_ABORT;
          return false;
      }
      return false; // still in progress
    }

    case TURN_COMPLETE:
    // Possibly do some finalization
    // Then we declare 'true' => task is done
    return true;

    case TURN_ABORT:
    // Stop motors, log error, etc.
    setMotorSpeed(0, 0);
    return true;  // done, but aborted
  }
  return true; // default to done if something unexpected
}
