# Minone
Minone is the MVP of robots
This robot is based on the ESP32-S2
It is designed as to be a two wheeled robot
designed to process sensor packages, starting with a single ultrasonic sensor
The overall design should allow for the basic configuration to work with different sized components.
The robot will connect to a MQTT topic and provide telemetry and sensor readings to it.
The goal will be to incorperate localized SLAM capabilities, subject to processing and sensors 

## Basic Funcationality
Despite the MVP status of Minone, it will need to peform the following functions
- Maintain its position and pose on a local occupancy grid (local map)
- peform a 'sensor scan' where the servo is swung and data from the sensor connected
- integrate locally the sensor scan into the local map
- periodic MQTT broadcast the local-map to topic for external map manager consumption
- either 1) recieve MQTT command targets, or 2) peform a local frontier search
- generate a PWM signal for the Left and Right motors
- count the encoder pulses
- generate an odometry from the the encoders, update the pose/position in the local map
- path plan on how to get to target, given the local map

## Excluded from Scope at this time:
- accelerometer for better posture determination
- physical collision detection for better determination of when the odometry is suspect
- wheel slipage detection
- recalibration of the SLAM

## Hardware
- ESP32 -S2
- robot vaccuum wheels, plus encoders
- L298N H-Bridge
- HS-04 Ultrasonic Sensor
- some 9g Metal gear cheap servo
- a 12V/5V battery bank
- currently breadboarded with jumpers and scavanged connectors
- wooden plank
- caster wheel from trashed luggage
- PLA 3D printed parts

## ESP32 Pin Use
| GPIO Number| Usage             | Details                                                             |
|------------|-------------------|---------------------------------------------------------------------|
| 2          | LED_BUILTIN       | On-board LED used for status indication (heartbeat).                |
| 4          | SERVO_PIN         | Controls the servo for ultrasonic sensor sweeping.                  |
| 5          | TRIG_PIN          | Ultrasonic sensor trigger signal.                                   |
| 12         | LEFT_MOTOR_EN     | PWM enable for left motor (controlled via LEDC channel 0).          |
| 13         | LEFT_MOTOR_IN1    | Left motor direction control pin 1.                                 |
| 14         | LEFT_MOTOR_IN2    | Left motor direction control pin 2.                                 |
| 15         | RIGHT_MOTOR_EN    | PWM enable for right motor (controlled via LEDC channel 1).         |
| 16         | RIGHT_MOTOR_IN1   | Right motor direction control pin 1.                                |
| 17         | RIGHT_MOTOR_IN2   | Right motor direction control pin 2.                                |
| 18         | ECHO_PIN          | Ultrasonic sensor echo input.                                       |
| 32         | LEFT_ENCODER_PIN  | Encoder input for the left motor (interrupt enabled).               |
| 33         | RIGHT_ENCODER_PIN | Encoder input for the right motor (interrupt enabled).              |


## Sensor Scan
- The Ultrasonic Sensor is on a structure that can rotate
- a single servo on axis can swing in a defined arc of ~170 degrees
- When performing a Sensor scan, the system will activate the servo go through angles, while the ultrasonic sensor will measure the distance returned. 
- It will collect an array of {angle, distance} value pairs
- The outputed array will be transformed into an occupancy grid (empty, ocupied, or frontier/unexplored)
- the local map will be merged based on robot position, pose, and newly create occupancy grid

# MQTT Overview for Minone

Minone uses MQTT to both receive commands and publish data for remote monitoring and control. The system connects to an MQTT broker (credentials and connection details defined in `secrets.h`), and it follows a simple topic-based messaging structure.

## Connection Details

- **Broker & Credentials:**  
  The ESP32 connects to a broker using the settings defined in `secrets.h` (e.g., `MQTT_BROKER`, `MQTT_PORT`, `MQTT_USERNAME`, and `MQTT_PASSWORD`).

- **Client ID:**  
  The MQTT client generates a unique client ID of the form `ESP32Client-<random_hex>` upon connection.

- **Quality of Service:**  
  The code uses the default MQTT publish settings (typically QoS 0). This may be revisited as needed.

## Subscribed Topics

- **`minone/commands`:**  
  The robot subscribes to this topic to receive command messages.

  **Current Commands:**  
  - **`SET_INTERVAL:<ms>`**  
    - *Purpose:* Adjusts the telemetry publish interval (in milliseconds).  
    - *Example:* `"SET_INTERVAL:3000"` sets the interval to 3000 ms.

  > **Note:** Additional command parsing can be added as new features are implemented.

## Published Topics

- **`minone/telemetry`:**  
  The robot periodically publishes telemetry data. The updated telemetry payload will include:
  
  - **Position:** Robot's current grid coordinates (e.g., `{ "x": 60, "y": 60 }`).
  - **Pose:** Orientation of the robot (to be defined, e.g., in degrees or radians).
  - **Status/Errors:** Any error codes or status messages generated by the system.
  - **WiFi Signal Strength:** RSSI value from the WiFi connection.
  - **Uptime:** Seconds since boot.
  
  **Suggested JSON Structure Example:**
  ```json
  {
    "position": {"x": 60, "y": 60},
    "pose": 0,
    "errors": "",
    "rssi": -45,
    "uptime_sec": 1234
  }
  ```

- **`minone/local_map`:**
    The robot publishes its occupancy grid as a binary payload.

    **Occupancy Grid Details:**

    Dimensions: 120 x 120 cells.
    Cell Definitions:
    - UNKNOWN (255): Unexplored or frontier.
    - FREE (0): Free space.
    - OCCUPIED (1): Detected obstacle.
    - ROBOT (2): The robot's current position.

    Note: The receiver must be aware of the grid dimensions and cell encoding to correctly interpret the data.

## Data Flow Summary
    Startup:
    - The ESP32 connects to WiFi.
    - OTA and MQTT are configured.
    - The occupancy grid is initialized, and the servo is centered.

    Main Loop:
    MQTT connection is maintained, and incoming commands on minone/commands are processed.
    OTA update handling is performed.
    The robot performs a servo sweep to update its local map using ultrasonic sensor readings.
    Telemetry is published on minone/telemetry at intervals defined by telemetryInterval.
    The local map is published on minone/local_map alongside telemetry updates.

    Command Handling:
    Incoming commands (e.g., to change telemetry interval) are parsed in handleMQTTCommands().
    Future commands can be added to control other aspects of the robot.
