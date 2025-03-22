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
- MQTT broadcast the local-map to the map-manager
- either 1) recieve command targets, or 2) peform a local frontier search
- generate a PWM signal for the Left and Right motors
- count the encoder pulses
- generate an odometry from the the encoders, update the pose/position in the local map
- path plan on how to get to target, given the local map

Security secretes are located in src/secrets.h  They are ignored from the repo

MQTT
uses broker on same network as Minone
Uses user and password to connect to Broker
