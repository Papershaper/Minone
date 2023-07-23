# Minone
Minone is the MVP of robots
This robot is based on the ESP32
It will be a two wheeled robot
designed to process sensor packages
The overall design should allow for the basic configuration to work with different sized components.
The robot will connect to a MQTT topic and provide telemetry and sensor readings to it.
The goal will be to incorperate localized SLAM capabilities, subject to processing and sensors

Security secretes are located in src/secrets.h  They are ignored from the repo

MQTT
uses broker on same network as Minone
Uses user and password to connect to Broker
