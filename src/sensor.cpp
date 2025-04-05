#include "sensor.h"

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
            sweepAndUpdateMap();  // This function blocks while sweeping
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