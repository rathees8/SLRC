#ifndef RPICOM_H
#define RPICOM_H

#include <Arduino.h>

class PiLink {
private:
    int targetX;
    int targetY;
    bool freshDataAvailable;

public:
    // Constructor initializes variables to zero
    PiLink() {
        targetX = 0;
        targetY = 0;
        freshDataAvailable = false;
    }

    // Call this in setup()
    void begin(long baudRate = 115200) {
        Serial.begin(baudRate);
        Serial.println("PiLink: UART Initialized. Waiting for coordinates...");
    }

    // Call this continuously in loop()
    void update() {
        // Only read if there is a complete line waiting in the buffer
        if (Serial.available() > 0) {
            String incomingData = Serial.readStringUntil('\n');
            int commaIndex = incomingData.indexOf(',');
            
            // Validate that we actually found a comma
            if (commaIndex > 0) {
                String xString = incomingData.substring(0, commaIndex);
                String yString = incomingData.substring(commaIndex + 1);
                
                targetX = xString.toInt();
                targetY = yString.toInt();
                
                // Flag that new data is ready to be processed by your motors
                freshDataAvailable = true;
            }
        }
    }

    // Checks if new coordinates arrived and resets the flag
    bool hasNewData() {
        if (freshDataAvailable) {
            freshDataAvailable = false; 
            return true;
        }
        return false;
    }

    // Getters to retrieve the coordinates safely
    int getX() { return targetX; }
    int getY() { return targetY; }
};



#endif