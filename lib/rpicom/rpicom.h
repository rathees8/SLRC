#ifndef RPICOM_H
#define RPICOM_H

#include <Arduino.h>
#include <ESP32Servo.h>

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


class camera {
private:
    int servopin;
    double angle;
    Servo panServo; // Create a Servo object from the ESP32Servo library

public:
    // Constructor
    camera(int SERVOPIN) : servopin(SERVOPIN), angle(90.0) {}

    // Initialize the servo
    void init() {
        // Attach the servo to the designated pin
        panServo.attach(servopin,1000,2000);
        
        // Optionally move to the center position on startup
        centre();
    }
    
    // Move camera to 0 degrees
    void turnleft() {
        angle = 0;
        panServo.write(0); // The library maps 0-180 directly to pulse widths
    }

    // Move camera to 180 degrees
    void turnRight() {
        angle = 180;
        panServo.write(180); // The library maps 0-180 directly to pulse widths
    }

    // Move camera to 90 degrees (Center)
    void centre() {
        angle = 90;
        panServo.write(90); // The library maps 0-180 directly to pulse widths
    }
};

#endif