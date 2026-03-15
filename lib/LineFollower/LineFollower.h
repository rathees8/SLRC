#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>

class LineFollower {
  private:
    // Array to hold the 6 digital pins for the HW-511
    // Order: Far-Left (s1) to Far-Right (s6)
    int sensorPins[6]; 
    
    // PID Tuning Variables
    float Kp;
    float Ki;
    float Kd;
    
    float previousError;
    float integral;

  public:
    // Constructor updated for 6 pins
    LineFollower(int s1, int s2, int s3, int s4, int s5, int s6);

    // Initialization
    void init();

    // Set custom PID values
    void setPID(float p, float i, float d);

    // Reads sensors and returns the position error
    int getError(); 

    // Calculates the PID adjustment value
    float calculatePID(); 
};

#endif