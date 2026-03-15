#ifndef STEPPERDRIVE_H
#define STEPPERDRIVE_H

#include <Arduino.h>

class StepperDrive {
  private:
    // Left Motor Pins
    int stepPinL; // CLK+
    int dirPinL;  // CW+
    
    // Right Motor Pins
    int stepPinR; // CLK+
    int dirPinR;  // CW+

    // Speed control (lower delay = faster speed)
    int stepDelayMicrosec; 

  public:
    // Constructor
    StepperDrive(int spL, int dpL, int spR, int dpR);

    // Initialization
    void init();

    // Movement Methods
    void setSpeed(int delayUs);
    void moveForward(long steps);
    void turnRight(long steps);
    void turnLeft(long steps);
    void stop();
    void turnAngle(double angle);
};

#endif