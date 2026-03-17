#ifndef STEPPERDRIVE_H
#define STEPPERDRIVE_H

#include <Arduino.h>
#include <AccelStepper.h>

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

    AccelStepper* leftMotor;
    AccelStepper* rightMotor;

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
    void step(int stepsL, int stepsR);
    void MoveCTS(double speedL, double speedR);
};

#endif