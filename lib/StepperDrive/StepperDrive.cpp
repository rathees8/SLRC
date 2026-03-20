#include "StepperDrive.h"

// --- CALIBRATION VARIABLE ---
// You will need to physically test and tune this number!
// It is the number of motor steps required to rotate the WHOLE ROBOT exactly 90 degrees.
long stepsFor90Degrees = 800; 

StepperDrive::StepperDrive(int spL, int dpL, int spR, int dpR) {
  stepPinL = spL;
  dirPinL = dpL;
  stepPinR = spR;
  dirPinR = dpR;
  
  stepDelayMicrosec = 1000; // Default fallback
}

void StepperDrive::init() {
  leftMotor = new AccelStepper(1, stepPinL, dirPinL);
  rightMotor = new AccelStepper(1, stepPinR, dirPinR);
  
  // You MUST set Max Speed AND Acceleration for the run() function to work!
  leftMotor->setPinsInverted(true, false, false); 
  leftMotor->setMaxSpeed(4000);
  rightMotor->setMaxSpeed(4000);
}

void StepperDrive::setSpeed(int speed) {
  stepDelayMicrosec = speed;
  leftMotor->setMaxSpeed(speed);
  rightMotor->setMaxSpeed(speed);
}


long StepperDrive::getLeftPosition() {
    return leftMotor->currentPosition();
}

void StepperDrive::step(int stepsL, int stepsR) {
    // 1. Set the target destinations relative to current position
    leftMotor->move(stepsL);
    rightMotor->move(stepsR);

    // 2. Trap the code here until BOTH motors reach their targets
    while (leftMotor->distanceToGo() != 0 || rightMotor->distanceToGo() != 0) {
        leftMotor->run();
        rightMotor->run();
        
        // Feed the ESP32 Watchdog so it doesn't crash while driving
        yield(); 
    }
}

void StepperDrive::moveForward(long steps) {
    step(steps, steps);
}
void StepperDrive::moveBackwards(long steps) {
    step(-steps, -steps);
}

void StepperDrive::turnRight() {
    step(stepsFor90Degrees, -stepsFor90Degrees);
}

void StepperDrive::turnLeft() {
    step(-stepsFor90Degrees, stepsFor90Degrees);
}

// --- Used by GridNavigator for continuous PID steering ---
void StepperDrive::MoveCTS(double speedL, double speedR) {
    leftMotor->setSpeed(speedL);
    rightMotor->setSpeed(speedR);
    
    // runSpeed() does NOT use acceleration, it instantly applies the speed.
    // Perfect for real-time PID adjustments!
    leftMotor->runSpeed();
    rightMotor->runSpeed();
}

void StepperDrive::stop() {
    leftMotor->stop();
    rightMotor->stop();
}

void StepperDrive::turnAngle(double angle){
    // Calculate ratio based on our known 90-degree step count
    long steps = (long)((angle / 90.0) * stepsFor90Degrees);
    step(steps, -steps); 
}