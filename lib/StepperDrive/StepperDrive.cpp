#include "StepperDrive.h"

double anglePerStep = 1.8;

StepperDrive::StepperDrive(int spL, int dpL, int spR, int dpR) {
  stepPinL = spL;
  dirPinL = dpL;
  stepPinR = spR;
  dirPinR = dpR;
  
  // Default speed (adjust based on your motor's specs)
  stepDelayMicrosec = 1000; 
}

void StepperDrive::init() {
  leftMotor = new AccelStepper(1, stepPinL, dirPinL);
  rightMotor = new AccelStepper(1, stepPinR, dirPinR);
  leftMotor->setMaxSpeed(4000);
  rightMotor->setMaxSpeed(4000);
}

void StepperDrive::setSpeed(int delayUs) {
  stepDelayMicrosec = delayUs;
}

void StepperDrive::step(int stepsL, int stepsR) {
    leftMotor->move(stepsL);
    rightMotor->move(stepsR);
    leftMotor->run();
    rightMotor->run();
}

void StepperDrive::moveForward(long steps) {
    step(steps, steps);
}

void StepperDrive::turnRight(long steps) {
    turnAngle(90);
}

void StepperDrive::turnLeft(long steps) {
    turnAngle(-90);
}

void StepperDrive::MoveCTS(double speedL, double speedR) {
    leftMotor->setSpeed(speedL);
    rightMotor->setSpeed(speedR);
    leftMotor->runSpeed();
    rightMotor->runSpeed();
}

void StepperDrive::stop() {
    leftMotor->stop();
    rightMotor->stop();
}

void StepperDrive::turnAngle(double angle){
    long steps = (long)(angle / anglePerStep);
    step(steps, -steps); 
}