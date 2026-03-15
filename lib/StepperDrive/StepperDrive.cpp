#include "StepperDrive.h"

StepperDrive::StepperDrive(int spL, int dpL, int spR, int dpR) {
  stepPinL = spL;
  dirPinL = dpL;
  stepPinR = spR;
  dirPinR = dpR;
  
  // Default speed (adjust based on your motor's specs)
  stepDelayMicrosec = 1000; 
}

void StepperDrive::init() {
  pinMode(stepPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
}

void StepperDrive::setSpeed(int delayUs) {
  stepDelayMicrosec = delayUs;
}

void StepperDrive::moveForward(long steps) {
  // Set directions for forward movement
  // (You may need to flip HIGH/LOW based on your physical wiring)
  digitalWrite(dirPinL, HIGH); 
  digitalWrite(dirPinR, HIGH); 

  // Generate pulses to move the motors
  for(long i = 0; i < steps; i++) {
    digitalWrite(stepPinL, HIGH);
    digitalWrite(stepPinR, HIGH);
    delayMicroseconds(stepDelayMicrosec);
    
    digitalWrite(stepPinL, LOW);
    digitalWrite(stepPinR, LOW);
    delayMicroseconds(stepDelayMicrosec);
  }
}

void StepperDrive::turnRight(long steps) {
  // Left motor forward, Right motor backward
  digitalWrite(dirPinL, HIGH); 
  digitalWrite(dirPinR, LOW);  

  for(long i = 0; i < steps; i++) {
    digitalWrite(stepPinL, HIGH);
    digitalWrite(stepPinR, HIGH);
    delayMicroseconds(stepDelayMicrosec);
    
    digitalWrite(stepPinL, LOW);
    digitalWrite(stepPinR, LOW);
    delayMicroseconds(stepDelayMicrosec);
  }
}

void StepperDrive::turnLeft(long steps) {
  // Left motor backward, Right motor forward
  digitalWrite(dirPinL, LOW); 
  digitalWrite(dirPinR, HIGH);  

  for(long i = 0; i < steps; i++) {
    digitalWrite(stepPinL, HIGH);
    digitalWrite(stepPinR, HIGH);
    delayMicroseconds(stepDelayMicrosec);
    
    digitalWrite(stepPinL, LOW);
    digitalWrite(stepPinR, LOW);
    delayMicroseconds(stepDelayMicrosec);
  }
}

void StepperDrive::stop() {

}

void StepperDrive::turnAngle(double angle){
  
}