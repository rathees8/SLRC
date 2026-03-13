#include <Arduino.h>
#include "StepperDrive.h"
#include "LineFollower.h"

// Stepper Motor Pins
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 27
#define R_DIR_PIN  14

// 6-Pin HW-511 setup
LineFollower sensors(34, 35, 32, 33, 23, 19);
StepperDrive motors(L_STEP_PIN, L_DIR_PIN, R_STEP_PIN, R_DIR_PIN);

void setup() {
  Serial.begin(115200);
  motors.init();
  sensors.init();

  // Initial PID Tuning (You will change these numbers during testing!)
  sensors.setPID(1.5, 0.0, 0.5); 
  
  Serial.println("Line Follower Calibration Start in 3 seconds...");
  delay(3000);
}

void loop() {
  // 1. Calculate the PID error
  float pidCorrection = sensors.calculatePID();
  
  // 2. Convert PID to Stepper speed adjustments
  int baseDelay = 1200; // Lower number = faster base speed
  int pidMultiplier = 150; // How hard it turns
  
  int leftDelay  = baseDelay - (pidCorrection * pidMultiplier); 
  int rightDelay = baseDelay + (pidCorrection * pidMultiplier);

  // Constrain to prevent the motors from screaming or locking up
  leftDelay = constrain(leftDelay, 600, 3000);
  rightDelay = constrain(rightDelay, 600, 3000);

  // 3. Step the motors independently based on their specific delays
  // Forward direction
  digitalWrite(L_DIR_PIN, HIGH); 
  digitalWrite(R_DIR_PIN, HIGH);

  // Pulse Left Motor
  digitalWrite(L_STEP_PIN, HIGH);
  delayMicroseconds(2); // Short pulse
  digitalWrite(L_STEP_PIN, LOW);
  delayMicroseconds(leftDelay); 

  // Pulse Right Motor
  digitalWrite(R_STEP_PIN, HIGH);
  delayMicroseconds(2); // Short pulse
  digitalWrite(R_STEP_PIN, LOW);
  delayMicroseconds(rightDelay); 
}
