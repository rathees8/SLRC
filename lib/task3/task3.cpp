#include "Task3.h"

Task3::Task3(AccelStepper* lMotor, AccelStepper* rMotor, LineFollower* lf, 
             int s1, int s2, int s3, int s4, int s5, int s6) {
  leftMotor = lMotor;
  rightMotor = rMotor;
  sensors = lf;
  
  s1_pin = s1; 
  s2_pin = s2; 
  s3_pin = s3; 
  s4_pin = s4; 
  s5_pin = s5; 
  s6_pin = s6; 
  
  // White Line = LOW (0), Black Floor = HIGH (1)
  lineState = LOW; 
  
  // Starting tuning values for this specific task
  turnSteps = 800; // Calibrate this for exactly 90 degrees
  baseSpeed = 600.0;
  pidMultiplier = 150.0;
}

void Task3::performLeftTurn() {
  Serial.println("Task 3: Left Path Detected! Executing 90-degree Turn.");

  // 1. Reset step counters
  leftMotor->setCurrentPosition(0);
  rightMotor->setCurrentPosition(0);

  // 2. Pivot left (Left wheel back, Right wheel forward)
  leftMotor->moveTo(-turnSteps); 
  rightMotor->moveTo(turnSteps); 

  // 3. Execute the precise turn
  while (leftMotor->distanceToGo() != 0 || rightMotor->distanceToGo() != 0) {
    leftMotor->run();
    rightMotor->run();
  }

  // 4. Drive straight forward briefly to clear the junction line
  leftMotor->setCurrentPosition(0);
  rightMotor->setCurrentPosition(0);
  leftMotor->moveTo(400); 
  rightMotor->moveTo(400);
  
  while (leftMotor->distanceToGo() != 0 || rightMotor->distanceToGo() != 0) {
    leftMotor->run();
    rightMotor->run();
  }
}

void Task3::perform180Turn() {
  Serial.println("Task 3: Line End Detected! Executing 180-degree Turn.");

  leftMotor->setCurrentPosition(0);
  rightMotor->setCurrentPosition(0);

  // Multiply the 90-degree steps by 2 for a full U-turn
  long turn180Steps = turnSteps * 2; 

  leftMotor->moveTo(-turn180Steps); 
  rightMotor->moveTo(turn180Steps); 

  while (leftMotor->distanceToGo() != 0 || rightMotor->distanceToGo() != 0) {
    leftMotor->run();
    rightMotor->run();
  }
  
  // Creep forward slightly to ensure center sensors find the line again
  leftMotor->setCurrentPosition(0);
  rightMotor->setCurrentPosition(0);
  leftMotor->moveTo(200); 
  rightMotor->moveTo(200);
  
  while (leftMotor->distanceToGo() != 0 || rightMotor->distanceToGo() != 0) {
    leftMotor->run();
    rightMotor->run();
  }
  
  Serial.println("Task 3: 180 Turn complete. Heading back.");
}

void Task3::followLine() {
  float pidCorrection = sensors->calculatePID();
  
  float leftSpeed  = baseSpeed + (pidCorrection * pidMultiplier); 
  float rightSpeed = baseSpeed - (pidCorrection * pidMultiplier);

  leftSpeed = constrain(leftSpeed, -200, 2500);
  rightSpeed = constrain(rightSpeed, -200, 2500);

  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);

  leftMotor->runSpeed();
  rightMotor->runSpeed();
}

void Task3::runTask() {
  int s1 = digitalRead(s1_pin);
  int s2 = digitalRead(s2_pin);
  int s3 = digitalRead(s3_pin);
  int s4 = digitalRead(s4_pin);
  int s5 = digitalRead(s5_pin);
  int s6 = digitalRead(s6_pin);

  // 1. Dead End Check (All sensors see the black floor)
  if (s1 != lineState && s2 != lineState && s3 != lineState && 
      s4 != lineState && s5 != lineState && s6 != lineState) {
    
    // Stop completely before spinning
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
    leftMotor->runSpeed();
    rightMotor->runSpeed();
    delay(200); 
    
    perform180Turn();
  }
  
  // 2. Left-Hand Rule Check
  // If Far-Left sees the line AND (Center sees the line OR Far-Right sees the line)
  else if (s1 == lineState && (s3 == lineState || s4 == lineState || s6 == lineState)) {
    performLeftTurn();
  }
  
  // 3. Normal Line Following
  else {
    followLine();
  }
}