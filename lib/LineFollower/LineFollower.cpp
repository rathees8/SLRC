#include "LineFollower.h"

// Set this based on your HW-511 behavior. 
// Usually, White Line = LOW (0), Black Floor = HIGH (1)
const int LINE_STATE = LOW; 

LineFollower::LineFollower(int s1, int s2, int s3, int s4, int s5, int s6) {
  sensorPins[0] = s1; // Far Left
  sensorPins[1] = s2; // Mid Left
  sensorPins[2] = s3; // Inner Left
  sensorPins[3] = s4; // Inner Right
  sensorPins[4] = s5; // Mid Right
  sensorPins[5] = s6; // Far Right
  
  // Starting PID values
  Kp = 1.5; 
  Ki = 0.0; 
  Kd = 0.5; 
  
  previousError = 0;
  integral = 0;
}

void LineFollower::init() {
  for(int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void LineFollower::setPID(float p, float i, float d) {
  Kp = p;
  Ki = i;
  Kd = d;
}

int LineFollower::getError() {
  // Read all 6 digital pins
  int s1 = digitalRead(sensorPins[0]);
  int s2 = digitalRead(sensorPins[1]);
  int s3 = digitalRead(sensorPins[2]);
  int s4 = digitalRead(sensorPins[3]);
  int s5 = digitalRead(sensorPins[4]);
  int s6 = digitalRead(sensorPins[5]);

  // 1. Check for an intersection first!
  // If the far-left AND far-right sensors both see the 3 cm line...
  if (s1 == LINE_STATE && s6 == LINE_STATE) {
    return 100; // Special code indicating a grid intersection
  }

  // 2. Determine error based on which sensors are over the white line
  // if (s1 == LINE_STATE) return -3; // Line is far left
  // if (s2 == LINE_STATE) return -2; // Line is mid left
  // if (s3 == LINE_STATE && s4 == LINE_STATE) return 0; // Perfectly centered (straddling)!
  // if (s3 == LINE_STATE) return -1; // Line is slightly left
  // if (s4 == LINE_STATE) return 1;  // Line is slightly right
  // if (s5 == LINE_STATE) return 2;  // Line is mid right
  // if (s6 == LINE_STATE) return 3;  // Line is far right

  int error = s1 * -3 + s2 * -2 + s3 * -1 + s4 * 1 + s5 * 2 + s6 * 3;
  return -error;

  
  // 3. If the line is totally lost, keep turning in the last known direction
  return previousError; 
}

float LineFollower::calculatePID() {
  int error = getError();
  
  // Ignore the intersection code for PID calculations so the robot doesn't twitch
  if (error == 100) return 0; 
  
  integral += error;
  float derivative = error - previousError;
  
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  previousError = error;
  return output;
}