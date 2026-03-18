#include <Arduino.h>
#include <AccelStepper.h>
#include "LineFollower.h"
#include "Task3.h"

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// Stepper Motor Pins
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 14 
#define R_DIR_PIN  12

// HW-511 IR Sensor Pins (s1 to s6)
#define IR_S1 34 // Far Left
#define IR_S2 35 // Mid Left
#define IR_S3 32 // Inner Left
#define IR_S4 33 // Inner Right
#define IR_S5 23 // Mid Right
#define IR_S6 19 // Far Right

// ==========================================
// 2. OBJECT INITIALIZATION
// ==========================================
// Initialize the core hardware objects
LineFollower sensors(IR_S1, IR_S2, IR_S3, IR_S4, IR_S5, IR_S6);
AccelStepper leftMotor(1, L_STEP_PIN, L_DIR_PIN);
AccelStepper rightMotor(1, R_STEP_PIN, R_DIR_PIN);

// Initialize Task 3, passing pointers (&) to the hardware we just created
Task3 taskThree(&leftMotor, &rightMotor, &sensors, IR_S1, IR_S2, IR_S3, IR_S4, IR_S5, IR_S6);

// ==========================================
// 3. SETUP FUNCTION
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Initialize standard sensors and PID
  sensors.init();
  sensors.setPID(2.5, 0.0, 0.5); 

  // Initialize Steppers
  leftMotor.setMaxSpeed(4000);
  rightMotor.setMaxSpeed(4000);
  
  // Acceleration is required for the turn logic to work smoothly
  leftMotor.setAcceleration(2000);
  rightMotor.setAcceleration(2000);

  // Keep pulses wide enough for industrial drivers
  leftMotor.setMinPulseWidth(20); 
  rightMotor.setMinPulseWidth(20);

  // Fix mirrored right motor
  rightMotor.setPinsInverted(true, false, false); 
  
  Serial.println("Task 3 (Left-Hand Rule & Dead Ends) starting in 3 seconds...");
  delay(3000);
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
void loop() {
  // Run the Task 3 logic continuously
  taskThree.runTask();
}