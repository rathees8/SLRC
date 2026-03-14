#include <Arduino.h>
#include <AccelStepper.h>
#include "LineFollower.h"

// Stepper Motor Pins
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 14
#define R_DIR_PIN  12

// 6-Pin HW-511 setup (Far Left to Far Right)
LineFollower sensors(34, 35, 32, 33, 23, 19);

// Create AccelStepper objects
// The "1" tells the library we are using a Step/Dir driver (like your stepper driver)
AccelStepper leftMotor(1, L_STEP_PIN, L_DIR_PIN);
AccelStepper rightMotor(1, R_STEP_PIN, R_DIR_PIN);

void setup() {
  Serial.begin(115200);
  sensors.init();

  // Initial PID Tuning 
  sensors.setPID(1.5, 0.0, 0.5); 

  // --- ACCELSTEPPER SETUP ---
  // 1. Set the absolute maximum speed the motors are allowed to go (Steps per second)
  leftMotor.setMaxSpeed(4000);
  rightMotor.setMaxSpeed(4000);

  // 2. Fix the "Mirrored Motor" issue!
  // This tells the library to permanently flip the HIGH/LOW logic for the Right motor's direction pin.
  // Both motors will now push the robot forward when given a positive speed.
  rightMotor.setPinsInverted(true, false, false); 
  
  Serial.println("Smooth AccelStepper Line Follower starting in 3 seconds...");
  delay(3000);
}

void loop() {
  // 1. Calculate the PID error from the HW-511
  float pidCorrection = sensors.calculatePID();
  
  // 2. Convert PID to Stepper Speed (Steps per second)
  // Higher number = Faster speed. (This is the opposite of the old delay method!)
  float baseSpeed = 800.0; 
  float pidMultiplier = 150.0; // How aggressively it turns
  
  // If PID is positive (line is to the right), left wheel speeds up, right wheel slows down
  float leftSpeed  = baseSpeed + (pidCorrection * pidMultiplier); 
  float rightSpeed = baseSpeed - (pidCorrection * pidMultiplier);

  // Constrain speeds. 
  // Notice we allow it to drop slightly below 0 (e.g., -200). 
  // This allows one wheel to spin slightly backward for very sharp, smooth turns!
  leftSpeed = constrain(leftSpeed, -200, 2500);
  rightSpeed = constrain(rightSpeed, -200, 2500);

  // 3. Set the target speeds
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);

  // 4. Run the motors
  // This function MUST be called as fast as possible in the loop.
  // It checks the clock and fires a step pulse ONLY exactly when needed.
  leftMotor.runSpeed();
  rightMotor.runSpeed();
}                                                         
#include <Arduino.h>
#include <AccelStepper.h>
#include "LineFollower.h"

// Stepper Motor Pins
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 14
#define R_DIR_PIN  12

LineFollower sensors(34, 35, 32, 33, 23, 19);
AccelStepper leftMotor(1, L_STEP_PIN, L_DIR_PIN);
AccelStepper rightMotor(1, R_STEP_PIN, R_DIR_PIN);

struct MotorSpeeds {
    volatile float left;
    volatile float right;
};

MotorSpeeds sharedSpeeds = {0.0, 0.0};

TaskHandle_t MotorTaskHandle;
TaskHandle_t LogicTaskHandle;

void Motor(void * pvParameters){
    MotorSpeeds* speeds = (MotorSpeeds*) pvParameters;
    for(;;){
        leftMotor.setSpeed(speeds->left);
        rightMotor.setSpeed(speeds->right);
        
        leftMotor.runSpeed();
        rightMotor.runSpeed();
        yield();
    }
}

void Movement(void * pvParameters){
    MotorSpeeds* speeds = (MotorSpeeds*) pvParameters;
    for(;;){
        float pidCorrection = sensors.calculatePID();
        float baseSpeed = 800.0; 
        float pidMultiplier = 150.0; // How aggressively it turns
        float leftSpeed  = baseSpeed + (pidCorrection * pidMultiplier); 
        float rightSpeed = baseSpeed - (pidCorrection * pidMultiplier);
        leftSpeed = constrain(leftSpeed, -200, 2500);
        rightSpeed = constrain(rightSpeed, -200, 2500);

        speeds->left = leftSpeed;
        speeds->right = rightSpeed;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void setup(){
    Serial.begin(115200);
    sensors.init();
    sensors.setPID(1.5, 0.0, 0.5);

    leftMotor.setMaxSpeed(4000);
    rightMotor.setMaxSpeed(4000);
    rightMotor.setPinsInverted(true, false, false); 

    xTaskCreatePinnedToCore(
        Motor,
        "Motor",
        4000,
        &sharedSpeeds,
        3,
        &MotorTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        Movement,
        "Movement Logic",
        4000,
        &sharedSpeeds,
        1,
        &LogicTaskHandle,
        1
    );
}

void loop() {
  vTaskDelete(NULL); 
}