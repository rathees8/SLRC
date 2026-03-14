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