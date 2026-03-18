#include <Arduino.h>
#include <AccelStepper.h>
#include <LineFollower.h>
#include <GridNavigator.h>
#include <StepperDrive.h>
#include <Wire.h>
#include <wallsensor.h>
#include <rpicom.h>

// Stepper Motor Pins
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 14
#define R_DIR_PIN  12

// TOF Sensor Pins
#define SDA 21
#define SCL 22
#define XSHUT_LEFT  4
#define XSHUT_FRONT 13
#define XSHUT_RIGHT 27 

// Controllers and Feedback
LineFollower sensors(34, 35, 32, 33, 23, 19);
AccelStepper leftMotor(1, L_STEP_PIN, L_DIR_PIN);
AccelStepper rightMotor(1, R_STEP_PIN, R_DIR_PIN);
WallSensors wall(XSHUT_LEFT, XSHUT_RIGHT, XSHUT_FRONT);
StepperDrive drive(L_STEP_PIN, L_DIR_PIN, R_STEP_PIN, R_DIR_PIN);
GridNavigator navigator(&drive, &sensors, 0, 0, 0);
PiLink camera;

struct MotorData {
    float speedLeft;
    float speedRight;
    long stepLeft;
    long stepRight;
};

// Global variables
MotorData sharedMotorData = {0.0, 0.0, 0, 0};
int pixel_gap = 20; // for perfect rotation, adjust it until 90 turn becomes straight to cube.
int deadzone = 10; // pixels within the center that are considered "close enough"

TaskHandle_t MotorTaskHandle;
TaskHandle_t LogicTaskHandle;

// States
enum STATE{
  TASK_1,
  TASK_2,
  SIM,
  TASK_3,
  TASK_4
};

enum SearchState{
    LINE_FOLLOW,
    BOX_FOLLOW,
    RETURN
};

volatile STATE currentState = TASK_1;
volatile SearchState searchState = LINE_FOLLOW;
volatile bool boxAligned = false;

void Motor(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        switch(currentState){
            case TASK_1:
                break;
            case TASK_2:
                switch (searchState){
                    case LINE_FOLLOW:
                        drive.MoveCTS(data->speedLeft, data->speedRight);
                        break;
                    case BOX_FOLLOW:
                        if (!boxAligned){
                            drive.step(data->stepLeft, data->stepRight);
                        }else{
                            drive.stop();
                        }
                        break;
                }
                break;
        }
        // leftMotor.setSpeed(data->left);
        // rightMotor.setSpeed(data->right);
        
        // leftMotor.runSpeed();
        // rightMotor.runSpeed();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Movement(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        switch(currentState) {
            case TASK_1: {
                DFS(wall, navigator);
                currentState = TASK_2; 
                break;
            }
            case TASK_2:{
                if (camera.hasNewData()){
                    int targetX = 320;
                    int currentX = camera.getX();
                    if (currentX > targetX + pixel_gap - deadzone ){
                        data->stepLeft = 10;
                        data->stepRight = 10;
                    }
                    // so we align the camera then execute the move and pickup logic, until then it doesn't go to else state. 
                    // then we go to return then only it goes to else. this is annoying. next is on wednesday need to test it tommorow.
                }else{
                    float pidCorrection = sensors.calculatePID();
                    float baseSpeed = 800.0; 
                    float pidMultiplier = 150.0; // How aggressively it turns
                    float leftSpeed  = baseSpeed + (pidCorrection * pidMultiplier); 
                    float rightSpeed = baseSpeed - (pidCorrection * pidMultiplier);
                    leftSpeed = constrain(leftSpeed, -200, 2500);
                    rightSpeed = constrain(rightSpeed, -200, 2500);

                    data->speedLeft = leftSpeed;
                    data->speedRight = rightSpeed;
                }
                break;
            }
            case SIM:{
                // Implementation for SIM
                break;
            }
            case TASK_3:{
                // Implementation for TASK_3
                break;
            }
            case TASK_4:{
                // Implementation for TASK_4
                break;
            }
        }
        

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void setup(){
    Serial.begin(115200);
    sensors.init();
    Wire.begin(SDA, SCL);
    wall.init();
    sensors.setPID(1.5, 0.0, 0.5);
    camera.begin(115200);
    drive.init();

    leftMotor.setMaxSpeed(4000);
    rightMotor.setMaxSpeed(4000);
    rightMotor.setPinsInverted(true, false, false); 

    xTaskCreatePinnedToCore(
        Motor,
        "Motor",
        4000,
        &sharedMotorData,
        3,
        &MotorTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        Movement,
        "Movement Logic",
        4000,
        &sharedMotorData,
        1,
        &LogicTaskHandle,
        1
    );
}

void loop() {
  vTaskDelete(NULL); 
}