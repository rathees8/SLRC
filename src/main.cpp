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

// PIN 16,17 for UART communication with Raspberry Pi (PiLink)

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
int boxesCollected = 0;
int sweepSide = 1; 
long stepsToBox = 2000;

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
volatile bool onLine = false;
volatile bool turn = false;
volatile bool pickup = false;

void Motor(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        switch(currentState){
            case TASK_1:
                break;
            case TASK_2:
                switch (searchState){
                    case LINE_FOLLOW:
                        if (turn){
                            drive.turnRight(); 
                            drive.turnRight(); 
                            turn = false;
                        }
                        drive.MoveCTS(data->speedLeft, data->speedRight);
                        break;
                    case BOX_FOLLOW:
                        if (!boxAligned){
                            drive.MoveCTS(data->speedLeft, data->speedRight);
                        }else{
                            drive.stop();
                            drive.turnRight();
                            drive.moveForward(stepsToBox);
                            //picking up the box (servo code would go here) | blocking code | else move to core 1 and block core 0 until pickup is done
                            pickup = false; // Signal that we have reached the box and can move to the next state
                            searchState = RETURN;
                            drive.stop();
                        }
                        break;
                    case RETURN:
                        // Implement return logic here (e.g., navigate back to start)
                        drive.moveBackwards(stepsToBox); // Move back from the box
                        drive.turnLeft();
                        onLine = true;
                        break;
                }
                break;
        }
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
            case TASK_2: {
                switch (searchState) {
                    
                    case LINE_FOLLOW: {
                        if(wall.getReading() & 010){
                            if (sweepSide == 2){
                                currentState = SIM; // Move to next phase after sweeping both sides
                            }else{
                                sweepSide++;
                                turn = true; // Flag to turn in the Motor task
                            }
                        }
                        if (camera.hasNewData()) {
                            // We found it! Lock into Box Follow mode so we don't switch back.
                            searchState = BOX_FOLLOW;
                            data->speedLeft = 0;
                            data->speedRight = 0; // Tap the brakes
                        } else {
                            float pidCorrection = sensors.calculatePID();
                            float baseSpeed = 800.0; 
                            float pidMultiplier = 150.0; 
                            data->speedLeft  = constrain(baseSpeed + (pidCorrection * pidMultiplier), -200, 2500);
                            data->speedRight = constrain(baseSpeed - (pidCorrection * pidMultiplier), -200, 2500);
                        }
                        break;
                    }

                    case BOX_FOLLOW: {
                        // We are locked onto the box. 
                        if (camera.hasNewData()) {
                            int targetX = 320; // Assuming 640px wide camera
                            int currentX = camera.getX();
                            int error = currentX - targetX;

                            // Are we perfectly aligned?
                            if (abs(error) <= deadzone) {
                                data->speedLeft = 0;
                                data->speedRight = 0;
                                boxAligned = true;
                                pickup = true;
                                while(pickup){
                                    // Wait here until the box is picked up before moving to the next state
                                    vTaskDelay(pdMS_TO_TICKS(100));
                                }
                                
                                searchState = RETURN; // Move to next phase
                                
                            } else {
                                data->speedLeft = (error > 0) ? 50 : -50; // Simple proportional control
                                data->speedRight = (error > 0) ? 50 : -50;
                            }
                        } else {
                            // Camera blinked or lost the box!
                            // DO NOT go back to line follow. Just stop and wait for the camera to catch up.
                            data->speedLeft = 0;
                            data->speedRight = 0;
                        }
                        break;
                    }

                    case RETURN: {
                        // Logic to return to the grid/start
                        if (onLine){
                            searchState = LINE_FOLLOW; // Start following the line again
                            onLine = false; // Reset for next time
                        }
                        break;
                    }
                }
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