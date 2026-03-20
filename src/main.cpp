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
#define R_STEP_PIN 18
#define R_DIR_PIN  19

// servo PIN
#define SERVO_1_PIN 23 
#define SERVO_2_PIN 2  
#define SERVO_3_PIN 4   

// TOF Sensor Pins
#define SDA 21
#define SCL 22
#define XSHUT_LEFT  14
#define XSHUT_FRONT 13
#define XSHUT_RIGHT 27 

// Controllers and Feedback
LineFollower sensors(34, 35, 32, 33, 39, 36);
AccelStepper leftMotor(1, L_STEP_PIN, L_DIR_PIN);
AccelStepper rightMotor(1, R_STEP_PIN, R_DIR_PIN);
WallSensors wall(XSHUT_LEFT, XSHUT_RIGHT, XSHUT_FRONT);
StepperDrive drive(L_STEP_PIN, L_DIR_PIN, R_STEP_PIN, R_DIR_PIN);
GridNavigator navigator(&drive, &sensors, 0, 0, 0);
PiLink camera; // This is your UART connection!

struct MotorData {
    float speedLeft;
    float speedRight;
    long stepLeft;
    long stepRight;
};

// Global variables
MotorData sharedMotorData = {0.0, 0.0, 0, 0};
SemaphoreHandle_t motorDataMutex;
int pixel_gap = 20; 
int deadzone = 10; 
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

volatile STATE currentState = TASK_2;
volatile SearchState searchState = LINE_FOLLOW;
volatile bool boxAligned = false;
volatile bool onLine = false;
volatile bool turn = false;
volatile bool pickup = false;

void Motor(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    float localSpeedL = 0;
    float localSpeedR = 0;
    
    for(;;){
        switch(currentState){
            case TASK_1:
                break;
            case TASK_2: {
                if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    localSpeedL = data->speedLeft;
                    localSpeedR = data->speedRight;
                    xSemaphoreGive(motorDataMutex); 
                }

                switch(searchState){
                    case LINE_FOLLOW:
                        if (turn){
                            drive.turnRight(); 
                            drive.turnRight(); 
                            turn = false;
                        }
                        drive.MoveCTS(localSpeedL, localSpeedR);
                        break;
                        
                    case BOX_FOLLOW:
                        if (!boxAligned){
                            drive.MoveCTS(localSpeedL, localSpeedR);
                        } else {
                            Serial.print("GOAT");
                            drive.stop();
                            drive.turnRight();             
                            drive.moveForward(stepsToBox); 
                            
                            // picking up the box (servo code would go here)
                            pickup = false; 
                            searchState = RETURN;
                            drive.stop();
                        }
                        break;
                        
                    case RETURN:
                        drive.moveBackwards(stepsToBox); 
                        drive.turnLeft();                
                        onLine = true;
                        break;
                }
                break; 
            }
            case SIM:
                break;
            case TASK_3:
                break;
            case TASK_4:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void Movement(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        // FIXED: Constantly process the UART buffer so hasNewData() actually works!
        camera.update();

        switch(currentState) {
            case TASK_1: { 
                break;
            }
            case TASK_2: {
                switch(searchState){
                    case LINE_FOLLOW:{
                        if (camera.hasNewData()) {
                            Serial.println("BOX SPOTTED! Switching to BOX_FOLLOW");
                            searchState = BOX_FOLLOW;
                            
                            if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                                data->speedLeft = 0;
                                data->speedRight = 0; 
                                xSemaphoreGive(motorDataMutex);
                            }
                        } else {
                            float pidCorrection = sensors.calculatePID();
                            float baseSpeed = 40.0; 
                            float pidMultiplier = 10.0; 
                            
                            if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                                data->speedLeft  = constrain(baseSpeed + (pidCorrection * pidMultiplier), 0, 500);
                                data->speedRight = constrain(baseSpeed - (pidCorrection * pidMultiplier), 0, 500);                                
                                xSemaphoreGive(motorDataMutex);
                            }
                        }
                        break;
                    }
                    case BOX_FOLLOW: {
                        if (camera.hasNewData()) {
                            int targetX = 320; 
                            int currentX = camera.getX();
                            int error = currentX - targetX;
                            
                            if (abs(error) <= deadzone) {
                                if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                                    data->speedLeft = 0;
                                    data->speedRight = 0;
                                    xSemaphoreGive(motorDataMutex);
                                }
                                
                                boxAligned = true;
                                pickup = true;
                                while(pickup){
                                    vTaskDelay(pdMS_TO_TICKS(100));
                                }
                                    
                                searchState = RETURN; 
                                
                            } else {
                                if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                                    data->speedLeft = (error > 0) ? 50 : -50; 
                                    data->speedRight = (error > 0) ? 50 : -50;
                                    xSemaphoreGive(motorDataMutex);
                                }
                            }
                        } else {
                            if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                                data->speedLeft = 0;
                                data->speedRight = 0;
                                xSemaphoreGive(motorDataMutex);
                            }
                        }
                        break;
                    }
                    case RETURN: {
                        break;
                    }
                }
                break; 
            }
            case SIM:
                break;
            case TASK_3:
                break;
            case TASK_4:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup(){
    Serial.begin(115200);
    sensors.init();
    Wire.begin(SDA, SCL);
    wall.init();
    sensors.setPID(1.5, 0.0, 0);
    camera.begin(115200);
    drive.init();
    motorDataMutex = xSemaphoreCreateMutex();

    leftMotor.setMaxSpeed(400);
    rightMotor.setMaxSpeed(400);

    xTaskCreatePinnedToCore(Motor, "Motor", 4000, &sharedMotorData, 3, &MotorTaskHandle, 0);
    xTaskCreatePinnedToCore(Movement, "Movement Logic", 4000, &sharedMotorData, 1, &LogicTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL); 
}