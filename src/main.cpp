#include <Arduino.h>
#include <Wire.h>
#include "LineFollower.h"
#include "GridNavigator.h"
#include "StepperDrive.h"
#include "wallsensor.h"
#include "rpicom.h"

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define L_STEP_PIN 25
#define L_DIR_PIN  26
#define R_STEP_PIN 14
#define R_DIR_PIN  12

#define SDA 21
#define SCL 22
#define XSHUT_LEFT  4
#define XSHUT_FRONT 13
#define XSHUT_RIGHT 27 

// HW-511 IR Sensor Pins
#define IR_S1 34 // Far Left
#define IR_S2 35 // Mid Left
#define IR_S3 32 // Inner Left
#define IR_S4 33 // Inner Right
#define IR_S5 23 // Mid Right
#define IR_S6 19 // Far Right

// ==========================================
// HARDWARE INITIALIZATION
// ==========================================
LineFollower sensors(IR_S1, IR_S2, IR_S3, IR_S4, IR_S5, IR_S6);
WallSensors wall(XSHUT_LEFT, XSHUT_RIGHT, XSHUT_FRONT);
StepperDrive drive(L_STEP_PIN, L_DIR_PIN, R_STEP_PIN, R_DIR_PIN);
GridNavigator navigator(&drive, &sensors, 0, 0, 0);
PiLink camera;

// ==========================================
// GLOBAL VARIABLES & RTOS SETUP
// ==========================================
struct MotorData {
    float speedLeft;
    float speedRight;
    long stepLeft;
    long stepRight;
};

MotorData sharedMotorData = {0.0, 0.0, 0, 0};
int pixel_gap = 20; 
int deadzone = 10; 
int boxesCollected = 0;
int sweepSide = 1; 
long stepsToBox = 2000;

TaskHandle_t MotorTaskHandle;
TaskHandle_t LogicTaskHandle;

enum STATE { TASK_1, TASK_2, SIM, TASK_3, TASK_4 };
enum SearchState { LINE_FOLLOW, BOX_FOLLOW, RETURN };

// Boot directly into Task 3
volatile STATE currentState = TASK_3; 
volatile SearchState searchState = LINE_FOLLOW;
volatile bool boxAligned = false;
volatile bool onLine = false;
volatile bool turn = false;
volatile bool pickup = false;

// --- TASK 3 SPECIFIC VARIABLES ---
const int LINE_STATE = LOW; // White line = LOW
float t3_baseSpeed = 800.0;
float t3_pidMultiplier = 150.0;
volatile bool isExecutingTurn = false; // Prevents Core 0 from interfering during point-turns

// ==========================================
// TASK 3 LOGIC FUNCTIONS
// ==========================================
void performLeftTurn_Task3() {
    isExecutingTurn = true; // Lock out Core 0
    Serial.println("Task 3: Left Path Detected!");
    
    drive.turnLeft(); 
    drive.moveForward(400); 
    
    isExecutingTurn = false; // Hand control back to Core 0
}

void perform180Turn_Task3() {
    isExecutingTurn = true; // Lock out Core 0
    Serial.println("Task 3: Line End Detected!");
    
    drive.turnAngle(180.0); 
    drive.moveForward(200); 
    
    isExecutingTurn = false; // Hand control back to Core 0
}

void runTask3Logic(float &outSpeedL, float &outSpeedR) {
    int s1 = digitalRead(IR_S1);
    int s2 = digitalRead(IR_S2);
    int s3 = digitalRead(IR_S3);
    int s4 = digitalRead(IR_S4);
    int s5 = digitalRead(IR_S5);
    int s6 = digitalRead(IR_S6);

    // 1. Dead End Check
    if (s1 != LINE_STATE && s2 != LINE_STATE && s3 != LINE_STATE && 
        s4 != LINE_STATE && s5 != LINE_STATE && s6 != LINE_STATE) {
        
        outSpeedL = 0; outSpeedR = 0; // Stop motor outputs
        drive.stop();
        delay(200); 
        perform180Turn_Task3();
    }
    // 2. Left-Hand Rule Check
    else if (s1 == LINE_STATE && (s3 == LINE_STATE || s4 == LINE_STATE || s6 == LINE_STATE)) {
        outSpeedL = 0; outSpeedR = 0; // Stop motor outputs
        performLeftTurn_Task3();
    }
    // 3. Normal Line Following
    else {
        float pidCorrection = sensors.calculatePID();
        outSpeedL = constrain(t3_baseSpeed + (pidCorrection * t3_pidMultiplier), -200, 2500);
        outSpeedR = constrain(t3_baseSpeed - (pidCorrection * t3_pidMultiplier), -200, 2500);
    }
}

// ==========================================
// CORE 0: MOTOR EXECUTION TASK
// ==========================================
void Motor(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        switch(currentState){
            case TASK_1: break;
            case TASK_2:
                // ... (Task 2 logic remains untouched) ...
                break;
            case TASK_3:
                // Only step the motors if Core 1 isn't currently blocking the thread with a turn
                if (!isExecutingTurn) {
                    drive.MoveCTS(data->speedLeft, data->speedRight);
                }
                break;
            case SIM:
            case TASK_4:
                break;
        }
        taskYIELD(); 
    }
}

// ==========================================
// CORE 1: SENSOR LOGIC & MOVEMENT TASK
// ==========================================
void Movement(void * pvParameters){
    MotorData* data = (MotorData*) pvParameters;
    for(;;){
        switch(currentState) {
            case TASK_1: break;
            case TASK_2:
                // ... (Task 2 logic remains untouched) ...
                break; 
            case SIM: break;
            case TASK_3: {
                // Run the flattened Task 3 logic
                runTask3Logic(data->speedLeft, data->speedRight);
                break;
            }
            case TASK_4: break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// SETUP & LOOP
// ==========================================
void setup(){
    Serial.begin(115200);
    
    sensors.init();
    sensors.setPID(2.5, 0.0, 0.5); // Using the snappier tuning
    
    Wire.begin(SDA, SCL);
    wall.init();
    camera.begin(115200);
    
    drive.init();

    xTaskCreatePinnedToCore(Motor, "Motor", 4000, &sharedMotorData, 3, &MotorTaskHandle, 0);
    xTaskCreatePinnedToCore(Movement, "Movement Logic", 4000, &sharedMotorData, 1, &LogicTaskHandle, 1);
    
    Serial.println("Task 3 Branch Ready!");
}

void loop() {
  vTaskDelete(NULL); 
}