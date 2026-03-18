#include "BoxLifter.h"

BoxLifter::BoxLifter(int aPin, int gPin) {
    armPin = aPin;
    gripperPin = gPin;
}

void BoxLifter::init() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    
    // Attach pins (500-2400us is standard for most SG90/MG996R servos)
    armServo.attach(armPin, 500, 2400);
    gripperServo.attach(gripperPin, 500, 2400);

    // Default to a safe driving position
    armServo.write(armUpAngle);
    gripperServo.write(gripperOpenAngle);
}

void BoxLifter::smoothSweep(Servo &servo, int startAngle, int endAngle, int speedDelayMs) {
    if (startAngle < endAngle) {
        for (int pos = startAngle; pos <= endAngle; pos += 1) {
            servo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelayMs)); // FreeRTOS-safe delay
        }
    } else {
        for (int pos = startAngle; pos >= endAngle; pos -= 1) {
            servo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelayMs)); 
        }
    }
}

void BoxLifter::armUp() {
    smoothSweep(armServo, armServo.read(), armUpAngle, 30); // 30ms for a slow, safe sweep
}

void BoxLifter::armDown() {
    smoothSweep(armServo, armServo.read(), armDownAngle, 30);
}

void BoxLifter::openGripper() {
    gripperServo.write(gripperOpenAngle); 
    vTaskDelay(pdMS_TO_TICKS(300));
}

void BoxLifter::closeGripper() {
    gripperServo.write(gripperCloseAngle); 
    vTaskDelay(pdMS_TO_TICKS(500)); // Half-second to ensure a tight grip
}

// ==========================================
// AUTOMATED DEPLOYMENT SEQUENCE
// ==========================================
void BoxLifter::deployBoxSequence() {
    Serial.println("Lifter: Grabbing box from top storage...");
    armUp();                    
    vTaskDelay(pdMS_TO_TICKS(200)); 
    
    closeGripper();               
    vTaskDelay(pdMS_TO_TICKS(400)); 
    
    Serial.println("Lifter: Lowering box to the floor...");
    armDown();                      
    vTaskDelay(pdMS_TO_TICKS(200));
    
    Serial.println("Lifter: Releasing box...");
    openGripper();
    vTaskDelay(pdMS_TO_TICKS(400));
    
    Serial.println("Lifter: Resetting arm for driving...");
    armUp();
}