#include "BoxLifter.h"

BoxLifter::BoxLifter(int aPin, int gPin, int lPin, int pPin) {
    armPin = aPin; 
    gripperPin = gPin; 
    latchPin = lPin; 
    pusherPin = pPin;
}

void BoxLifter::init() {
    // Allocate all 4 hardware timers for the ESP32
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    armServo.attach(armPin, 500, 2400);
    gripperServo.attach(gripperPin, 500, 2400);
    latchServo.attach(latchPin, 500, 2400);
    pusherServo.attach(pusherPin, 500, 2400);

    // Boot-up state: Empty, open, and retracted
    armServo.write(armUpAngle);
    gripperServo.write(gripperOpen);
    latchServo.write(latchRelease); 
    pusherServo.write(pusherRetracted); 
    
    boxCount = 0;
}

void BoxLifter::smoothSweep(Servo &servo, int startAngle, int endAngle, int speedDelayMs) {
    if (startAngle < endAngle) {
        for (int pos = startAngle; pos <= endAngle; pos += 1) {
            servo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelayMs));
        }
    } else {
        for (int pos = startAngle; pos >= endAngle; pos -= 1) {
            servo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(speedDelayMs)); 
        }
    }
}

// ==========================================
// 6-BOX COLLECTION & SORTING LOGIC
// ==========================================
void BoxLifter::collectAndStoreBox() {
    if (boxCount >= 6) {
        Serial.println("Magazine Full! Cannot collect more.");
        return;
    }

    Serial.println("Collecting Box...");
    
    // 1. Arm moves down (Gripper is already open)
    smoothSweep(armServo, armServo.read(), armDownAngle, 20); 
    vTaskDelay(pdMS_TO_TICKS(100)); // Let arm stabilize
    
    // 2. Clamp and Lift
    gripperServo.write(gripperClose);                         
    vTaskDelay(pdMS_TO_TICKS(400));
    smoothSweep(armServo, armDownAngle, armUpAngle, 20);      

    boxCount++;
    Serial.printf("Processing Box #%d\n", boxCount);

    // 3. FIFO Magazine Sorting
    switch (boxCount) {
        case 1:
            latchServo.write(latchRelease); 
            vTaskDelay(pdMS_TO_TICKS(200));
            gripperServo.write(gripperOpen); // Drop to bottom
            vTaskDelay(pdMS_TO_TICKS(400));
            
            // Shove forward for Box 2
            pusherServo.write(pusherHalfPush);
            vTaskDelay(pdMS_TO_TICKS(300));
            pusherServo.write(pusherRetracted);
            break;

        case 2:
            latchServo.write(latchRelease); 
            vTaskDelay(pdMS_TO_TICKS(200));
            gripperServo.write(gripperOpen); // Drop behind Box 1
            vTaskDelay(pdMS_TO_TICKS(400));
            break;

        case 3:
            latchServo.write(latchHold); // Close the middle latch
            vTaskDelay(pdMS_TO_TICKS(300));
            gripperServo.write(gripperOpen); // Drop onto latch
            break;

        case 4:
        case 5:
            // Drop onto stack
            gripperServo.write(gripperOpen); 
            break;

        case 6:
            Serial.println("Box 6 secured in gripper.");
            // Do not open gripper.
            break;
    }
}

// ==========================================
// DEPLOYMENT & SHIFTING LOGIC
// ==========================================
void BoxLifter::deployAndShiftQueue() {
    if (boxCount == 0) {
        Serial.println("Magazine Empty!");
        return;
    }

    Serial.println("Deploying front box...");

    // 1. Eject Box 1
    pusherServo.write(pusherFullPush);
    vTaskDelay(pdMS_TO_TICKS(500));
    pusherServo.write(pusherRetracted);
    vTaskDelay(pdMS_TO_TICKS(300));

    // 2. Drop exactly ONE box from the middle latch
    if (boxCount >= 3) {
        latchServo.write(latchRelease);
        
        // CRITICAL DELAY: Tune this so only Box 3 falls!
        vTaskDelay(pdMS_TO_TICKS(250)); 
        
        latchServo.write(latchHold);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 3. Drop Box 6 onto the latch stack
    if (boxCount == 6) {
        gripperServo.write(gripperOpen);
    }

    boxCount--;
    Serial.printf("Boxes remaining: %d\n", boxCount);
}