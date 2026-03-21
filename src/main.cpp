#include <Arduino.h>
#include "BoxLifter.h"

// ==========================================
// SERVO PIN DEFINITIONS
// ==========================================
// Check your ESP32 wiring and update these pins!
#define ARM_PIN 15
#define GRIP_PIN 2
#define LATCH_PIN 4
#define PUSHER_PIN 5

BoxLifter lifter(ARM_PIN, GRIP_PIN, LATCH_PIN, PUSHER_PIN);

void setup() {
    Serial.begin(115200);
    delay(1000); 
    
    Serial.println("\n--- BoxLifter Mechanical Test Bench ---");
    Serial.println("Initializing Servos to Boot-up State...");
    
    lifter.init(); 

    Serial.println("\nReady for Commands!");
    Serial.println("Type 'c' and hit Enter -> Collect & Store 1 Box");
    Serial.println("Type 'd' and hit Enter -> Deploy & Shift Queue");
    Serial.println("Type 'r' and hit Enter -> Reset System (Empty Queue)");
    Serial.println("-------------------------------------------");
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        // Clear the buffer of any newline characters
        while(Serial.available() > 0) { Serial.read(); }

        if (command == 'c' || command == 'C') {
            Serial.println("\n>>> COMMAND RECEIVED: Collect Box <<<");
            lifter.collectAndStoreBox();
            Serial.println("Done. Ready for next command.");
        } 
        else if (command == 'd' || command == 'D') {
            Serial.println("\n>>> COMMAND RECEIVED: Deploy Box <<<");
            lifter.deployAndShiftQueue();
            Serial.println("Done. Ready for next command.");
        }
        else if (command == 'r' || command == 'R') {
            Serial.println("\n>>> COMMAND RECEIVED: System Reset <<<");
            lifter.init(); 
            Serial.println("System reset to 0 boxes. Boot state active.");
        }
    }
    
    // FreeRTOS safe delay
    vTaskDelay(pdMS_TO_TICKS(10)); 
}