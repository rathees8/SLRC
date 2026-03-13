#include <Arduino.h>

// HW-511 IR Sensor Pins (Far Left to Far Right)
const int IR_PINS[6] = {34, 35, 32, 33, 23, 19};

void setup() {
  Serial.begin(115200);
  
  // Set all pins as inputs
  for(int i = 0; i < 6; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  
  Serial.println("HW-511 Sensor Test Initialized.");
  Serial.println("Place over the black floor and white line to test.");
  delay(2000);
}

void loop() {
  Serial.print("Sensors (Left -> Right): [ ");
  
  for(int i = 0; i < 6; i++) {
    int sensorValue = digitalRead(IR_PINS[i]);
    Serial.print(sensorValue);
    Serial.print(" ");
  }
  
  Serial.println("]");
  
  // Wait a little bit so the Serial Monitor isn't flooded too fast
  delay(250); 
}