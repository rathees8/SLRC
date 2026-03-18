#ifndef BOXLIFTER_H
#define BOXLIFTER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class BoxLifter {
  private:
    Servo armServo;
    Servo gripperServo;
    
    int armPin;
    int gripperPin;

    // --- CALIBRATION ANGLES ---
    // You must physically tune these so the arm aligns perfectly with your top storage deck!
    int armUpAngle = 90;       // Top storage position
    int armDownAngle = 10;     // Floor deployment position
    int gripperOpenAngle = 90; // Released
    int gripperCloseAngle = 10;// Grabbed

    // Internal helpers
    void smoothSweep(Servo &servo, int startAngle, int endAngle, int speedDelayMs);
    void armUp();
    void armDown();
    void openGripper();
    void closeGripper();

  public:
    BoxLifter(int aPin, int gPin);
    
    void init();
    void deployBoxSequence(); 
};

#endif