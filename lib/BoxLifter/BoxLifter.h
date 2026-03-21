#ifndef BOXLIFTER_H
#define BOXLIFTER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class BoxLifter {
  private:
    Servo armServo;
    Servo gripperServo;
    Servo latchServo; 
    Servo pusherServo; 
    
    int armPin, gripperPin, latchPin, pusherPin;

    // --- CALIBRATION ANGLES ---
    // You will tune these numbers during your bench test!
    int armDownAngle = 10;      
    int armUpAngle = 120;       

    int gripperOpen = 90;       
    int gripperClose = 10;      

    int latchHold = 90;         
    int latchRelease = 0;       

    int pusherRetracted = 0;    
    int pusherHalfPush = 45;    
    int pusherFullPush = 90;    

    int boxCount = 0;           // State memory

    void smoothSweep(Servo &servo, int startAngle, int endAngle, int speedDelayMs);

  public:
    BoxLifter(int aPin, int gPin, int lPin, int pPin);
    
    void init();
    void collectAndStoreBox(); 
    void deployAndShiftQueue();
};

#endif