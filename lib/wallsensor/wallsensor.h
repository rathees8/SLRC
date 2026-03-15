#ifndef WALLSENSOR_H
#define WALLSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class WallSensors{
private:
    int shut_left;
    int shut_right;
    int shut_front;
    uint16_t thresh = 100;
    VL53L0X sensorLeft;
    VL53L0X sensorFront;
    VL53L0X sensorRight;

public:
    WallSensors(int xl,int xr,int xf): shut_left(xl), shut_right(xr), shut_front(xf){}

    void init(){
        pinMode(shut_left, OUTPUT);
        pinMode(shut_front, OUTPUT);
        pinMode(shut_right, OUTPUT);
        digitalWrite(shut_left, LOW);
        digitalWrite(shut_front, LOW);
        digitalWrite(shut_right, LOW);
        delay(10);

        digitalWrite(shut_left, HIGH);
        delay(10);
        sensorLeft.init();
        sensorLeft.setTimeout(500);
        sensorLeft.setAddress(0x30);

        digitalWrite(shut_front, HIGH);
        delay(10);
        sensorFront.init();
        sensorFront.setTimeout(500);
        sensorFront.setAddress(0x31);

        digitalWrite(shut_right, HIGH);
        delay(10);
        sensorRight.init();
        sensorRight.setTimeout(500);
        sensorRight.setAddress(0x32); 

        sensorLeft.startContinuous();
        sensorFront.startContinuous();
        sensorRight.startContinuous();
    }

    uint8_t getReading(){
        uint8_t state = 0;

        uint16_t distLeft = sensorLeft.readRangeContinuousMillimeters();
        uint16_t distFront = sensorFront.readRangeContinuousMillimeters();
        uint16_t distRight = sensorRight.readRangeContinuousMillimeters();

        if (distLeft < thresh){state |= 0b100;}
        if (distFront < thresh){state |= 0b010;}
        if (distRight < thresh){state |= 0b001;}

        return state;
    }
};


#endif