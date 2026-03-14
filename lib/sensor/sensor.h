#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class WallSensors{
private:
    int XSHUT_LEFT;
    int XSHUT_RIGHT;
    int XSHUT_FRONT;
    uint16_t thresh = 100;
    VL53L0X sensorLeft;
    VL53L0X sensorFront;
    VL53L0X sensorRight;

public:
    WallSensors(int xl,int xr,int xf): XSHUT_FRONT(xf), XSHUT_LEFT(xl), XSHUT_RIGHT(xr){
        pinMode(XSHUT_LEFT, OUTPUT);
        pinMode(XSHUT_FRONT, OUTPUT);
        pinMode(XSHUT_RIGHT, OUTPUT);
        digitalWrite(XSHUT_LEFT, LOW);
        digitalWrite(XSHUT_FRONT, LOW);
        digitalWrite(XSHUT_RIGHT, LOW);
        delay(10);

        digitalWrite(XSHUT_LEFT, HIGH);
        delay(10);
        sensorLeft.init();
        sensorLeft.setTimeout(500);
        sensorLeft.setAddress(0x30);

        digitalWrite(XSHUT_FRONT, HIGH);
        delay(10);
        sensorFront.init();
        sensorFront.setTimeout(500);
        sensorFront.setAddress(0x31);

        digitalWrite(XSHUT_RIGHT, HIGH);
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