#ifndef GRIDNAVIGATOR_H
#define GRIDNAVIGATOR_H

#include "StepperDrive.h"
#include "LineFollower.h"

// Define Heading Constants
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

class GridNavigator {
  private:
    StepperDrive* motors;
    LineFollower* sensors;
    
    int currentX;
    int currentY;
    int heading;

    // Steps required to move exactly one 40cm cell
    long stepsPerCell;

  public:
    // Constructor passes in your existing motor and sensor objects
    GridNavigator(StepperDrive* m, LineFollower* s, int startX, int startY, int startHeading);

    // Coordinate Tracking
    int getX();
    int getY();
    int getHeading();

    // Movement Commands
    void moveForwardOneCell();
    void turnRight();
    void turnLeft();
    void turnAround();
};

#endif