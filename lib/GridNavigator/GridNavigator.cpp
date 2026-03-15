#include "GridNavigator.h"

GridNavigator::GridNavigator(StepperDrive* m, LineFollower* s, int startX, int startY, int startHeading) {
  motors = m;
  sensors = s;
  
  currentX = startX;
  currentY = startY;
  heading = startHeading;
  
  // You will need to calibrate this exact number on the real arena!
  stepsPerCell = 3200; 
}

int GridNavigator::getX() { return currentX; }
int GridNavigator::getY() { return currentY; }
int GridNavigator::getHeading() { return heading; }

void GridNavigator::moveForwardOneCell() {
  // Ideally, in a non-blocking setup, you would loop step-by-step here 
  // and apply your LineFollower.calculatePID() to adjust motor speeds.
  // For this basic architecture, we command the steppers to move the set distance.
  
  motors->moveForward(stepsPerCell);

  // Update coordinates based on the direction we just traveled
  if (heading == NORTH) {
    currentY++;
  } else if (heading == SOUTH) {
    currentY--;
  } else if (heading == EAST) {
    currentX++;
  } else if (heading == WEST) {
    currentX--;
  }
}

void GridNavigator::turnRight() {
  // 800 steps is a placeholder for a 90-degree turn
  motors->turnRight(800); 
  
  // Update heading (Clockwise)
  heading++;
  if (heading > WEST) {
    heading = NORTH; // Loop back to 0
  }
}

void GridNavigator::turnLeft() {
  motors->turnLeft(800);
  
  // Update heading (Counter-Clockwise)
  heading--;
  if (heading < NORTH) {
    heading = WEST; // Loop back to 3
  }
}

void GridNavigator::turnAround() {
  motors->turnRight(1600); // 180 degree turn
  
  // Update heading
  heading = (heading + 2) % 4;
}