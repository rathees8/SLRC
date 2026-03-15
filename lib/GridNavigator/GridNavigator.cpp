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

struct Point { 
    int x; 
    int y; 
};

void DFS(WallSensors &sensors, GridNavigator &nav) {
    bool visited[6][6] = {false};
    for (int x = 3; x <= 5; x++) {
        for (int y = 0; y <= 4; y++) {
            visited[x][y] = true; 
        }
    }
    Point pathStack[36];
    int stackTop = -1;
    int visitedCount = 0;
    visited[nav.getX()][nav.getY()] = true;
    visitedCount++;


    while(true) {
        int currX = nav.getX();
        int currY = nav.getY();
        int currH = nav.getHeading(); 


        if (visitedCount == 21 && currX == 5 && currY == 5) {
            break;
        }

        uint8_t w = sensors.getReading();

        // Helper lambdas to calculate neighbor coordinates based on heading
        auto getAdjX = [](int x, int h) { return (h == 1) ? x+1 : (h == 3) ? x-1 : x; };
        auto getAdjY = [](int y, int h) { return (h == 0) ? y+1 : (h == 2) ? y-1 : y; };

        int targetHeading = -1;

        // --- SCAN PRIORITIES: Right -> Front -> Left ---
        
        // Check Right (0b001)
        int rightH = (currH + 1) % 4;
        int rightX = getAdjX(currX, rightH);
        int rightY = getAdjY(currY, rightH);
        if (!(w & 0b001) && rightX >= 0 && rightX <= 5 && rightY >= 0 && rightY <= 5 && !visited[rightX][rightY]) {
            // Prevent entering 5,5 too early! Only allow it if it's the absolute last block.
            if (!(rightX == 5 && rightY == 5 && visitedCount < 20)) {
                targetHeading = rightH;
            }
        }
        
        // Check Front (0b010) - Only if Right is blocked/visited
        if (targetHeading == -1 && !(w & 0b010)) {
            int frontX = getAdjX(currX, currH);
            int frontY = getAdjY(currY, currH);
            if (frontX >= 0 && frontX <= 5 && frontY >= 0 && frontY <= 5 && !visited[frontX][frontY]) {
                if (!(frontX == 5 && frontY == 5 && visitedCount < 20)) {
                    targetHeading = currH;
                }
            }
        }
        
        // Check Left (0b100) - Only if Right and Front are blocked/visited
        int leftH = (currH + 3) % 4;
        if (targetHeading == -1 && !(w & 0b100)) {
            int leftX = getAdjX(currX, leftH);
            int leftY = getAdjY(currY, leftH);
            if (leftX >= 0 && leftX <= 5 && leftY >= 0 && leftY <= 5 && !visited[leftX][leftY]) {
                if (!(leftX == 5 && leftY == 5 && visitedCount < 20)) {
                    targetHeading = leftH;
                }
            }
        }

        // --- EXECUTE THE MOVE ---
        if (targetHeading != -1) {
            // Unvisited Path Found! 
            // 1. Save current spot to stack so we can find our way back
            stackTop++;
            pathStack[stackTop].x = currX;
            pathStack[stackTop].y = currY;

            // 2. Turn to face the new path
            int diff = (targetHeading - currH + 4) % 4;
            if (diff == 1) nav.turnRight();
            else if (diff == 3) nav.turnLeft();

            // 3. Drive forward one physical cell
            nav.moveForwardOneCell();

            // 4. Update the Map
            visited[nav.getX()][nav.getY()] = true;
            visitedCount++;
            
        } else {
            // DEAD END! Time to backtrack.
            if (stackTop < 0) {
                Serial.println("Error: Stack Empty before reaching 5,5!");
                break; // Failsafe
            }

            // Look at the last coordinate we visited
            Point prev = pathStack[stackTop];
            stackTop--; // "Pop" it off the stack

            // Figure out which way we need to face to get back there
            int backH = currH;
            if (prev.x > currX) backH = 1; // Need to go East
            else if (prev.x < currX) backH = 3; // Need to go West
            else if (prev.y > currY) backH = 0; // Need to go North
            else if (prev.y < currY) backH = 2; // Need to go South

            // Turn to face the previous cell
            int diff = (backH - currH + 4) % 4;
            if (diff == 1) nav.turnRight();
            else if (diff == 2) nav.turnAround(); // U-Turn!
            else if (diff == 3) nav.turnLeft();

            // Drive back
            nav.moveForwardOneCell();
            
            // Note: We do NOT update the visited count here, because we've already been here!
        }
        
        // Tiny FreeRTOS delay to prevent Watchdog timeout during this intense loop
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}