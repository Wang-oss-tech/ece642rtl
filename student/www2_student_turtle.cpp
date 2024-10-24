#include "student.h"
#include <stdint.h>  

const int32_t MAZE_SIZE = 100; // Increased size to handle more complex mazes
static int32_t currentX = START_POS;
static int32_t currentY = START_POS;
static int32_t numTurns = 0;  // Track the number of turns
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // Track visit counts

// Check each direction and return the visit count or -1 if the direction is blocked
int checkDirection(int direction) {
    switch (direction) {
        case NORTH: return (currentX > 0) ? getVisits(currentX - 1, currentY) : -1;
        case EAST:  return (currentY < MAZE_SIZE - 1) ? getVisits(currentX, currentY + 1) : -1;
        case SOUTH: return (currentX < MAZE_SIZE - 1) ? getVisits(currentX + 1, currentY) : -1;
        case WEST:  return (currentY > 0) ? getVisits(currentX, currentY - 1) : -1;
        default: return -1;
    }
}

// Calculate the optimal number of turns needed to face the target direction
int calculateTurns(int currentDirection, int targetDirection) {
    int turns = (targetDirection - currentDirection + 4) % 4;
    return (turns > 2) ? (4 - turns) : turns;
}

// Increment visit count for the current cell
void incrementVisits(int32_t x, int32_t y) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visitMap[x][y]++;
    } else {
        ROS_ERROR("Invalid coordinates (%d, %d) in visitMap", x, y);
    }
}

// Get the visit count for a specific cell
int32_t getVisits(int32_t x, int32_t y) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visitMap[x][y];
    } else {
        ROS_ERROR("Invalid coordinates (%d, %d) in visitMap", x, y);
        return 0;
    }
}

// Update the turtle's position based on the direction
void updatePosition_turtle(int direction) {
    switch (direction) {
        case NORTH: currentX--; break;
        case EAST:  currentY++; break;
        case SOUTH: currentX++; break;
        case WEST:  currentY--; break;
    }
}

// Decide the turtle's next move based on the state and available directions
turtleMove studentTurtleStep(bool bumped, int nw_or) {
    static State currentState = STATE_MOVE_FORWARD;
    int minVisits = INT32_MAX;
    int targetDirection = nw_or;

    // Check all four directions to determine the least-visited one
    for (int i = 0; i < 4; ++i) {
        int visits = checkDirection(i);
        if (visits != -1 && visits < minVisits) {
            minVisits = visits;
            targetDirection = i;
        }
    }

    // Calculate how many turns are needed to face the target direction
    numTurns = calculateTurns(nw_or, targetDirection);

    // State transitions based on the number of turns and bump detection
    if (numTurns > 0) {
        currentState = STATE_TURN_LEFT;
    } else if (bumped) {
        currentState = STATE_TURN_LEFT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }

    // Execute the appropriate move
    switch (currentState) {
        case STATE_MOVE_FORWARD:
            updatePosition_turtle(targetDirection);
            incrementVisits(currentX, currentY);
            return MOVE_FORWARD;

        case STATE_TURN_LEFT:
            numTurns--;
            return TURN_LEFT;

        default:
            return MOVE_FORWARD;
    }
}
