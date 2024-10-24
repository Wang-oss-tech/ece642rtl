#include "student.h"
#include <stdint.h>  

const int32_t MAZE_SIZE = 100; // Larger grid size
static int32_t currentX = START_POS;
static int32_t currentY = START_POS;
static int32_t numTurns = 0;  // Track the number of turns

static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero

// Check directions and return visit counts or -1 if blocked
int checkDirection(int direction) {
    switch (direction) {
        case NORTH: return (currentX > 0) ? getVisits(currentX - 1, currentY) : -1;
        case EAST:  return (currentY < MAZE_SIZE - 1) ? getVisits(currentX, currentY + 1) : -1;
        case SOUTH: return (currentX < MAZE_SIZE - 1) ? getVisits(currentX + 1, currentY) : -1;
        case WEST:  return (currentY > 0) ? getVisits(currentX, currentY - 1) : -1;
        default: return -1;
    }
}

// Calculate optimal number of turns needed to face the target direction
int calculateTurns(int currentDirection, int targetDirection) {
    int turns = (targetDirection - currentDirection + 4) % 4;
    return (turns > 2) ? (4 - turns) : turns;
}

// Increment the visit count for the current position
void incrementVisits(int32_t x, int32_t y) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visitMap[x][y]++;
    }
}

// Main decision-making function for the turtle
turtleMove studentTurtleStep(bool bumped, int nw_or) {
    static State currentState = STATE_MOVE_FORWARD;
    int minVisits = INT32_MAX;
    int targetDirection = nw_or;

    // Check all four directions and choose the one with the fewest visits
    for (int i = 0; i < 4; ++i) {
        int visits = checkDirection(i);
        if (visits != -1 && visits < minVisits) {
            minVisits = visits;
            targetDirection = i;
        }
    }

    // Calculate turns needed to face the target direction
    numTurns = calculateTurns(nw_or, targetDirection);

    // State transitions based on turns and bump detection
    if (numTurns > 0) {
        currentState = STATE_TURN_LEFT;
    } else if (bumped) {
        currentState = STATE_TURN_LEFT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }

    // Execute appropriate move
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
