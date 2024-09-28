#include "student.h"
#include <stdint.h>  // Include stdint.h for fixed-width integer types

// Define size of the maze array
const int32_t MAZE_SIZE = 23;         // size of internal tracking array (23x23)
const int32_t START_POS = 11;         // starting position in center of 23x23 array

// Typedefs for readability and future flexibility
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static array to keep track of visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero

// Variables to track the turtle's current and previous positions
static int32_t currentX = START_POS;  // Current relative X position of the turtle
static int32_t currentY = START_POS;  // Current relative Y position of the turtle
static int32_t prevX = START_POS;     // Previous relative X position
static int32_t prevY = START_POS;     // Previous relative Y position

/**
 * @brief Function to get the number of visits to the current cell.
 */
int32_t getVisitCount() {
    return visitMap[currentX][currentY];
}

/**
 * @brief Function to increment the number of visits to the current cell.
 */
void incrementVisits() {
    visitMap[currentX][currentY]++;
}

/**
 * @brief Function to handle the turtle's movement and track visits.
 * @param bumped - boolean flag indicating if the turtle hit a wall
 * @return turtleMove - the next move the turtle should make
 */
turtleMove studentTurtleStep(bool bumped) {
    static State currentState = STATE_MOVE_FORWARD;  // Current state of the turtle's movement
    
    ROS_INFO("Student turtle step called. Current State: %d", currentState);

    // If the turtle has moved to a new cell, increment the visit count
    if (currentX != prevX || currentY != prevY) {
        incrementVisits();  // Increment the visit count for the new cell
        prevX = currentX;   // Update previous position
        prevY = currentY;
    }

    // Determine the next move
    if (bumped) {
        currentState = STATE_TURN_LEFT;
    } else if (currentState == STATE_MOVE_FORWARD) {
        currentState = STATE_TURN_RIGHT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }

    // Return the turtle's next move
    switch (currentState) {
        case STATE_MOVE_FORWARD:
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;
    }
}
