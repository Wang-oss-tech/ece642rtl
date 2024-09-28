/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2    
 * LAST UPDATE: 9/8/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the right-hand rule.
 */

#include "student.h"
#include <stdint.h>  // Include stdint.h for fixed-width integer types

// Define size of the maze array

// Constants for various states and timeout values
const int32_t STATE_MOVE_FORWARD = 2;
const int32_t STATE_TURN_LEFT = 0;
const int32_t STATE_TURN_RIGHT = 1;
const int32_t MAZE_SIZE = 23;         // size of internal tracking array (23x23)
static int32_t relativeX = START_POS;
static int32_t relativeY = START_POS;
static Direction currentDirection = NORTH;  // Initial orientation of the turtle
static int32_t currentX = START_POS;  // Current relative X position of the turtle
static int32_t currentY = START_POS;  // Current relative Y position of the turtle
static int32_t prevX = START_POS;     // Previous relative X position
static int32_t prevY = START_POS;     // Previous relative Y position


// Typedefs for readability and future flexibility
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static array to keep track of visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero

/**
 * @brief Function to get the number of visits to a specific cell.
 */
int32_t getVisits(int32_t x, int32_t y) {
    return visitMap[x][y];
}

/**
 * @brief Function to increment the number of visits to a specific cell.
 */
void incrementVisits(int32_t x, int32_t y) {
    visitMap[x][y]++;
}

/**
 * @brief Updates the current turtle's position based on its direction.
 * The turtle moves forward by 1 unit based on the current direction.
 */
void updatePosition() {
    switch (currentDirection) {
        case NORTH:
            currentX -= 1;  // Move north (up in Y axis)
            break;
        case EAST:
            currentY -= 1;  // Move east (right in X axis)
            break;
        case SOUTH:
            currentX += 1;  // Move south (down in Y axis)
            break;
        case WEST:
            currentY += 1;  // Move west (left in X axis)
            break;
    }
}

/**
 * @brief Function decided the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
turtleMove studentTurtleStep(bool bumped, int nw_or) {
    static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    
    ROS_INFO("Student turtle step called Orig State: %d", currentState);
    
    if (currentX != prevX || currentY != prevY) {
        incrementVisits(currentX, currentY);  // Increment the visit count for the new cell
        prevX = currentX;   // Update previous position
        prevY = currentY;
    }
    // returns the move back to maze on what to do (depends on current state & whether it has bumped)
    if (currentState == STATE_MOVE_FORWARD){
        currentState = STATE_TURN_RIGHT;
    } else if (bumped){
        currentState = STATE_TURN_LEFT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }
    ROS_INFO("New Current State: %d", currentState);

    // return turtleMove based on defined current state
    switch (currentState){
        case STATE_MOVE_FORWARD:
            updatePosition();  
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;
    }
}