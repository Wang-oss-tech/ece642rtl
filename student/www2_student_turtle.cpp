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
const int32_t MAZE_SIZE = 100;         // size of internal tracking array (23x23)
static int32_t relativeX = START_POS;
static int32_t relativeY = START_POS;
static int32_t currentX = START_POS;  // Current relative X position of the turtle
static int32_t currentY = START_POS;  // Current relative Y position of the turtle


// Typedefs of state and flag
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static array to keep track of visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero


/**
 *  @brief Retrieve visits at specific (x,y) coordinate
 * 
 * */ 
int32_t getVisits(int32_t x, int32_t y) {
    // Check if the x and y coordinates are within the valid range of the visitMap array
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visitMap[x][y];  // Return the visit count if coordinates are valid
    } else {
        ROS_ERROR("Invalid coordinates (%d, %d) accessed in visitMap.", x, y);
        return 0;  // Return 0 or an appropriate default value for out-of-bounds coordinates
    }
}


/**
 * @brief Function to increment the number of visits to a specific cell.
 */
void incrementVisits(int32_t x, int32_t y) {
    // Check if the x and y coordinates are within the valid range of the visitMap array
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visitMap[x][y]++;  // Increment the visit count if coordinates are valid
    } else {
        ROS_ERROR("Invalid coordinates (%d, %d) accessed in visitMap during increment.", x, y);
    }
}

/**
 * @brief Updates the current turtle's position based on its direction.
 * The turtle moves forward by 1 unit based on the current direction.
 */
void updatePosition_turtle(int nw_or) {
    switch (nw_or) {
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
 * @brief Determine if the turtle can move in a certain direction and return the number of visits to that square
 */
int checkDirection(int direction) {
    switch (direction) {
        case NORTH:
            return (currentX > 0) ? getVisits(currentX - 1, currentY) : -1;
        case EAST:
            return (currentY < MAZE_SIZE - 1) ? getVisits(currentX, currentY + 1) : -1;
        case SOUTH:
            return (currentX < MAZE_SIZE - 1) ? getVisits(currentX + 1, currentY) : -1;
        case WEST:
            return (currentY > 0) ? getVisits(currentX, currentY - 1) : -1;
        default:
            return -1;
    }
}

/**
 * @brief Calculating turns based on direction
 */
int calculateTurns(int currentDirection, int targetDirection) {
    // Calculate the difference in direction mod 4
    int turns = (targetDirection - currentDirection + 4) % 4;
    
    // If the number of turns is more than 2, it's faster to turn the other way
    return (turns > 2) ? (4 - turns) : turns;
}


turtleMove studentTurtleStep(bool bumped, int nw_or) {
    static State currentState = STATE_MOVE_FORWARD;
    static int numTurns = 0;

    ROS_INFO("studentTurtleStep called. \ncurrentState = %d", currentState);

    // Default to the current direction
    int targetDirection = nw_or;
    int minVisits = INT32_MAX;

    ROS_INFO("Target Direction: %d", targetDirection);

    // Check all four directions and find the optimal target direction
    for (int i = 0; i < 4; ++i) {
        int visits = checkDirection(i);
        if (visits != -1 && visits < minVisits) {
            minVisits = visits;
            targetDirection = i;
        }
    }

    // Calculate the number of turns required to face the target direction
    numTurns = calculateTurns(nw_or, targetDirection);

    // State transition logic based on the number of turns and whether the turtle bumped
    if (numTurns > 0) {
        currentState = STATE_TURN_LEFT;  // Turn towards the target direction
    } else if (bumped) {
        currentState = STATE_TURN_LEFT;  // If blocked, keep turning left
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
        case STATE_TURN_RIGHT:
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;
    }
}