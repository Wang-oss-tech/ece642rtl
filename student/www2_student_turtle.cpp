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
const int32_t START_POS = 11;         // starting position in center of 23x23 array


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
 * @brief Function decided the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
// turtleMove studentTurtleStep(bool bumped) {
//     static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    
//     ROS_INFO("Student turtle step called Orig State: %d", currentState);
//     // returns the move back to maze on what to do (depends on current state & whether it has bumped)
//     if (currentState == STATE_MOVE_FORWARD){
//         currentState = STATE_TURN_RIGHT;
//     } else if (bumped){
//         currentState = STATE_TURN_LEFT;
//     } else {
//         currentState = STATE_MOVE_FORWARD;
//     }
//     ROS_INFO("New Current State: %d", currentState);

//     // return turtleMove based on defined current state
//     switch (currentState){
//         case STATE_MOVE_FORWARD:
//             return MOVE_FORWARD;
//         case STATE_TURN_LEFT:
//             return TURN_LEFT;
//         case STATE_TURN_RIGHT:
//             return TURN_RIGHT;
//         default:
//             return MOVE_FORWARD;
//     }
// }
turtleMove studentTurtleStep(bool bumped) {
    static State currentState = STATE_TURN_LEFT;  // Initial state: turn left
    static Flag movePending = false;             // Flag to track if a move is pending after a turn

    ROS_INFO("Student turtle step called. Current State: %d", currentState);
    ROS_INFO("Move pending: %d", movePending);

    // If the turtle bumps into something, it should turn left
    if (bumped) {
        currentState = STATE_TURN_LEFT;
        movePending = false;  // Reset the pending move since we're turning after a bump
    }
    // If a move is pending after a turn, execute the move
    else if (movePending) {
        currentState = STATE_MOVE_FORWARD;
        movePending = false;  // Reset the pending flag after the move
    }
    // If not bumped and the current state is turning, move forward next tick
    else if (currentState == STATE_TURN_RIGHT || currentState == STATE_TURN_LEFT) {
        movePending = true;   // Set the move to happen in the next tick
    }
    // After moving forward, turn right to follow the right-hand rule
    else if (currentState == STATE_MOVE_FORWARD) {
        currentState = STATE_TURN_RIGHT;
        movePending = false;  // Ensure no move is pending after a turn
    }

    ROS_INFO("New Current State: %d", currentState);

    // Return the turtle's next move based on its state
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

