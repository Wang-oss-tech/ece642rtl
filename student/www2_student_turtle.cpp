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
 * 
 * The turtle is responsible for its own relative understanding of the world,
 * while its absolute position and orientation are handled by the maze file.
 */

#include "student.h"
#include <stdint.h>  // Include stdint.h for fixed-width integer types

// Constants for various states and timeout values
const int32_t STATE_MOVE_FORWARD = 2;
const int32_t STATE_TURN_LEFT = 0;
const int32_t STATE_TURN_RIGHT = 1;

// Enum to represent directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Static variables to store the turtle's orientation and current state
static int32_t orientation = NORTH;  // Turtle's initial orientation
static int32_t currentState = STATE_MOVE_FORWARD;  // Turtle's initial state (moving forward)

/*
 * This function is called to decide the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
turtleMove studentTurtleStep(bool bumpedFlag) {
    if (bumpedFlag) {
        // If we bumped into a wall, we must turn left
        currentState = STATE_TURN_LEFT;
    } else if (rightIsClear()) {
        // If the right side is clear, turn right (right-hand rule)
        currentState = STATE_TURN_RIGHT;
    } else {
        // If no bump and right is blocked, move forward
        currentState = STATE_MOVE_FORWARD;
    }

    // Decide what to do based on the current state
    switch (currentState) {
        case STATE_MOVE_FORWARD:
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            // Update orientation to reflect the left turn
            orientation = (orientation + 3) % 4;  // Turn left (counter-clockwise)
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            // Update orientation to reflect the right turn
            orientation = (orientation + 1) % 4;  // Turn right (clockwise)
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;
    }
}

/*
 * Function to check if the right side is clear based on the turtle's relative orientation.
 * This should be implemented based on relative knowledge, not absolute coordinates.
 * We simulate the result using information passed to `student_maze.cpp`.
 */
bool rightIsClear() {
    // This function should call the equivalent of `rightIsClear()` in student_maze.cpp
    // to check whether there is a wall on the turtle's right.
    // Since `student_turtle.cpp` doesn't know absolute coordinates, the actual check
    // happens in `student_maze.cpp` via the simulation.
    return false;  // This should be implemented properly in `student_maze.cpp`
}
