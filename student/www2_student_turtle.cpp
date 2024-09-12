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

// Ignoring this line until project 5
turtleMove studentTurtleStep(bool bumped) {
    return MOVE;
}

// Constants for various states and timeout values
const int32_t TIMEOUT = 40;            // Timer value to slow down the simulation for better visibility
const int32_t TIMER_EXPIRED = 0;       // Timer value indicating that the countdown has completed
const int32_t STATE_MOVE_FORWARD = 2;
const int32_t STATE_TURN_LEFT = 0;
const int32_t STATE_TURN_RIGHT = 1;

// Typedefs for readability and future flexibility
typedef int32_t State;    // Typedef for state representation
typedef bool Flag;        // Typedef for boolean flags

// Struct to represent a geometric pair (X, Y)
typedef struct {
    int32_t x;
    int32_t y;
} Position;

// Constants for movement increments
const int32_t MOVE_INCREMENT = 1;
const int32_t MOVE_DECREMENT = -1;
const int32_t NO_MOVE = 0;

// Enum to represent directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

/**
 * @brief Checks the turtle's direction and updates its orientation and state.
 * 
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * @param bumpedFlag  Indicates if the turtle has bumped into an obstacle.
 * @param currentState The current state of the turtle's movement.
 * 
 * @details
 * Purpose: This function determines the turtle's next move based on its current orientation and whether it has bumped into an obstacle. 
 *          It updates the orientation and state of the turtle to ensure it follows the right-hand rule in navigating the maze.
 * Inputs:  - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 *          - `bumpedFlag`: A boolean indicating if the turtle has encountered an obstacle.
 *          - `currentState`: An integer representing the turtle's current state.
 * Outputs: - Updates the `orientation` variable to reflect the turtle's new direction.
 *          - Updates the `currentState` to control the turtle's movement logic.
 * Saved Internal: Updates the `currentState` to manage state transitions.
 */
void checkDirection(int32_t& orientation, Flag bumpedFlag, State& currentState) {
    switch (orientation) {
        case NORTH:
            if (currentState == STATE_MOVE_FORWARD) {
                orientation = EAST;  // Turn right to face East
                currentState = STATE_TURN_RIGHT;
            } else if (bumpedFlag) {
                orientation = WEST;  // Turn left to face West if bumped
                currentState = STATE_TURN_LEFT;
            } else {
                currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
            }
            break;
        case EAST:
            if (currentState == STATE_MOVE_FORWARD) {
                orientation = SOUTH; // Turn right to face South
                currentState = STATE_TURN_RIGHT;
            } else if (bumpedFlag) {
                orientation = NORTH; // Turn left to face North if bumped
                currentState = STATE_TURN_LEFT;
            } else {
                currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
            }
            break;
        case SOUTH:
            if (currentState == STATE_MOVE_FORWARD) {
                orientation = WEST;  // Turn right to face West
                currentState = STATE_TURN_RIGHT;
            } else if (bumpedFlag) {
                orientation = EAST;  // Turn left to face East if bumped
                currentState = STATE_TURN_LEFT;
            } else {
                currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
            }
            break;
        case WEST:
            if (currentState == STATE_MOVE_FORWARD) {
                orientation = NORTH; // Turn right to face North
                currentState = STATE_TURN_RIGHT;
            } else if (bumpedFlag) {
                orientation = SOUTH; // Turn left to face South if bumped
                currentState = STATE_TURN_LEFT;
            } else {
                currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
            }
            break;
        default:
            ROS_ERROR("Invalid orientation value: %d", orientation);
            break;
    }
}

/**
 * @brief Updates the position of the turtle based on its current orientation.
 * 
 * @param position    Current position of the turtle as a Position struct.
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * 
 * @details
 * Purpose: This function updates the turtle's position on the grid based on its current orientation. The position is adjusted to simulate movement in the specified direction.
 * Inputs:  - `position`: A Position struct representing the current position of the turtle on the grid.
 *          - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 * Outputs: - Modifies the `position` variable to reflect the turtle's new location on the grid.
 * Saved Internal: None.
 */
void updatePosition(Position& position, int32_t orientation) {
    switch (orientation) {
        case EAST:
            position.y += MOVE_DECREMENT; // Move East (right)
            break;
        case SOUTH:
            position.x += MOVE_INCREMENT; // Move South (down)
            break;
        case WEST:
            position.y += MOVE_INCREMENT; // Move West (left)
            break;
        case NORTH:
            position.x += MOVE_DECREMENT; // Move North (up)
            break;
        default:
            ROS_ERROR("Invalid orientation value: %d", orientation);
            break;
    }
}

/**
 * @brief Determines whether the turtle should move and updates its position accordingly.
 * 
 * @param position    Current position of the turtle as a Position struct.
 * @param orientation Current orientation of the turtle (NORTH, EAST, SOUTH, WEST).
 * @return true if changes should be submitted, false otherwise.
 * 
 * @details
 * Purpose: This function controls the turtle's movement within the maze, determining if it should move based on the current timer, its orientation, and whether it has reached the end of the maze.
 *          It manages the movement logic by updating the position and orientation of the turtle while considering potential obstacles.
 * Inputs:  - `position`: A Position struct representing the current position of the turtle on the grid.
 *          - `orientation`: An integer representing the turtle's current orientation (NORTH, EAST, SOUTH, WEST).
 * Outputs: - Returns `true` if the turtle's movement should be submitted (timer has reset), or `false` if it should not.
 * Saved Internal: - Updates the local variables `timer`, `currentState`, `shouldMove`, `atEnd`, `modifyFlag`, and `bumpedFlag` to manage the turtle's state and logic flow.
 */
bool studentMoveTurtle(Position& position, int32_t& orientation) {
    // Define all variables at the start of the procedure
    static int32_t timer = TIMEOUT;        // Timer for managing movement
    static State currentState = STATE_TURN_LEFT; // Current state of the turtle's movement
    Position futurePos1 = position, futurePos2 = position; // Future positions based on orientation
    Flag shouldMove = false;            // Flag to determine if turtle should move
    Flag atEnd = false;                 // Flag to check if turtle has reached the end
    Flag modifyFlag = true;             // Flag to check if movement needs modification
    Flag bumpedFlag = false;            // Flag to check if turtle bumped into something

    ROS_INFO("Turtle update called - timer=%d", timer);

    // Timer countdown logic
    if (timer == TIMER_EXPIRED) { // Timer has completed its countdown, execute logic
        // Determine the future position based on the current orientation
        switch (orientation) {
            case NORTH:
                futurePos2.y += MOVE_INCREMENT; // Moving North increases Y
                break;
            case EAST:
                futurePos2.x += MOVE_INCREMENT; // Moving East increases X
                break;
            case SOUTH:
                futurePos2.x += MOVE_INCREMENT; // Moving South increases X
                futurePos2.y += MOVE_INCREMENT; // Moving South increases Y (diagonal)
                futurePos1.x += MOVE_INCREMENT;
                break;
            case WEST:
                futurePos2.x += MOVE_INCREMENT; // Moving West increases X
                futurePos2.y += MOVE_INCREMENT; // Moving West increases Y (diagonal)
                futurePos1.y += MOVE_INCREMENT;
                break;
            default:
                ROS_ERROR("Invalid orientation value: %d", orientation);
                break;
        }

        // Check if the turtle is about to bump into a wall or has reached the end
        bumpedFlag = bumped(futurePos1.x, futurePos1.y, futurePos2.x, futurePos2.y);
        atEnd = atend(position.x, position.y);

        // Check direction and update orientation
        checkDirection(orientation, bumpedFlag, currentState);

        ROS_INFO("Orientation=%d  STATE=%d", orientation, currentState);

        shouldMove = (currentState == STATE_MOVE_FORWARD);
        modifyFlag = true;

        // Move the turtle if allowed and not at the end
        if (shouldMove && !atEnd) {
            updatePosition(position, orientation);
            shouldMove = false;
            modifyFlag = true;
        }
    }

    // Check if the turtle has reached the end of the maze
    if (atEnd) {
        return false;
    }

    // Update the timer
    timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - 1;

    // Submit changes if the timer has reset
    return (timer == TIMEOUT);
}
