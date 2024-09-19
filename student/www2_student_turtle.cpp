/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2    
 * LAST UPDATE: 9/18/2024
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

// Define size of the maze array

// Constants for various states and timeout values
const int32_t TIMEOUT = 40;           // Timer value to slow down the simulation for better visibility
const int32_t TIMER_EXPIRED = 0;      // Timer expired value
const int32_t STATE_MOVE_FORWARD = 2;
const int32_t STATE_TURN_LEFT = 0;
const int32_t STATE_TURN_RIGHT = 1;
const int32_t MOVE_INCREMENT = 1;
const int32_t MOVE_DECREMENT = -1;
const int32_t TIME_DECREMENT = 1;
const int32_t MAZE_SIZE = 23;        // size of internal tracking array (23x23)
const int32_t START_POS = 11;        // starting position in center of 23x23 array

// Typedefs for readability and future flexibility
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static Array to keep track visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // all cells initialized to zero

// Enum to represent directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Struct to couple position (x, y)
typedef struct {
    int32_t x;
    int32_t y;

    // Method to update position based on orientation
    void update(int32_t orientation) {
        switch (orientation) {
            case NORTH:
                x += MOVE_DECREMENT;
                break;
            case EAST:
                y += MOVE_DECREMENT;
                break;
            case SOUTH:
                x += MOVE_INCREMENT;
                break;
            case WEST:
                y += MOVE_INCREMENT;
                break;
            default:
                ROS_ERROR("Invalid orientation for position update");
        }
    }

    // Method to set position directly
    void set(int32_t newX, int32_t newY) {
        x = newX;
        y = newY;
    }

} Position;

/**
 * @brief Function to get number of visits to specific cell
 */
int32_t getVisits(int32_t x, int32_t y) {
    return visitMap[x][y];
}

/**
 * @brief Function to increment the number of visits to a specific cell
 */
void incrementVisits(int32_t x, int32_t y) {
    visitMap[x][y]++;
}

/**
 * @brief Checks the turtle's direction and updates its orientation and state.
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
 * @brief Determines whether the turtle should move and updates its position accordingly.
 */
bool studentMoveTurtle(Position& position, int32_t& orientation) {
    static int32_t timer = TIMEOUT;        // Timer for managing movement
    static State currentState = STATE_TURN_LEFT; // Current state of the turtle's movement
    Flag shouldMove = false;            // Flag to determine if turtle should move
    Flag atEnd = false;                 // Flag to check if turtle has reached the end
    Flag modifyFlag = true;             // Flag to check if movement needs modification
    Flag bumpedFlag = false;            // Flag to check if turtle bumped into something

    ROS_INFO("Turtle update called - timer=%d", timer);

    if (timer == TIMER_EXPIRED) {
        Position futurePos1 = position;
        Position futurePos2 = position;

        futurePos2.update(orientation); // Update future position based on orientation

        bumpedFlag = bumped(futurePos1.x, futurePos1.y, futurePos2.x, futurePos2.y);
        atEnd = atend(position.x, position.y);

        checkDirection(orientation, bumpedFlag, currentState);

        shouldMove = (currentState == STATE_MOVE_FORWARD);
        modifyFlag = true;

        if (shouldMove && !atEnd) {
            position.update(orientation); // Update the current position

            // update visit count in internal map
            incrementVisits(position.x + START_POS, position.y + START_POS);

            // Call displayVisits to visualize the visit count
            displayVisits(getVisits(position.x + START_POS, position.y + START_POS));
            
            shouldMove = false;
            modifyFlag = true;
        }
    }

    if (atEnd) {
        return false;
    }

    timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - TIME_DECREMENT;
    return (timer == TIMEOUT);
}
