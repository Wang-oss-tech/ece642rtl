#include "student.h"
#include <stdint.h>  // Include stdint.h for fixed-width integer types

// Define size of the maze array
const int32_t STATE_MOVE_FORWARD = 2;
const int32_t STATE_TURN_LEFT = 0;
const int32_t STATE_TURN_RIGHT = 1;
const int32_t MAZE_SIZE = 23;         // Size of internal tracking array (23x23)
const int32_t START_POS = 11;         // Starting position in center of 23x23 array

// Typedefs for readability and future flexibility
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static array to keep track of visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero

// Static variables to track turtle's current & previous positions
static int32_t currentX = START_POS;  // Current relative X position of the turtle
static int32_t currentY = START_POS;  // Current relative Y position of the turtle
static int32_t prevX = START_POS;     // Previous relative X position
static int32_t prevY = START_POS;     // Previous relative Y position

// Enum to represent direction/orientation (relative to starting point)
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Internal state for current direction
static Direction currentDirection = NORTH;  // Turtle starts facing NORTH

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
            currentY += 1;  // Move north (up in Y axis)
            break;
        case EAST:
            currentX += 1;  // Move east (right in X axis)
            break;
        case SOUTH:
            currentY -= 1;  // Move south (down in Y axis)
            break;
        case WEST:
            currentX -= 1;  // Move west (left in X axis)
            break;
    }
}

/**
 * @brief Updates the current direction based on the next move.
 * Turns the turtle left or right by adjusting the internal direction.
 */
void updateDirection(turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        // Turn left (counterclockwise)
        currentDirection = static_cast<Direction>((currentDirection + 3) % 4);  // Equivalent to subtracting 1 mod 4
    } else if (nextMove == TURN_RIGHT) {
        // Turn right (clockwise)
        currentDirection = static_cast<Direction>((currentDirection + 1) % 4);  // Equivalent to adding 1 mod 4
    }
}

/**
 * @brief Function to decide the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
turtleMove studentTurtleStep(bool bumped) {
    static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    
    ROS_INFO("Student turtle step called. Orig State: %d, Current Direction: %d", currentState, currentDirection);

    // If the turtle has moved to a new cell, increment the visit count
    if (currentX != prevX || currentY != prevY) {
        incrementVisits(currentX, currentY);  // Increment the visit count for the new cell
        prevX = currentX;   // Update previous position
        prevY = currentY;
    }

    // Determine the next move based on the current state and bump status
    if (bumped) {
        currentState = STATE_TURN_LEFT;  // If bumped, turn left
    } else if (currentState == STATE_TURN_RIGHT || currentState == STATE_TURN_LEFT) {
        currentState = STATE_MOVE_FORWARD;  // After a turn, move forward
    } else {
        currentState = STATE_TURN_RIGHT;  // Default to turn right
    }

    // Execute the next move based on the current state
    switch (currentState) {
        case STATE_MOVE_FORWARD:
            updatePosition();  // Move forward (adjust relative coordinates)
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            updateDirection(TURN_LEFT);  // Turn left (adjust internal direction)
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            updateDirection(TURN_RIGHT);  // Turn right (adjust internal direction)
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;  // Fallback to moving forward
    }
}
