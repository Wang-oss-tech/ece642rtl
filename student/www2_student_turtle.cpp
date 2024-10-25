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

// Constants for various states and timeout values
const int32_t MAZE_SIZE = 100;         // size of internal tracking array (23x23)
static int32_t currentX = START_POS;  // Current relative X position of the turtle
static int32_t currentY = START_POS;  // Current relative Y position of the turtle

// Enum to represent direction/orientation
enum STATE {
  STATE_START,
  STATE_TURN_LEFT,
  STATE_MOVE_FORWARD
};


// Typedefs of state and flag
typedef int State;       // Typedef for state representation
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
        ROS_INFO("X: %d, Y: %d", x, y);
        ROS_INFO("visit count at location: %d", visitMap[x][y]);
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
    int nextX = currentX;
    int nextY = currentY;

    // Calculate the next position based on the direction
    switch (direction) {
        case NORTH:
            nextX -= 1;
            break;
        case EAST:
            nextY += 1;
            break;
        case SOUTH:
            nextX += 1;
            break;
        case WEST:
            nextY -= 1;
            break;
        default:
            return -1;  // Invalid direction
    }

    // Check if the move is within the bounds of the maze
    if (nextX < 0 || nextX >= MAZE_SIZE || nextY < 0 || nextY >= MAZE_SIZE) {
        return -1;  // Out of bounds, not a valid move
    }

    // Check if the next position is blocked by a wall (bumped)
    bool isBlocked = bumped(currentX, currentY, nextX, nextY);
    if (isBlocked) {
        ROS_INFO("Direction %d is blocked by a wall.", direction);
        return -1;  // Return -1 to indicate this direction is not valid
    }

    ROS_INFO("CurrentX: %d, CurrentY: %d", currentX, currentY);
    ROS_INFO("NextX: %d, NextY: %d", nextX, nextY);

    // Return the number of visits for this square if valid
    return getVisits(nextX, nextY);
}

/**
 * @brief Calculating turns based on direction
 */
int calculateTurns(int currentDirection, int targetDirection) {
    switch (currentDirection) {
        case NORTH:
            switch (targetDirection) {
                case NORTH: return 0;   // Already facing the right direction
                case EAST:  return 1;   // One left turn needed
                case SOUTH: return 2;   // Two left turns needed
                case WEST:  return 3;   // Three left turns needed
            }
            break;

        case EAST:
            switch (targetDirection) {
                case NORTH: return 3;   // Three left turns needed
                case EAST:  return 0;   // Already aligned
                case SOUTH: return 1;   // One left turn needed
                case WEST:  return 2;   // Two left turns needed
            }
            break;

        case SOUTH:
            switch (targetDirection) {
                case NORTH: return 2;   // Two left turns needed
                case EAST:  return 3;   // Three left turns needed
                case SOUTH: return 0;   // Already aligned
                case WEST:  return 1;   // One left turn needed
            }
            break;

        case WEST:
            switch (targetDirection) {
                case NORTH: return 1;   // One left turn needed
                case EAST:  return 2;   // Two left turns needed
                case SOUTH: return 3;   // Three left turns needed
                case WEST:  return 0;   // Already aligned
            }
            break;

        default:
            ROS_ERROR("Invalid direction: %d", currentDirection);
            return 0;  // Default case to prevent undefined behavior
    }
    return 0;  // This should never be reached
}

turtleMove studentTurtleStep(bool bumped, int nw_or){
    // Initialize state
    static State currentState = STATE_START;
    static int numTurns = 0;

    int minVisits = INT32_MAX;
    int targetDirection = nw_or;

    // Evaluate direction for all 4 cells
    for (int i = 0; i < 4; ++i) {
        int visits = checkDirection(i);  // Check each direction
        if (visits != -1 && visits < minVisits) {  // If valid and fewer visits
            minVisits = visits;
            targetDirection = i;
        }
    }

    numTurns = calculateTurns(nw_or, targetDirection);

    // Keep turning left based on numTurns
    if (numTurns > 0){
        currentState = STATE_TURN_LEFT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }

    // Execute the appropriate action based on the current state
    switch (currentState) {
        case STATE_MOVE_FORWARD:
            ROS_INFO("Moving forward to direction: %d", targetDirection);
            updatePosition_turtle(targetDirection);  // Move forward
            incrementVisits(currentX, currentY);     // Update visit count
            return MOVE_FORWARD;

        case STATE_TURN_LEFT:
            ROS_INFO("Turning left. Remaining turns: %d", numTurns);
            numTurns--;  // Decrement the turn counter
            if (numTurns <= 0) {
                currentState = STATE_MOVE_FORWARD;  // Transition to moving forward after turning
            }
            return TURN_LEFT;

        default:
            ROS_ERROR("Invalid state: %d", currentState);
            return MOVE_FORWARD;
    }
}























// /**
//  * @brief Turtle's next action based on its orientation and bump status.
//  */
// turtleMove studentTurtleStep(bool bumped, int nw_or) {
//     static State currentState = STATE_MOVE_FORWARD;
//     static int numTurns = 0;

//     ROS_INFO("Current State: %d", currentState);

//     // Default to the current direction
//     int targetDirection = nw_or;
//     int minVisits = INT32_MAX;

//     // Check all four directions and find the optimal target direction
//     for (int i = 0; i < 4; ++i) {
//         int visits = checkDirection(i);
//         ROS_INFO("Checking direction: %d, Visits: %d", i, visits);

//         // Choose the direction with the fewest visits and ensure it's valid
//         if (visits != -1 && visits < minVisits) {
//             minVisits = visits;
//             targetDirection = i;
//         }
//     }

//     // Calculate the number of turns required to align with the target direction
//     numTurns = calculateTurns(nw_or, targetDirection);

//     // State transition logic
//     if (bumped) {
//         ROS_INFO("Bumped into a wall. Turning left.");
//         currentState = STATE_TURN_LEFT;  // Keep turning left if bumped
//     } else if (numTurns > 0) {
//         currentState = STATE_TURN_LEFT;  // Turn towards the target direction
//     } else {
//         currentState = STATE_MOVE_FORWARD;  // Move forward if aligned
//     }

//     // Execute the appropriate action based on the current state
//     switch (currentState) {
//         case STATE_MOVE_FORWARD:
//             ROS_INFO("Moving forward to direction: %d", targetDirection);
//             updatePosition_turtle(targetDirection);  // Move forward
//             incrementVisits(currentX, currentY);     // Update visit count
//             return MOVE_FORWARD;

//         case STATE_TURN_LEFT:
//             ROS_INFO("Turning left. Remaining turns: %d", numTurns);
//             numTurns--;  // Decrement the turn counter
//             if (numTurns <= 0) {
//                 currentState = STATE_MOVE_FORWARD;  // Transition to moving forward after turning
//             }
//             return TURN_LEFT;

//         case STATE_TURN_RIGHT:
//             ROS_INFO("Turning right.");
//             return TURN_RIGHT;

//         default:
//             ROS_ERROR("Invalid state: %d", currentState);
//             return MOVE_FORWARD;
//     }
// }
