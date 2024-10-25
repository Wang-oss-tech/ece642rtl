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


// Typedefs for readability and future flexibility
typedef int32_t State;       // Typedef for state representation
typedef bool Flag;           // Typedef for boolean flags

// Static array to keep track of visits to each cell
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // All cells initialized to zero

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

int checkDirection(int direction){
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
                case EAST:  return 3;   // One left turn needed
                case SOUTH: return 2;   // Two left turns needed
                case WEST:  return 1;   // Three left turns needed
            }
            break;

        case EAST:
            switch (targetDirection) {
                case NORTH: return 1;   // Three left turns needed
                case EAST:  return 0;   // Already aligned
                case SOUTH: return 3;   // One left turn needed
                case WEST:  return 2;   // Two left turns needed
            }
            break;

        case SOUTH:
            switch (targetDirection) {
                case NORTH: return 2;   // Two left turns needed
                case EAST:  return 1;   // Three left turns needed
                case SOUTH: return 0;   // Already aligned
                case WEST:  return 3;   // One left turn needed
            }
            break;

        case WEST:
            switch (targetDirection) {
                case NORTH: return 3;   // One left turn needed
                case EAST:  return 2;   // Two left turns needed
                case SOUTH: return 1;   // Three left turns needed
                case WEST:  return 0;   // Already aligned
            }
            break;

        default:
            ROS_ERROR("Invalid direction: %d", currentDirection);
            return 0;  // Default case to prevent undefined behavior
    }
    return 0;  // This should never be reached
}



/**
 * @brief Function decided the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
turtleMove studentTurtleStep(bool bumped, int nw_or) {
    static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    static int numTurns = 0;  // Tracks the number of required turns
    static int currentVisitIndex = 0;  // Tracks which direction to try next on a bump


    // Array to store visits for all four directions: [NORTH, EAST, SOUTH, WEST]
    std::pair<int, int> visitArray[4];  // Pair of (visit count, direction)

    // Populate the visit array with the number of visits for each direction
    ROS_INFO("\n\nChecking Direction for CurrentX: %d, CurrentY: %d",currentX, currentY);
    for (int i = 0; i < 4; i++) {
        int visits = checkDirection(i);  // Get the number of visits for each direction
        visitArray[i] = std::make_pair(visits, i);  // Store (visit count, direction) pairs
        ROS_INFO("Direction: %d, Visits: %d", i, visits);
    }

    // Sort the visit array in ascending order based on the visit count
    ROS_INFO("Sorting Array Based On Visit Count");
    std::sort(visitArray, visitArray + 4);  // Sort by visit count


    ROS_INFO("\n\nVisit Array (Sorted by Visit Count):");
    for (int i = 0; i < 4; i++) {
        ROS_INFO("Direction: %d, Visits: %d", visitArray[i].second, visitArray[i].first);
    }

    // Declare targetDirection and initialize var. w/ current orientation
    int targetDirection = nw_or;

    // State transition logic: If bumped, go to the next direction in the array
    if (bumped) {
        ROS_INFO("Bumped into a wall. Trying the next direction.");
        currentVisitIndex = (currentVisitIndex + 1) % 4;  // Cycle to the next direction
        targetDirection = visitArray[currentVisitIndex].second;
    } else {
        currentVisitIndex = 0;  // Reset index if not bumped
        targetDirection = visitArray[0].second;  // Pick the best (least visited) direction
    }

    ROS_INFO("\n\nTarget Direction Chosen: %d", targetDirection);

    // Calculate the number of turns required to align with the target direction
    numTurns = calculateTurns(nw_or, targetDirection);

    // State transition logic based on the number of turns
    if (numTurns > 0) {
        currentState = STATE_TURN_LEFT;  // Turn towards the target direction
    } else {
        currentState = STATE_MOVE_FORWARD;  // Move forward if aligned to target direction
    }

    // return turtleMove based on defined current state
    switch (currentState){
        case STATE_MOVE_FORWARD:
            updatePosition_turtle(nw_or);  // move forward
            incrementVisits(currentX, currentY); // update visit count
            ROS_INFO("Moving Forward\n-----------------------------------------")
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            numTurns --; // decrement turns counter
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD;
    }
}

// /**
//  * @brief Function decided the next move the turtle should make.
//  * `bumpedFlag` tells us whether the turtle hit a wall in front.
//  */
// turtleMove studentTurtleStep(bool bumped, int nw_or) {
//     static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    
//     // returns the move back to maze on what to do (depends on current state & whether it has bumped)
//     if (currentState == STATE_MOVE_FORWARD){
//         currentState = STATE_TURN_RIGHT;
//     } else if (bumped){
//         ROS_INFO("Bump occurred, turning left");
//         currentState = STATE_TURN_LEFT;
//     } else {
//         currentState = STATE_MOVE_FORWARD;
//     }

//     // return turtleMove based on defined current state
//     switch (currentState){
//         case STATE_MOVE_FORWARD:
//             updatePosition_turtle(nw_or); 
//             incrementVisits(currentX, currentY); 
//             return MOVE_FORWARD;
//         case STATE_TURN_LEFT:
//             return TURN_LEFT;
//         case STATE_TURN_RIGHT:
//             return TURN_RIGHT;
//         default:
//             return MOVE_FORWARD;
//     }
// }