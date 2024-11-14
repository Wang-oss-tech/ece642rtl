/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2    
 * LAST UPDATE: 9/8/2024
 *
 * This file is an algorithm to solve the ece642rtle maze j
 * using the right-hand rule.
 */

#ifdef testing
#include "student_mock.h"
#else
#include "student.h"
#include "ros/ros.h"
#endif


#include <algorithm> // For std::sort
#include <cstdio> // For fprintf with ROS_ERROR macro in testing mode

#ifdef testing
#define ROS_ERROR(...) fprintf(stderr, __VA_ARGS__)
#endif


#include <stdint.h>  // Include stdint.h for fixed-width integer types
#include <utility>  // For std::pair


// Define size of the maze array

// Constants for various states and timeout values
const int32_t MAZE_SIZE = 100;                      // size of internal tracking array (23x23)
static int32_t currentX = START_POS;                // Current relative X position of the turtle
static int32_t currentY = START_POS;                // Current relative Y position of the turtle

enum STATE {
    STATE_MOVE_FORWARD,
    STATE_TURN_LEFT,
};


// Typedefs for readability and future flexibility
typedef STATE State;   // Typedef for state representation
typedef bool Flag;     // Typedef for boolean flags

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
        default: break;
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
            nextY -= 1;
            break;
        case SOUTH:
            nextX += 1;
            break;
        case WEST:
            nextY += 1;
            break;
        default:
            return -1;  // Invalid direction
    }

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
                default: break;
            }
            break;

        case EAST:
            switch (targetDirection) {
                case NORTH: return 1;   // Three left turns needed
                case EAST:  return 0;   // Already aligned
                case SOUTH: return 3;   // One left turn needed
                case WEST:  return 2;   // Two left turns needed
                default: break;
            }
            break;

        case SOUTH:
            switch (targetDirection) {
                case NORTH: return 2;   // Two left turns needed
                case EAST:  return 1;   // Three left turns needed
                case SOUTH: return 0;   // Already aligned
                case WEST:  return 3;   // One left turn needed
                default: break;
            }
            break;

        case WEST:
            switch (targetDirection) {
                case NORTH: return 3;   // One left turn needed
                case EAST:  return 2;   // Two left turns needed
                case SOUTH: return 1;   // Three left turns needed
                case WEST:  return 0;   // Already aligned
                default: break;
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
std::pair<turtleMove, int> studentTurtleStep(bool bumped, int nw_or) {
    printf("\n\nstudentTurtleStep called");
    static State currentState = STATE_MOVE_FORWARD; // Current state of the turtle's movement
    static int numTurns = 0;                        // Tracks the number of required turns
    static currentVisitIndex = 0;               // Tracks which direction to try next on a bump

    // Array to store visits for all four directions: [NORTH, EAST, SOUTH, WEST]
    std::pair<int, int> visitArray[4];  // Pair of (visit count, direction)

    // Populate the visit array with the number of visits for each direction
    for (int i = 0; i < 4; i++) {
        int visits = checkDirection(i);             // Get the number of visits for each direction
        visitArray[i] = std::make_pair(visits, i);  // Store (visit count, direction) pairs
    }

    // Sort the visit array in ascending order based on the visit count
    std::sort(visitArray, visitArray + 4);  // Sort by visit count


    for (int i = 0; i < 4; i ++){
        printf("\nSorted Direction[%d]: %d, Visit count: %d", i, visitArray[i].second, 
                                                                 visitArray[i].first);
    }

    printf("\nCurrent Visit Index: %d", currentVisitIndex);

    // Declare targetDirection and initialize var. w/ current orientation
    int targetDirection = nw_or;
    targetDirection = visitArray[currentVisitIndex].second;

    printf("\nTarget Direction calculated: %d", targetDirection);

    // Calculate the number of turns required to align with the target direction
    numTurns = calculateTurns(nw_or, targetDirection);

    printf("\nNumber of Turns calculated: %d", numTurns);

    // State transition logic based on the number of turns
    if (numTurns > 0) {
        currentState = STATE_TURN_LEFT;  // Turn towards the target direction
    } else {
        currentState = STATE_MOVE_FORWARD;  // Move forward if aligned to target direction
    }

    // If current state is bumped
    if (currentState == STATE_MOVE_FORWARD && bumped){
        currentVisitIndex = (currentVisitIndex + 1) % 4;        // Cycle to the next direction
        printf("\nBumped, New Current Visit Index: %d", currentVisitIndex);
        targetDirection = visitArray[currentVisitIndex].second;
        numTurns = calculateTurns(nw_or, targetDirection);
        if (numTurns > 0){
            currentState = STATE_TURN_LEFT;
        }
    } 

    // return turtleMove based on defined current state
    switch (currentState){
        // Tells turtle to move forard
        case STATE_MOVE_FORWARD:
            updatePosition_turtle(targetDirection);     // move forward
            incrementVisits(currentX, currentY);        // update visit count
            currentVisitIndex = 0;
            return std::make_pair(MOVE_FORWARD, 0);
        case STATE_TURN_LEFT:
            return std::make_pair(TURN_LEFT, numTurns);
        default:
            return std::make_pair(MOVE_FORWARD, 0);
    } 
}

