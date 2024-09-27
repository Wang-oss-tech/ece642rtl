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

// Enum to represent directions / orientation
// enum Direction {
//     NORTH = 0,
//     EAST = 1,
//     SOUTH = 2,
//     WEST = 3
// };

// Define Struct for Position
// typedef struct {
//     int32_t X;
//     int32_t Y;
// } Position;

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

// /**
//  * @brief Checks the turtle's direction and updates its orientation and state.
//  */
// void checkDirection(int32_t& orientation, Flag bumpedFlag, State& currentState) {
//     switch (orientation) {
//         case NORTH:
//             if (currentState == STATE_MOVE_FORWARD) {
//                 orientation = EAST;  // Turn right to face East
//                 currentState = STATE_TURN_RIGHT;
//             } else if (bumpedFlag) {
//                 orientation = WEST;  // Turn left to face West if bumped
//                 currentState = STATE_TURN_LEFT;
//             } else {
//                 currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
//             }
//             break;
//         case EAST:
//             if (currentState == STATE_MOVE_FORWARD) {
//                 orientation = SOUTH; // Turn right to face South
//                 currentState = STATE_TURN_RIGHT;
//             } else if (bumpedFlag) {
//                 orientation = NORTH; // Turn left to face North if bumped
//                 currentState = STATE_TURN_LEFT;
//             } else {
//                 currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
//             }
//             break;
//         case SOUTH:
//             if (currentState == STATE_MOVE_FORWARD) {
//                 orientation = WEST;  // Turn right to face West
//                 currentState = STATE_TURN_RIGHT;
//             } else if (bumpedFlag) {
//                 orientation = EAST;  // Turn left to face East if bumped
//                 currentState = STATE_TURN_LEFT;
//             } else {
//                 currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
//             }
//             break;
//         case WEST:
//             if (currentState == STATE_MOVE_FORWARD) {
//                 orientation = NORTH; // Turn right to face North
//                 currentState = STATE_TURN_RIGHT;
//             } else if (bumpedFlag) {
//                 orientation = SOUTH; // Turn left to face South if bumped
//                 currentState = STATE_TURN_LEFT;
//             } else {
//                 currentState = STATE_MOVE_FORWARD;  // Move forward if no bump
//             }
//             break;
//         default:
//             ROS_ERROR("Invalid orientation value: %d", orientation);
//             break;
//     }
// }

// /**
//  * @brief Updates the position of the turtle based on its current orientation.
//  */
// void updatePosition(QPointF& position, int32_t orientation) {
//     switch (orientation) {
//         case EAST:
//             position.setY(position.y() + MOVE_DECREMENT); // Move East (right)
//             break;
//         case SOUTH:
//             position.setX(position.x() + MOVE_INCREMENT); // Move South (down)
//             break;
//         case WEST:
//             position.setY(position.y() + MOVE_INCREMENT); // Move West (left)
//             break;
//         case NORTH:
//             position.setX(position.x() + MOVE_DECREMENT); // Move North (up)
//             break;
//         default:
//             ROS_ERROR("Invalid orientation value: %d", orientation);
//             break;
//     }
// }


/**
 * @brief Function decided the next move the turtle should make.
 * `bumpedFlag` tells us whether the turtle hit a wall in front.
 */
turtleMove studentTurtleStep(bool bumped) {
    static State currentState = STATE_TURN_LEFT; // Current state of the turtle's movement
    

    // returns the move back to maze on what to do (depends on current state & whether it has bumped)
    if (currentState = STATE_MOVE_FORWARD){
        currentState = STATE_TURN_RIGHT;
    } else if (bumped){
        currentState = STATE_TURN_LEFT;
    } else {
        currentState = STATE_MOVE_FORWARD;
    }

    // return turtleMove based on defined current state
    switch (currentState){
        case STATE_MOVE_FORWARD:
            return MOVE_FORWARD;
        case STATE_TURN_LEFT:
            return TURN_LEFT;
        case STATE_TURN_RIGHT:
            return TURN_RIGHT;
        default:
            return MOVE_FORWARD
    }
}


// /**
//  * @brief Determines whether the turtle should move and updates its position accordingly.
//  */
// bool studentMoveTurtle(QPointF& position, int32_t& orientation) {
//     static int32_t timer = TIMEOUT;        // Timer for managing movement
//     static State currentState = STATE_TURN_LEFT; // Current state of the turtle's movement
//     Position futureX1, futureY1, futureX2, futureY2; // Future positions based on orientation
//     Flag shouldMove = false;            // Flag to determine if turtle should move
//     Flag atEnd = false;                 // Flag to check if turtle has reached the end
//     Flag modifyFlag = true;             // Flag to check if movement needs modification
//     Flag bumpedFlag = false;            // Flag to check if turtle bumped into something

//     ROS_INFO("Turtle update called - timer=%d", timer);

//     if (timer == TIMER_EXPIRED) {
//         futureX1.X = position.x();
//         futureY1.Y = position.y();
//         futureX2.X = position.x();
//         futureY2.Y = position.y();

//         switch (orientation) {
//             case NORTH:
//                 futureY2.Y += MOVE_INCREMENT; // Moving North increases Y
//                 break;
//             case EAST:
//                 futureX2.X += MOVE_INCREMENT; // Moving East increases X
//                 break;
//             case SOUTH:
//                 futureX2.X += MOVE_INCREMENT;
//                 futureY2.Y += MOVE_INCREMENT; // Moving South increases both X and Y (diagonal)
//                 futureX1.X += MOVE_INCREMENT;
//                 break;
//             case WEST:
//                 futureX2.X += MOVE_INCREMENT;
//                 futureY2.Y += MOVE_INCREMENT; // Moving West increases both X and Y (diagonal)
//                 futureY1.Y += MOVE_INCREMENT;
//                 break;
//             default:
//                 ROS_ERROR("Invalid orientation value: %d", orientation);
//                 break;
//         }

//         bumpedFlag = bumped(futureX1.X, futureY1.Y, futureX2.X, futureY2.Y);
//         atEnd = atend(position.x(), position.y());

//         checkDirection(orientation, bumpedFlag, currentState);

//         shouldMove = (currentState == STATE_MOVE_FORWARD);
//         modifyFlag = true;

//         if (shouldMove && !atEnd) {
//             updatePosition(position, orientation);

//             // Update the visit count in the internal map
//             incrementVisits(static_cast<int32_t>(position.x() + START_POS), static_cast<int32_t>(position.y() + START_POS));

//             // Call displayVisits to visualize the visit count
//             displayVisits(getVisits(static_cast<int32_t>(position.x() + START_POS), static_cast<int32_t>(position.y() + START_POS)));

//             shouldMove = false;
//             modifyFlag = true;
//         }
//     }

//     if (atEnd) {
//         return false;
//     }

//     timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - TIME_DECREMENT;
//     return (timer == TIMEOUT);
// }