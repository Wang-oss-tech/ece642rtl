/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2  
 * LAST UPDATE: 9/8/2024
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze-solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze-solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 */

#include "student.h"
#include <stdint.h>  // Include stdint.h for fixed-width integer types

// Static array to keep track of visits to each cell
static int32_t visitMap[23][23] = {0};  // All cells initialized to zero

// Enum to represent directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

/*
 * This procedure takes the current turtle position and orientation 
 * and returns true=accept changes, false=do not accept changes
 */
bool moveTurtle(QPointF& pos_, int& nw_or) {
    // Check if there's a wall directly in front of the turtle
    bool bumpedFlag = bumped(pos_.x(), pos_.y(), pos_.x(), pos_.y());
    turtleMove nextMove = studentTurtleStep(bumpedFlag);

    // If bumpedFlag is true, the turtle should not move forward
    if (nextMove == MOVE_FORWARD && bumpedFlag) {
        // Don't update position, just return false to indicate failure
        return false;
    }

    // Update position and orientation based on nextMove
    pos_ = translatePos(pos_, nw_or, nextMove);
    nw_or = translateOrnt(nw_or, nextMove);

    // After translating the position, update visits and call displayVisits()
    int32_t x = static_cast<int32_t>(pos_.x() + 11);  // START_POS
    int32_t y = static_cast<int32_t>(pos_.y() + 11);
    incrementVisits(x, y);
    displayVisits(getVisits(x, y));

    // Return true/false based on whether movement succeeded
    return true;
}

/*
 * Function to check if the right-hand side is clear for movement
 * It checks based on the current orientation
 */
bool rightIsClear(QPointF pos_, int orientation) {
    switch (orientation) {
        case NORTH:
            return !bumped(pos_.x() + 1, pos_.y(), pos_.x() + 1, pos_.y()); // Check EAST (right of NORTH)
        case EAST:
            return !bumped(pos_.x(), pos_.y() + 1, pos_.x(), pos_.y() + 1); // Check SOUTH (right of EAST)
        case SOUTH:
            return !bumped(pos_.x() - 1, pos_.y(), pos_.x() - 1, pos_.y()); // Check WEST (right of SOUTH)
        case WEST:
            return !bumped(pos_.x(), pos_.y() - 1, pos_.x(), pos_.y() - 1); // Check NORTH (right of WEST)
        default:
            return false;
    }
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, int orientation, turtleMove nextMove) {
    switch (orientation) {
        case NORTH:
            if (nextMove == MOVE_FORWARD) {
                pos_.setY(pos_.y() - 1);  // Moving north decreases the Y coordinate
            }
            break;
        case EAST:
            if (nextMove == MOVE_FORWARD) {
                pos_.setX(pos_.x() + 1);  // Moving east increases the X coordinate
            }
            break;
        case SOUTH:
            if (nextMove == MOVE_FORWARD) {
                pos_.setY(pos_.y() + 1);  // Moving south increases the Y coordinate
            }
            break;
        case WEST:
            if (nextMove == MOVE_FORWARD) {
                pos_.setX(pos_.x() - 1);  // Moving west decreases the X coordinate
            }
            break;
        default:
            ROS_ERROR("Invalid orientation value: %d", orientation);
            break;
    }
    // If TURN_LEFT or TURN_RIGHT, no position change
    return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orientation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
    switch (orientation) {
        case NORTH:
            if (nextMove == TURN_RIGHT) {
                return EAST;  // Turn right to face East
            } else if (nextMove == TURN_LEFT) {
                return WEST;  // Turn left to face West
            }
            break;
        case EAST:
            if (nextMove == TURN_RIGHT) {
                return SOUTH; // Turn right to face South
            } else if (nextMove == TURN_LEFT) {
                return NORTH; // Turn left to face North
            }
            break;
        case SOUTH:
            if (nextMove == TURN_RIGHT) {
                return WEST;  // Turn right to face West
            } else if (nextMove == TURN_LEFT) {
                return EAST;  // Turn left to face East
            }
            break;
        case WEST:
            if (nextMove == TURN_RIGHT) {
                return NORTH; // Turn right to face North
            } else if (nextMove == TURN_LEFT) {
                return SOUTH; // Turn left to face South
            }
            break;
        default:
            ROS_ERROR("Invalid orientation value: %d", orientation);
            break;
    }
    return orientation;  // If no turn, return the same orientation
}

/**
 * @brief Function to increment the number of visits to a specific cell.
 */
void incrementVisits(int32_t x, int32_t y) {
    visitMap[x][y]++;
}

/**
 * @brief Function to get the number of visits to a specific cell.
 */
int32_t getVisits(int32_t x, int32_t y) {
    return visitMap[x][y];
}
