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
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"
#include <stdint.h> // fixed-width integer types

// Size of maze array
const int32_t MAZE_SIZE = 23; // (23x23) internal tracking array
const int32_t START_POS = 11; // middle starting position in (23x23) array

// ENUM to represent direction
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF& pos_, int& nw_or)
{
  bool bumped = false; // logic to determine if turtle bumped should be added here

  // call to get next turtle move from student_turtle.cpp
  turtleMove nextMove = studentTurtleStep(bumped); 

  // update position & orientation based on nextMove
  pos_ = translatePos(pos_, nw_or, nextMove);
  nw_or = translateOrnt(nw_or, nextMove);

  // after translating the position, update visits & call displayVisits()
  int32_t x = static_cast<int32_t>(pos_.x() + START_POS);
  int32_t y = static_cast<int32_t>(pos_.y() + START_POS);
  incrementVisits(x,y);
  displayVisits(getVisits(x,y));

  // Return true/false based on whether movement succeeded
  return true;
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
 * Takes an orientation and a turtleMove and returns a new orienation
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