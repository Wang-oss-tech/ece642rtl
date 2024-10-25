/* 
 *
 * STUDENT NAME: William Wang 
 * ANDREW ID: www2  
 * LAST UPDATE: 10/24/2024
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

// Constants
const int32_t TIMEOUT = 40;           // Timer value to slow down the simulation for better visibility
const int32_t TIMER_EXPIRED = 0;      // Timer expired value
const int32_t TIME_DECREMENT = 1;     // Constant to decrement timer by
const int32_t MOVE_INCREMENT = 1;
const int32_t MOVE_DECREMENT = -1;

// Typedefs for future flexibility
typedef int32_t State;    // typedef for state representation
typedef bool Flag;        // typedef for boolean flag

// Defining struct for Position
typedef struct{
  int32_t X;
  int32_t Y;
} Position;

// Position Count
static int32_t relativeX = START_POS;
static int32_t relativeY = START_POS;

/**
 * @brief Updates the current turtle's position based on its direction.
 * The turtle moves forward by 1 unit based on the current direction.
 */
void updatePosition(int nw_or) {
    ROS_INFO("Update Position Called w/ Direction %d", nw_or);
    switch (nw_or) {
        case NORTH:
            relativeX -= 1;  // Move north (up in Y axis)
            break;
        case EAST:
            relativeY -= 1;  // Move east (right in X axis)
            break;
        case SOUTH:
            relativeX += 1;  // Move south (down in Y axis)
            break;
        case WEST:
            relativeY += 1;  // Move west (left in X axis)
            break;
    }
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int32_t timer = TIMEOUT;
    Flag bumpedFlag = false;
    Flag atEnd = false;

    if (timer == TIMER_EXPIRED) {
        bumpedFlag = bumped(pos_.x(), pos_.y(), pos_.x(), pos_.y());
        atEnd = atend(pos_.x(), pos_.y());

        if (atEnd) {
            return false;  // Stop the turtle if it reaches the goal
        }

        turtleMove nextMove = studentTurtleStep(bumpedFlag, nw_or);
        nw_or = translateOrnt(nw_or, nextMove);  // Update orientation

        if (nextMove == MOVE_FORWARD) {
            pos_ = translatePos(pos_, nextMove, nw_or);
            updatePosition(nw_or);
            int visits = getVisits(relativeX, relativeY);  // Get visit count
            ROS_INFO("X = %d, Y = %d, visits = %d", relativeX, relativeY, visits);
            ROS_INFO("Moving Forward\n\n");
            displayVisits(visits);  // Update display with visit count
        }
    }

    timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - TIME_DECREMENT;
    return (timer == TIMEOUT);
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or) {
  if (nextMove == MOVE_FORWARD){
    switch (nw_or){
      case EAST:
        pos_.setY(pos_.y() + MOVE_DECREMENT); // Move East (right)
        break;
      case SOUTH:
        pos_.setX(pos_.x() + MOVE_INCREMENT); // Move South (down)
        break;
      case WEST:
        pos_.setY(pos_.y() + MOVE_INCREMENT); // Move West (left)
        break;
      case NORTH:
        pos_.setX(pos_.x() + MOVE_DECREMENT); // Move North (up)
        break;
      default:
        ROS_ERROR("Invalid orientation value 2: %d", nw_or);
        break;
    }
    return pos_;
  } else {
    ROS_INFO("Next move is not MOVE_FORWARD, no position update");
  }
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  switch (orientation){
    case NORTH:
      if (nextMove == TURN_RIGHT) {
        orientation = EAST;  // Turn right to face East
      } else if (nextMove == TURN_LEFT) { 
        orientation = WEST;  // Turn left to face West if bumped
      } 
      break;
    case EAST:
      if (nextMove == TURN_RIGHT) {
        orientation = SOUTH;  // Turn right to face South
      } else if (nextMove == TURN_LEFT) { 
        orientation = NORTH;  // Turn left to face North if bumped
      }
      break;
    case SOUTH:
      if (nextMove == TURN_RIGHT) {
        orientation = WEST;  // Turn right to face West
      } else if (nextMove == TURN_LEFT) { 
        orientation = EAST;  // Turn left to face East if bumped
      } 
      break;
    case WEST:
      if (nextMove == TURN_RIGHT) {
        orientation = NORTH;  // Turn right to face North
      } else if (nextMove == TURN_LEFT) { 
        orientation = SOUTH;  // Turn left to face South if bumped
      } 
      break;
    default:
      ROS_ERROR("Invalid orientation value 3: %d", orientation);
  }
  return orientation;  // Make sure to return the updated orientation
}