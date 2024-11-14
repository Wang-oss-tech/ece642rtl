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

// Constants
const int32_t TIMEOUT = 1;           // Timer value to slow down the simulation for better visibility
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
        default:
            ROS_ERROR("INVALID ORIENTATION");
            break;
    }
} 

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 * 
 * nw_or = orientation pos_ = position
 */
bool moveTurtle(QPointF& pos_, int& nw_or) 
{
  static int32_t timer = TIMEOUT;                   // timer for managing movement
  
  Position futureX1, futureY1, futureX2, futureY2;  // future posistions based on orientation
  Flag bumpedFlag = false;
  Flag atEnd = false;

  if (timer == TIMER_EXPIRED){
    int old_nw_or = nw_or;
    Flag shouldMove = false; // Flag to determine if turtle should move
    futureX1.X = static_cast<int32_t>(pos_.x());
    futureY1.Y = static_cast<int32_t>(pos_.y());
    futureX2.X = static_cast<int32_t>(pos_.x());
    futureY2.Y = static_cast<int32_t>(pos_.y());
    switch (nw_or){
      case NORTH: // moving north increases Y
        futureY2.Y += MOVE_INCREMENT; 
        break;
      case EAST: // moving east increases x
        futureX2.X += MOVE_INCREMENT; 
        break;
      case SOUTH: // moving south increases both X and Y (diagonal)
        futureX2.X += MOVE_INCREMENT;
        futureY2.Y += MOVE_INCREMENT; 
        futureX1.X += MOVE_INCREMENT;
        break;
      case WEST: // moving west increases both X and Y (diagonal)
        futureX2.X += MOVE_INCREMENT;
        futureY2.Y += MOVE_INCREMENT; 
        futureY1.Y += MOVE_INCREMENT;
        break;
      default:
        ROS_ERROR("Invalid orientation value 1: %d", nw_or);
        break;
    }
    bumpedFlag = bumped(futureX1.X, futureY1.Y, futureX2.X, futureY2.Y);
    atEnd = atend(static_cast<int32_t>(pos_.x()), static_cast<int32_t>(pos_.y()));

    // Call to studentTurtleStep() to determine next step based on whether a bump occurred
    std::pair<turtleMove, int> result = studentTurtleStep(bumpedFlag, nw_or);
    turtleMove nextMove = result.first;
    int numTurns = result.second;

    if (numTurns != 0 && nextMove != MOVE_FORWARD){
        nw_or = translateOrnt(nw_or); // update orientation
    }
    shouldMove = (nextMove == MOVE_FORWARD);

    if (shouldMove && !atEnd) {
      pos_ = translatePos(pos_, nextMove, old_nw_or);            // updates Position
      updatePosition(nw_or);
      int visits = getVisits(relativeX, relativeY);             // Get the visit count for the current position
      displayVisits(visits);                                    // Update the display with the visit count
      shouldMove = false;
    }
  }

  if (atEnd){
    return false;
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
  }
  // Default return in case no return is reached in the above logic
  return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation) {
  switch (orientation){
    case NORTH:
      orientation = WEST;  // Turn left to face West if bumped
      break;
    case EAST:
      orientation = NORTH;  // Turn left to face North if bumped
      break;
    case SOUTH:
      orientation = EAST;  // Turn left to face East if bumped
      break;
    case WEST:
      orientation = SOUTH;  // Turn left to face South if bumped
      break;
    default:
      ROS_ERROR("Invalid orientation value 3: %d", orientation);
  }
  return orientation;  // Make sure to return the updated orientation
}