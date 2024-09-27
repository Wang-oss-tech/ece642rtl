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

// Enum to represent direction/orientation
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
 * 
 * nw_or = orientation pos_ = position
 */
bool moveTurtle(QPointF& pos_, int& nw_or) 
{
  // bool bumped = true; // Replace with your own procedure
  // turtleMove nextMove = studentTurtleStep(bumped); // define your own turtleMove enum or structure
  // pos_ = translatePos(pos_, nextMove);
  // nw_or = translateOrnt(nw_or, nextMove);

  // // REPLACE THE FOLLOWING LINE IN PROJECT 5
  // return studentMoveTurtle(pos_, nw_or);
  static int32_t timer = TIMEOUT;                   // timer for managing movement
  
  Position futureX1, futureY1, futureX2, futureY2;  // future posistions based on orientation
  Flag bumpedFlag = false;
  Flag shouldMove = false;
  Flag atEnd = false;

  // ROS_INFO("MOVE TURTLE CALLED - TIMER VALUE: %d", timer);

  if (timer == TIMER_EXPIRED){
    ROS_INFO("TIMER_EXPIRED START MOVE TURTLE: %d", timer);
    ROS_INFO("Before Updating Position (X, Y) 1: %f, %f", pos_.x(), pos_.y());

    futureX1.X = pos_.x();
    futureY1.Y = pos_.y();
    futureX2.X = pos_.x();
    futureY2.Y = pos_.y();

    switch (nw_or){
      case NORTH:
        futureY2.Y += MOVE_INCREMENT; // moving north increases Y
        ROS_INFO("ENTERS NORTH CASE");
        break;
      case EAST:
        futureX2.X += MOVE_INCREMENT; // moving east increases x
        ROS_INFO("ENTERS EAST CASE");
        break;
      case SOUTH:
        futureX2.X += MOVE_INCREMENT;
        futureY2.Y += MOVE_INCREMENT; // moving south increases both X and Y (diagonal)
        futureX1.X += MOVE_INCREMENT;
        ROS_INFO("ENTERS SOUTH CASE");
        break;
      case WEST:
        futureX2.X += MOVE_INCREMENT;
        futureY2.Y += MOVE_INCREMENT; // moving west increases both X and Y (diagonal)
        futureY1.Y += MOVE_INCREMENT;
        ROS_INFO("ENTERS WEST CASE");
        break;
      default: 
        ROS_ERROR("Invalid orientation value: %d", nw_or);
        break;
    }


    ROS_INFO("future (X1, Y1): %d", futureX1.X, futureY1.Y);
    ROS_INFO("future (X2, Y2): %d", futureX2.X, futureY2.Y);

    bumpedFlag = bumped(futureX1.X, futureY1.Y, futureX2.X, futureY2.Y);
    ROS_INFO("BUMPEDFLAG: %d", bumpedFlag);
    atEnd = atend(pos_.x(), pos_.y());
    ROS_INFO("atEnd: %d", atEnd);




    // Call to studentTurtleStep() to determine next step based on whether a bump occurred
    turtleMove nextMove = studentTurtleStep(bumpedFlag);

    ROS_INFO("nextMove: %d", nextMove);
    nw_or = translateOrnt (nw_or, nextMove);        // update orientation
    ROS_INFO("Updates Orientation!: %d", nw_or);
    shouldMove = (nextMove == MOVE_FORWARD);

    ROS_INFO("After Updating Position (X, Y) 5: %f, %f", pos_.x(), pos_.y());
    if (shouldMove && !atEnd) {
      pos_ = translatePos(pos_, nextMove, nw_or);            // updates Position
    }
    ROS_INFO("Updated Position (X, Y): %d, %d", pos_.x(), pos_.y());

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
  // return pos_;
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
      pos_.setY(pos_.x() + MOVE_DECREMENT); // Move North (up)
    default:
      ROS_ERROR("Invalid orientation value: %d", nw_or);
      break;
  }
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  // return orientation;
  switch (orientation){
    case NORTH:
      if (nextMove == MOVE_FORWARD) {
        orientation = EAST;  // Turn right to face East
      } else if (nextMove == TURN_LEFT) { 
        orientation = WEST;  // Turn left to face West if bumped
      }
      break;
    case EAST:
      if (nextMove == MOVE_FORWARD) {
        orientation = SOUTH;  // Turn right to face South
      } else if (nextMove == TURN_LEFT) { 
        orientation = NORTH;  // Turn left to face North if bumped
      }
      break;
    case SOUTH:
      if (nextMove == MOVE_FORWARD) {
        orientation = WEST;  // Turn right to face West
      } else if (nextMove == TURN_LEFT) { 
        orientation = EAST;  // Turn left to face East if bumped
      }
      break;
    case WEST:
      if (nextMove == MOVE_FORWARD) {
        orientation = NORTH;  // Turn right to face North
      } else if (nextMove == TURN_LEFT) { 
        orientation = SOUTH;  // Turn left to face South if bumped
      }
      break;
    default:
      ROS_ERROR("Invalid orientation value: %d", orientation);
  }

  return orientation;  // Make sure to return the updated orientation
}
