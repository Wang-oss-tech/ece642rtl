#include "student.h"

// Constants
const int32_t TIMEOUT = 40;           // Timer value to slow down the simulation for better visibility
const int32_t TIMER_EXPIRED = 0;      // Timer expired value
const int32_t TIME_DECREMENT = 1;     // Constant to decrement timer by
const int32_t MOVE_INCREMENT = 1;
const int32_t MOVE_DECREMENT = -1;
const int32_t START_POS = 11;

// Typedefs for future flexibility
typedef int32_t State;    // typedef for state representation
typedef bool Flag;        // typedef for boolean flag

// Defining struct for Position
typedef struct {
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
 */
bool moveTurtle(QPointF& pos_, int& nw_or) 
{
    static int32_t timer = TIMEOUT;                   // timer for managing movement
    Position futureX1, futureY1, futureX2, futureY2;  // future positions based on orientation
    Flag bumpedFlag = false;
    Flag atEnd = false;
    int32_t visitCount = 0;  // Variable to store visit count from the turtle code

    if (timer == TIMER_EXPIRED) {
        ROS_INFO("\n\nTIMER_EXPIRED START MOVE TURTLE: %d", timer);
        int old_nw_or = nw_or;

        futureX1.X = pos_.x();
        futureY1.Y = pos_.y();
        futureX2.X = pos_.x();
        futureY2.Y = pos_.y();
        
        // Update the future position based on the orientation
        switch (nw_or){
            case NORTH:
                futureY2.Y += MOVE_INCREMENT;
                break;
            case EAST:
                futureX2.X += MOVE_INCREMENT;
                break;
            case SOUTH:
                futureY2.Y += MOVE_INCREMENT;
                futureX1.X += MOVE_INCREMENT;
                break;
            case WEST:
                futureX2.X += MOVE_INCREMENT;
                futureY2.Y += MOVE_INCREMENT;
                break;
            default:
                ROS_ERROR("Invalid orientation value 1: %d", nw_or);
                break;
        }

        bumpedFlag = bumped(futureX1.X, futureY1.Y, futureX2.X, futureY2.Y);
        atEnd = atend(pos_.x(), pos_.y());

        // Call to studentTurtleStep() and get the visit count from the turtle
        turtleMove nextMove = studentTurtleStep(bumpedFlag, visitCount, pos_.x() + START_POS, pos_.y() + START_POS);

        nw_or = translateOrnt(nw_or, nextMove);  // Update orientation based on the turtle's next move

        if (nextMove == MOVE_FORWARD && !atEnd) {
            pos_ = translatePos(pos_, nextMove, old_nw_or);  // Update position

            // After moving, update the visit display with the count
            displayVisits(visitCount);  // Call displayVisits() in the maze file

            ROS_INFO("UPDATE POSITION (X, Y): %f, %f", pos_.x(), pos_.y());
        }
    }

    if (atEnd){
        return false;
    }

    timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - TIME_DECREMENT;
    return (timer == TIMEOUT);
}
