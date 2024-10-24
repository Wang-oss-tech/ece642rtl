#include "student.h"

// Timer constants
const int32_t TIMEOUT = 40;
const int32_t TIMER_EXPIRED = 0;
const int32_t TIME_DECREMENT = 1;

// Position tracking variables
static int32_t relativeX = START_POS;
static int32_t relativeY = START_POS;

// Update the relative position based on the orientation
void updatePosition(int nw_or) {
    switch (nw_or) {
        case NORTH: relativeX--; break;
        case EAST:  relativeY++; break;
        case SOUTH: relativeX++; break;
        case WEST:  relativeY--; break;
    }
}

// Main function to control the turtle's movement
bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int32_t timer = TIMEOUT;
    bool bumpedFlag = false;
    bool atEnd = false;

    if (timer == TIMER_EXPIRED) {
        // Check if the turtle has reached the goal or encountered a bump
        bumpedFlag = bumped(pos_.x(), pos_.y(), pos_.x(), pos_.y());
        atEnd = atend(pos_.x(), pos_.y());

        // If the turtle reaches the goal, stop
        if (atEnd) return false;

        // Get the next move from the turtle's logic
        turtleMove nextMove = studentTurtleStep(bumpedFlag, nw_or);
        nw_or = translateOrnt(nw_or, nextMove);  // Update orientation

        // If the move is forward, update the position and visits
        if (nextMove == MOVE_FORWARD) {
            pos_ = translatePos(pos_, nextMove, nw_or);  // Use translatePos
            updatePosition(nw_or);  // Update the relative position
            int visits = getVisits(relativeX, relativeY);  // Get visit count
            displayVisits(visits);  // Update the display
        }
    }

    // Reset or decrement the timer
    timer = (timer == TIMER_EXPIRED) ? TIMEOUT : timer - TIME_DECREMENT;
    return (timer == TIMEOUT);
}

// Helper function to translate position based on move
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or) {
    if (nextMove == MOVE_FORWARD) {
        switch (nw_or) {
            case EAST:  pos_.setY(pos_.y() + MOVE_DECREMENT); break;
            case SOUTH: pos_.setX(pos_.x() + MOVE_INCREMENT); break;
            case WEST:  pos_.setY(pos_.y() + MOVE_INCREMENT); break;
            case NORTH: pos_.setX(pos_.x() + MOVE_DECREMENT); break;
            default: ROS_ERROR("Invalid orientation value: %d", nw_or); break;
        }
    }
    return pos_;
}

// Helper function to translate orientation based on move
int translateOrnt(int orientation, turtleMove nextMove) {
    switch (orientation) {
        case NORTH: return (nextMove == TURN_RIGHT) ? EAST : WEST;
        case EAST:  return (nextMove == TURN_RIGHT) ? SOUTH : NORTH;
        case SOUTH: return (nextMove == TURN_RIGHT) ? WEST : EAST;
        case WEST:  return (nextMove == TURN_RIGHT) ? NORTH : SOUTH;
        default: ROS_ERROR("Invalid orientation value: %d", orientation); return orientation;
    }
}
