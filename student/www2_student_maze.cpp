#include "student.h"

// Function to move the turtle in the maze
bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int32_t timer = TIMEOUT;  // Timer for managing movement
    Flag bumpedFlag = false;         // Flag for whether the turtle has hit a wall
    Flag atEnd = false;              // Flag for reaching the maze's end

    if (timer == TIMER_EXPIRED) {
        // Determine the turtle's next move
        turtleMove nextMove = studentTurtleStep(bumpedFlag);

        // Update position and orientation
        pos_ = translatePos(pos_, nextMove, nw_or);
        nw_or = translateOrnt(nw_or, nextMove);

        // Convert to absolute coordinates (based on the maze's center)
        int32_t x = static_cast<int32_t>(pos_.x() + START_POS);
        int32_t y = static_cast<int32_t>(pos_.y() + START_POS);

        // Get the visit count for the current cell (relative in turtle, absolute in maze)
        int visits = getVisitCount();

        // Call displayVisits with the absolute coordinates and visit count
        displayVisits(visits);

        // Handle the end of the maze
        atEnd = atend(pos_.x(), pos_.y());
        if (atEnd) {
            return false;
        }

        // Reset the timer for the next tick
        timer = TIMEOUT;
    } else {
        // Decrement the timer
        timer -= TIME_DECREMENT;
    }

    return true;
}
