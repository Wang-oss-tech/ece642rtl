/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2    
 * LAST UPDATE: 9/8/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionally left obfuscated.
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) {
    return MOVE;
}

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT 40    // Bigger number slows down simulation so you can see what's happening

float timer, currentState; // Timer and state
float shouldMove, atEnd, modifyFlag, bumpedFlag; // Various variables used in logic

// Enum to replace magic numbers for directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Function to check the direction and decide on the new orientation and state
void checkDirection(int& orientation, bool bumpedFlag) {
    if (orientation == NORTH) {
        if (currentState == 2) {
            orientation = EAST;  // Turn right to face East if possible
            currentState = 1;
        } else if (bumpedFlag) {
            orientation = WEST;  // Turn left to face West if bumped
            currentState = 0;
        } else {
            currentState = 2;    // Move forward if no bump
        }
    } else if (orientation == EAST) {
        if (currentState == 2) {
            orientation = SOUTH;  // Turn right to face South if possible
            currentState = 1;
        } else if (bumpedFlag) {
            orientation = NORTH;  // Turn left to face North if bumped
            currentState = 0;
        } else {
            currentState = 2;    // Move forward if no bump
        }
    } else if (orientation == SOUTH) {
        if (currentState == 2) {
            orientation = WEST;  // Turn right to face West if possible
            currentState = 1;
        } else if (bumpedFlag) {
            orientation = EAST;  // Turn left to face East if bumped
            currentState = 0;
        } else {
            currentState = 2;    // Move forward if no bump
        }
    } else if (orientation == WEST) {
        if (currentState == 2) {
            orientation = NORTH;  // Turn right to face North if possible
            currentState = 1;
        } else if (bumpedFlag) {
            orientation = SOUTH;  // Turn left to face South if bumped
            currentState = 0;
        } else {
            currentState = 2;    // Move forward if no bump
        }
    }
}

// Function to update the position of the turtle based on the current orientation
void updatePosition(QPointF& position, int orientation) {
    if (orientation == EAST) {
        position.setY(position.y() - 1); // Move East (right)
    } else if (orientation == SOUTH) {
        position.setX(position.x() + 1); // Move South (down)
    } else if (orientation == WEST) {
        position.setY(position.y() + 1); // Move West (left)
    } else if (orientation == NORTH) {
        position.setX(position.x() - 1); // Move North (up)
    }
}

// This procedure takes the current turtle position and orientation and returns
// true = submit changes, false = do not submit changes
// Ground rule: you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& position, int& orientation) {
    ROS_INFO("Turtle update Called  timer=%f", timer);

    modifyFlag = true;

    // Timer countdown logic
    if (timer == 0) { // Timer has completed its countdown, execute logic
        float futureX1 = position.x();
        float futureY1 = position.y();
        float futureX2 = position.x();
        float futureY2 = position.y();

        // Determine the future position based on the current orientation
        if (orientation == NORTH) {
            futureY2 += 1; // Moving North increases Y
        } else if (orientation == EAST) {
            futureX2 += 1; // Moving East increases X
        } else if (orientation == SOUTH) {
            futureX2 += 1; // Moving South increases X and Y (diagonal)
            futureY2 += 1;
            futureX1 += 1;
        } else if (orientation == WEST) {
            futureX2 += 1; // Moving West increases X and Y (diagonal)
            futureY2 += 1;
            futureY1 += 1;
        }

        // Check if turtle is about to bump into a wall or has reached the end
        bumpedFlag = bumped(futureX1, futureY1, futureX2, futureY2);
        atEnd = atend(position.x(), position.y());

        // Check direction and update orientation
        checkDirection(orientation, bumpedFlag);

        ROS_INFO("Orientation=%f  STATE=%f", orientation, currentState);

        shouldMove = (currentState == 2);
        modifyFlag = true;

        // Move the turtle in the direction of the current orientation
        if (shouldMove && !atEnd) {
            updatePosition(position, orientation);
            shouldMove = false;
            modifyFlag = true;
        }
    }

    // Check if the turtle has reached the end of the maze
    if (atEnd) return false;

    // Update the timer
    if (timer == 0) {
        timer = TIMEOUT;
    } else {
        timer -= 1;
    }

    // Submit the changes
    if (timer == TIMEOUT) return true;
    return false;
}
