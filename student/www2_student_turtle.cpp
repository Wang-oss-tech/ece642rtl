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
float futureX1, futureY1, futureX2, futureY2; // Temporary positions for checking
float shouldMove, atEnd, modifyFlag, bumpedFlag, tempVar; // Various variables used in logic

// This procedure takes the current turtle position and orientation and returns
// true = submit changes, false = do not submit changes
// Ground rule: you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& position, int& orientation) {
    ROS_INFO("Turtle update Called  timer=%f", timer);

    modifyFlag = true;

    // Timer countdown logic
    if (timer == 0) { // Timer has completed its countdown, execute logic
        futureX1 = position.x();
        futureY1 = position.y();
        futureX2 = position.x();
        futureY2 = position.y();

        // Update future position based on current orientation
        if (orientation < 2) {
            if (orientation == 0) futureY2 += 1;
            else futureX2 += 1;
        } else {
            futureX2 += 1;
            futureY2 += 1;
            if (orientation == 2) futureX1 += 1;
            else futureY1 += 1;
        }

        // Check if turtle is about to bump into a wall or has reached the end
        bumpedFlag = bumped(futureX1, futureY1, futureX2, futureY2);
        atEnd = atend(position.x(), position.y());

        // Reversing logic for right-hand rule
        if (orientation == 0) {
            if (currentState == 2) { orientation = 1; currentState = 1; } // Turn right if possible
            else if (bumpedFlag) { orientation = 3; currentState = 0; } // Turn left if bumped
            else currentState = 2;
        } else if (orientation == 1) {
            if (currentState == 2) { orientation = 2; currentState = 1; } // Turn right if possible
            else if (bumpedFlag) { orientation = 0; currentState = 0; } // Turn left if bumped
            else currentState = 2;
        } else if (orientation == 2) {
            if (currentState == 2) { orientation = 3; currentState = 1; } // Turn right if possible
            else if (bumpedFlag) { orientation = 1; currentState = 0; } // Turn left if bumped
            else currentState = 2;
        } else if (orientation == 3) {
            if (currentState == 2) { orientation = 0; currentState = 1; } // Turn right if possible
            else if (bumpedFlag) { orientation = 2; currentState = 0; } // Turn left if bumped
            else currentState = 2;
        }

        ROS_INFO("Orientation=%f  STATE=%f", orientation, currentState);

        shouldMove = (currentState == 2);
        modifyFlag = true;

        // Move the turtle in the direction of the current orientation
        if (shouldMove && !atEnd) {
            if (orientation == 1) position.setY(position.y() - 1);
            if (orientation == 2) position.setX(position.x() + 1);
            if (orientation == 3) position.setY(position.y() + 1);
            if (orientation == 0) position.setX(position.x() - 1);
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
