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

float w, cs; // Timer and state
float fx1, fy1, fx2, fy2; // Temporary positions for checking
float z, aend, mod, bp, q; // Various variables used in logic

// This procedure takes the current turtle position and orientation and returns
// true = submit changes, false = do not submit changes
// Ground rule: you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& pos_, int& nw_or) {
    ROS_INFO("Turtle update Called  w=%f", w);

    mod = true;

    // Timer countdown logic
    if (w == 0) { // Timer has completed its countdown, execute logic
        fx1 = pos_.x();
        fy1 = pos_.y();
        fx2 = pos_.x();
        fy2 = pos_.y();

        // Update future position based on current orientation
        if (nw_or < 2) {
            if (nw_or == 0) fy2 += 1;
            else fx2 += 1;
        } else {
            fx2 += 1;
            fy2 += 1;
            if (nw_or == 2) fx1 += 1;
            else fy1 += 1;
        }

        // Check if turtle is about to bump into a wall or has reached the end
        bp = bumped(fx1, fy1, fx2, fy2);
        aend = atend(pos_.x(), pos_.y());

        // Reversing logic for right-hand rule
        if (nw_or == 0) {
            if (cs == 2) { nw_or = 1; cs = 1; } // Turn right if possible
            else if (bp) { nw_or = 3; cs = 0; } // Turn left if bumped
            else cs = 2;
        } else if (nw_or == 1) {
            if (cs == 2) { nw_or = 2; cs = 1; } // Turn right if possible
            else if (bp) { nw_or = 0; cs = 0; } // Turn left if bumped
            else cs = 2;
        } else if (nw_or == 2) {
            if (cs == 2) { nw_or = 3; cs = 1; } // Turn right if possible
            else if (bp) { nw_or = 1; cs = 0; } // Turn left if bumped
            else cs = 2;
        } else if (nw_or == 3) {
            if (cs == 2) { nw_or = 0; cs = 1; } // Turn right if possible
            else if (bp) { nw_or = 2; cs = 0; } // Turn left if bumped
            else cs = 2;
        }

        ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);

        z = (cs == 2);
        mod = true;

        // Move the turtle in the direction of the current orientation
        if (z == true && aend == false) {
            if (nw_or == 1) pos_.setY(pos_.y() - 1);
            if (nw_or == 2) pos_.setX(pos_.x() + 1);
            if (nw_or == 3) pos_.setY(pos_.y() + 1);
            if (nw_or == 0) pos_.setX(pos_.x() - 1);
            z = false;
            mod = true;
        }
    }

    // Check if the turtle has reached the end of the maze
    if (aend) return false;

    // Update the timer
    if (w == 0) {
        w = TIMEOUT;
    } else {
        w -= 1;
    }

    // Submit the changes
    if (w == TIMEOUT) return true;

    return false;
}
