/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: William Wang
 * ANDREW ID: www2	
 * LAST UPDATE: 9/8/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) {return MOVE;}

// Enumeration for direction
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Define the TIMEOUT constant
#define TIMEOUT 40

// Function to turn the turtle right
Direction turnRight(Direction dir) {
    return static_cast<Direction>((dir + 1) % 4);
}

// Function to turn the turtle left
Direction turnLeft(Direction dir) {
    return static_cast<Direction>((dir + 3) % 4);
}

// Function to move the turtle forward based on current direction
void moveForward(QPointF& pos_, Direction dir) {
    switch (dir) {
        case NORTH: pos_.setY(pos_.y() + 1); break;
        case EAST: pos_.setX(pos_.x() + 1); break;
        case SOUTH: pos_.setY(pos_.y() - 1); break;
        case WEST: pos_.setX(pos_.x() - 1); break;
    }
}

// Function to determine if the turtle should move and update its state
bool studentMoveTurtle(QPointF& pos_, Direction& dir) {
    // Static variables to maintain state across function calls
    static float timer = TIMEOUT;
    static float currentState = 0;
    bool modify = true;

    // Declare atEnd at the beginning of the function
    bool atEnd;

    ROS_INFO("Turtle update Called  timer=%f", timer);

    if (timer == 0) {
        // Determine future positions based on current direction
        float futureX1 = pos_.x(), futureY1 = pos_.y();
        float futureX2 = pos_.x(), futureY2 = pos_.y();

        if (dir == NORTH) futureY2 += 1;
        else if (dir == EAST) futureX2 += 1;
        else if (dir == SOUTH) futureX1 += 1;
        else if (dir == WEST) futureY1 += 1;

        // Check if the turtle is bumped or at the end
        bool isBumped = bumped(static_cast<int)(futureX1), static_cast<int>(futureY1), static_cast<int)(futureX2), static_cast<int>(futureY2));

        // Assign a value to atEnd
        atEnd = atend(static_cast<int>(pos_.x()), static_cast<int>(pos_.y()));

        // Apply the right-hand rule logic
        if (currentState == 2) {
            dir = turnRight(dir);
            currentState = 1;
        } else if (isBumped) {
            dir = turnLeft(dir);
            currentState = 0;
        } else {
            currentState = 2;
        }

        ROS_INFO("Orientation=%d  STATE=%f", dir, currentState);

        // Move the turtle if conditions allow
        if (currentState == 2 && !atEnd) {
            moveForward(pos_, dir);
        }
    }

    // Now atEnd can be used here, outside the if statement where it was initially assigned
    if (atEnd) return false;

    // Update the timer
    if (timer == 0) timer = TIMEOUT; else timer -= 1;
    if (timer == TIMEOUT) return true;

    return false;
}
