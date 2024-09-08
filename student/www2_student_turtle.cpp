#include "student.h"

// Enumeration for direction
enum Direction { NORTH, EAST, SOUTH, WEST };

// Helper function for turning right
Direction turnRight(Direction dir) {
    return static_cast<Direction>((dir + 1) % 4);
}

// Helper function for turning left
Direction turnLeft(Direction dir) {
    return static_cast<Direction>((dir + 3) % 4);
}

// Function to update turtle position based on current direction
void moveForward(QPointF& pos_, Direction dir) {
    switch (dir) {
        case NORTH: pos_.setY(pos_.y() - 1); break;
        case EAST: pos_.setX(pos_.x() + 1); break;
        case SOUTH: pos_.setY(pos_.y() + 1); break;
        case WEST: pos_.setX(pos_.x() - 1); break;
    }
}

// Declare or initialize the necessary variables
bool studentMoveTurtle(QPointF& pos_, Direction& dir) {
    // Static variables to maintain state across function calls
    static float w = TIMEOUT; // Timer, initialized to TIMEOUT
    static float mod = true;  // Modification flag, initially true
    static float cs = 0;      // Current state, initially 0

    ROS_INFO("Turtle update Called  w=%f", w);

    if(w == 0) {
        // Determine future positions based on current direction
        float futureX1 = pos_.x(), futureY1 = pos_.y();
        float futureX2 = pos_.x(), futureY2 = pos_.y();

        if (dir == NORTH) futureY2 += 1;
        else if (dir == EAST) futureX2 += 1;
        else if (dir == SOUTH) futureX1 += 1;
        else if (dir == WEST) futureY1 += 1;

        // Check if the turtle is bumped or at the end
        bool bumped = bumped(futureX1, futureY1, futureX2, futureY2);
        bool atEnd = atend(pos_.x(), pos_.y());

        // Right-hand rule logic
        if(cs == 2) {
            dir = turnRight(dir); 
            cs = 1; 
        } else if (bumped) {
            dir = turnLeft(dir); 
            cs = 0; 
        } else {
            cs = 2;
        }

        ROS_INFO("Orientation=%d  STATE=%f", dir, cs);

        // Move the turtle if conditions allow
        if(cs == 2 && !atEnd) {
            moveForward(pos_, dir);
        }
    }

    if (atEnd) return false;
    if (w == 0) w = TIMEOUT; else w -= 1;
    if (w == TIMEOUT) return true;
    
    return false;
}
