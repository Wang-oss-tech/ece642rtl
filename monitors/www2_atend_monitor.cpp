/*
 * Name: William Wang
 * ID: www2
 *
 *  Turtle shall only call atEnd(x,y) if it is at a position x,y. 
 * (No remotely fishing around the maze to figure out where the goal is for path planning.)
 * 
 */

#include <map>
#include <string>
#include "monitor_interface.h"

// Initial status call for poseInterrupt
static bool init_pos = false;

// Current position
static Pose current_position;

// Indicates if the turtle has reached the goal (to output success messages)
static bool reached_goal = false;

// Interrupt called when position and orientation are updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_position.x = x;
    current_position.y = y; 
}

// Interrupt occurs when atEnd(x, y) is called
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (init_pos) {
        if (current_position.x == x && current_position.y == y) {
            if (atEnd) {
                // Turtle is at the correct position and correctly calls atEnd
                reached_goal = true;
            } 
            ROS_INFO("Successful atEnd at [%d, %d] at time [%ld ns]", x, y, t.toNSec());
        } else {
            // Violation: Turtle is not at the position it is calling atEnd for
            ROS_WARN("VIOLATION: Current position is [%d, %d] with supposed atEnd location at [%d, %d]",
                     current_position.x, current_position.y, x, y);
        }
    }
    init_pos = true;
}

// Tick interrupt occurs periodically (outputs success messages to flush the queue)
void tickInterrupt(ros::Time t) {
    if (reached_goal) {
        ROS_INFO("Flushing output queue: Turtle successfully reached the end.");
    }
}

// Interrupt for visit count (not used in this monitor)
void visitInterrupt(ros::Time t, int visits) {
}

// Interrupt occurs when bumped() is called (not used in this monitor)
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}
