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


// inital status call for poseInterrupt
static bool init_pos = false;

// current position
static Pose current_position;


// interrupt called when position and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_position.x = x;
    current_position.y = y; 
}

// interrupt occurs when atend(x,y) returns
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (init_pos == true){
        if(current_position == x &&
           current_position == y){
            ROS_INFO("[[%ld ns]] AtEnd at correct final location [%d,%d]", 
                      t.toNSec(), x, y);
        } else{
            ROS_WARN("VIOLATION: Current position is [%d, %d]
                      with supposed AtEnd location at [%d, %d]",
                      current_position.x, current_position.y,
                      x, y);
        }
    }
    init_pos = true;
}


/* Empty Interrupts*/
void tickInterrupt(ros::Time t) {
}


void visitInterrupt(ros::Time t, int visits) {
}

// interrupt occurs when bumped() is called 
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}
