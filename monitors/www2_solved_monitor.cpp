/*
 * Name: William Wang
 * ID: www2
 *
 * If the turtle has solved the maze (atEnd(x,y)==true), it shall not move or turn
 */

#include <map>
#include <string>
#include "monitor_interface.h"


// atEnd status variable
static bool at_ending = false;


// interrupt called when position and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (at_ending == true){
        ROS_WARN("VIOLATION: atEnd() true. However, moved even after reaching end");
    }
}

// interrupt occurs when atend(x,y) retunr
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (atEnd == true){
        at_ending = true;
        ROS_INFO("[[%ld ns]] atEnd returns true at (%d, %d)", x, y);
    }
}


/* Empty Interrupts*/
void tickInterrupt(ros::Time t) {
}


void visitInterrupt(ros::Time t, int visits) {
}

// interrupt occurs when bumped() is called 
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}
