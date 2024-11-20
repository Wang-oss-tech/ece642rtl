/*
 * Name: William Wang
 * ID: www2
 *
 * For any call to bumped (x1,y1,x2,y2), the turtle shall be facing the 
 * wall segment with endpoints (x1,y1) and (x2, y2)
 */

#include <map>
#include <string>
#include "monitor_interface.h"

// Turn orientation macros into strings
static std::map<int, std::string> or_string {
    {NORTH, "North"},
    {EAST, "East"},
    {SOUTH, "South"},
    {WEST, "West"}
};


// Difference in X1
static std::map<int, int> x1_diff {
    {NORTH, 0},
    {EAST, 1},
    {SOUTH, 0},
    {WEST, 0}
};

// Difference in X2
static std::map<int, int> x2_diff {
    {NORTH, 1},
    {EAST, 1},
    {SOUTH, 1},
    {WEST, 0}
};

// Difference in Y1
static std::map<int, int> y1_diff {
    {NORTH, 0},
    {EAST, 0},
    {SOUTH, 1},
    {WEST, 0}
};

// Difference in Y2
static std::map<int, int> y2_diff {
    {NORTH, 0},
    {EAST, 1},
    {SOUTH, 1},
    {WEST, 1}
};


// current position
static Pose current_position;

// updated position
static Pose moved_pos;

// current orientation
static Orientation curr_or;

// number of walls to check
const int num_walls = 4;

// first position change status
bool initial_pos = true;


void tickInterrupt(ros::Time t) {
}

// position and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (initial_pos){
        current_position.x = x;
        current_position.y = y;
        curr_or = o;
        initial_pos = false;
    }
}


void visitInterrupt(ros::Time t, int visits) {
}


// interrupt occurs when bumped() is called 
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if(!initial_pos){
        if((x1 == current_position.x + x1_diff[curr_or]) &&
           (x2 == current_position.x + x2_diff[curr_or]) &&
           (y1 == current_position.y + y1_diff[curr_or]) &&
           (y2 == current_position.y + y2_diff[curr_or])){
            ROS_INFO("[[%ld ns]] Bump correct at [%d,%d] at orientation %s", 
                    t.toNSec(), 
                    current_position.x, 
                    current_position.y, 
                    or_string[curr_or].c_str());
        } else {
            ROS_WARN("VIOLATION: Incorrect bump at (%d,%d) at orientation %s"
                     "(x1, y1), (x2, y2): (%d,%d), (%d,%d)", 
                      current_position.x, 
                      current_position.y,
                      or_string[curr_or].c_str(),
                      x1, y1, x2, y2);
        }
        initial_pos = true;
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
