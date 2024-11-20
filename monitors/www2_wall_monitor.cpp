/*
 * Name: William Wang
 * ID: www2
 *
 * Turtle shall not go through walls
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

// current position
static Pose current_position;

// updated position
static Pose moved_pos;

// current orientation
static Orientation curr_or;

// number of walls to check
const int num_walls = 4;

// array for storing bump data
static bool bump_data[num_walls];

void tickInterrupt(ros::Time t) {
}

// position and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_position.x = x;
    current_position.y = y;
    curr_or = o;

    if (!((current_position.y == moved_pos.y) && 
          (current_position.x == moved_pos.x))){
            if (bump_data[o] == false){
                ROS_INFO("[[%ld ns]] Wall not detected,"
                         "correct update at (%d,%d)", 
                          t.toNSec(), 
                          moved_pos.x, 
                          moved_pos.y);
            } else{
                ROS_WARN("VIOLATION[[%ld ns]] Moved through wall at"
                         " (%d,%d)", 
                          moved_pos.x, 
                          moved_pos.y);
            }
            moved_pos.x = x;
            moved_pos.y = y;
    }
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (current_position.y == moved_pos.y &&
        current_position.x == moved_pos.x){
        bump_data[curr_or] = bumped;
    } else{
        for(int i = 0; i < num_walls; i++){
            bump_data[i] = true;
        }
    }
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}


void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
