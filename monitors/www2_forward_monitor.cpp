/*
 * Name: William Wang
 * ID: www2
 *
 * Turtle the turtle shall face the direction it is moving
 */

#include <map>
#include <string>
#include "monitor_interface.h"

// tick called fist time
static bool tick_first = true;

// previous position of turtle
static Pose prev_pos;

// previous orientation of turtle 
static Orientation prev_or;

// update state variable
static bool update_initial = true;

// Difference in X
static std::map<int, int> x_difference {
    {NORTH, 0},
    {EAST, 1},
    {SOUTH, 0},
    {WEST, -1}
};

// Difference in Y
static std::map<int, int> y_difference {
    {NORTH, -1},
    {EAST, 0},
    {SOUTH, 1},
    {WEST, 0}
};

// Turn orientation macros into strings
static std::map<int, std::string> or_string {
    {NORTH, "North"},
    {EAST, "East"},
    {SOUTH, "South"},
    {WEST, "West"}
};


void tickInterrupt(ros::Time t) {
}

/*
 * Whenever the turtle moves, compare the current location
 * to the previous location and throw an invariant violation
 * if the locations differ by more than 1 in Manhattan Distance.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (update_initial == true){
        update_initial = false;
    } else if (update_initial == false){
        if (!((prev_pos.y == y) && (prev_pos.x == x))){
            if (x == prev_pos.x + x_difference[o] &&
                y == prev_pos.y + y_difference[o] && 
                prev_or == o){
                ROS_WARN("Forward called sucessfully: \n Last Position: [%d], [%d]\n"
                         "Current Position: [%d], [%d]\n"
                         "Current Orientation %s",
                         prev_pos.x,
                         prev_pos.y,
                         x,
                         y,
                         or_string[o].c_str());
            } else {
                ROS_WARN("VIOLATION: \n Last Position: [%d], [%d]\n"
                         "Current Position: [%d], [%d]\n"
                         "Current Orientation %s",
                         prev_pos.x,
                         prev_pos.y,
                         x,
                         y,
                         or_string[o].c_str());
            }
        }
    }
    prev_pos.x = x;
    prev_pos.y = y;
    prev_or = o;
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}


void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
