/*
 * Name: William Wang
 * ID: www2
 *
 * Turtle turns no more than 90 degrees per simulator time step
 */

#include <map>
#include <string>
#include "monitor_interface.h"


// turn orientation macros into strings
static std::map<int, std::string> or_string {
    {NORTH, "North"},
    {EAST, "East"},
    {SOUTH, "South"},
    {WEST, "West"}
};

// Opposite illegal orientations
static std::map<int, int> opp_orientations {
    {NORTH, SOUTH}, 
    {EAST, WEST}, 
    {SOUTH, NORTH}, 
    {WEST, EAST}
};

// tick called fist time
static bool tick_first = true;

// current orienation of tutle 
static Orientation curr_or;

// previous orientation of turtle 
static Orientation prev_or;

void tickInterrupt(ros::Time t) {
    if (tick_first == true){
        prev_or = curr_or;
        tick_first = false;
    } else {
        ROS_INFO("[[%ld ns]] 'Tick' was sent. Data: orientation = %s", 
                    t.toNSec(), 
                    or_string[curr_or].c_str());

        if (opp_orientations[curr_or] == prev_or) {
            ROS_WARN("VIOLATION: Turned  %s to %s", 
                     or_string[curr_or].c_str(), 
                     or_string[prev_or].c_str());
        }
        prev_or = curr_or;
    }
}

/*
 * Whenever the turtle moves, compare the current location
 * to the previous location and throw an invariant violation
 * if the locations differ by more than 1 in Manhattan Distance.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    curr_or = o; 
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
