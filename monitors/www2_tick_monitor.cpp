/*
 * Name: William Wang
 * ID: www2
 *
 * 
 * Between calls to tickInterrupt, there shall be at most one call 
 * to each of poseInterrupt, visitsInterrupt, and bumpInterrupt
 */

#include <map>
#include <string>
#include "monitor_interface.h"


// tick called fist time
static bool tick_first = false;

// counter variables for each interrupts
static int pose_interrupt_cnt = 0;
static int visit_interrupt_cnt = 0;
static int bump_interrupt_cnt = 0;

// current orienation of tutle 
static Orientation curr_or;

// previous orientation of turtle 
static Orientation prev_or;


/**
 * Interrupt triggers when moveTurtle is called
 */

void tickInterrupt(ros::Time t) {
    if (tick_first == true) {
        if (bump_interrupt_cnt <= 1 && 
            visit_interrupt_cnt <= 1 && 
            pose_interrupt_cnt <= 1){
            ROS_INFO("[[%ld ns]] 'Tick' received", 
                    t.toNSec());
		} else {
			ROS_WARN("VIOLATION: 'Tick' received but"
                    "more than one poseInterrupt, visitsInterrupt,"
                    "and bumpInterrupt called\n"
                    "pose_interrupt_cnt: %d \n"
                    "visit_interrupt_cnt: %d \n"
                    "bump_interrupt_cnt: %d \n",
                    pose_interrupt_cnt, visit_interrupt_cnt, 
                    bump_interrupt_cnt);
		}
	} else {
		tick_first = true;
	}

    // reset counters
    pose_interrupt_cnt = 0;
    visit_interrupt_cnt = 0;
    bump_interrupt_cnt = 0;
}

/*
 * position interrupt when position or orientation updates
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    pose_interrupt_cnt = pose_interrupt_cnt + 1;
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    bump_interrupt_cnt = bump_interrupt_cnt + 1;
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
    visit_interrupt_cnt = visit_interrupt_cnt + 1;
}


void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
