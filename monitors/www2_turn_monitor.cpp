/*
 * Code by Milda Zizyte
 *
 * This monitor checks that the invariant "turtle shall not move more
 * than on square at a time" is not violated.
 * It keeps track of the previous position of the turtle and compares it
 * to the current position to check the invariant.
 */

#include "monitor_interface.h"

// Keeps track of the last orientation received
static Orientation last_orientation;
static bool initialized = false;  // Tracks if this is the first orientation received


// Helper function to calculate the absolute difference between two orientations
int orientationDifference(Orientation a, Orientation b) {
    int diff = abs(a - b);
    // Adjust if the difference is more than half a circle (wrap-around case)
    return diff > 180 ? 360 - diff : diff;
}

/*
 * This interrupt checks if the turtle's orientation change exceeds 90 degrees
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation current_orientation) {
    // Print current orientation info
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Orientation: %d", t.toNSec(), current_orientation);

    // If this is the first pose received, initialize last_orientation and return
    if (!initialized) {
        last_orientation = current_orientation;
        initialized = true;
        return;
    }

    // Calculate the orientation difference
    int orientation_diff = orientationDifference(last_orientation, current_orientation);

    // Check if the orientation difference exceeds 90 degrees
    if (orientation_diff > 90) {
        ROS_WARN("VIOLATION: Orientation change from %d to %d exceeds 90 degrees!", last_orientation, current_orientation);
    }

    // Update last_orientation for the next tick
    last_orientation = current_orientation;
}

/*
 * Empty interrupt handlers beyond this point
 */
void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
