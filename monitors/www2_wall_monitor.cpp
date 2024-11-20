// /*
//  * Name: William Wang
//  * ID: www2
//  *
//  * Turtle shall not go through walls
//  */

// #include <map>
// #include <string>
// #include "monitor_interface.h"

// // Turn orientation macros into strings
// static std::map<int, std::string> or_string {
//     {NORTH, "North"},
//     {EAST, "East"},
//     {SOUTH, "South"},
//     {WEST, "West"}
// };

// // current position
// static Pose current_position;

// // updated position
// static Pose moved_pos;

// // current orientation
// static Orientation curr_or;

// // number of walls to check
// const int num_walls = 4;

// // array for storing bump data
// static bool bump_data[num_walls];

// void tickInterrupt(ros::Time t) {
// }


// // position and orientation is updated
// void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
//     current_position.x = x;
//     current_position.y = y;
//     curr_or = o;

//     if (!((current_position.y == y) && 
//           (current_position.x == x))){
//             if (bump_data[0] == false){
//                 ROS_INFO("[[%ld ns]] Wall not detected,"
//                          "correct update at (%d,%d)", 
//                           t.toNSec(), 
//                           moved_pos.x, 
//                           moved_pos.y);
//             } else{
//                 ROS_WARN("[[%ld ns]] Moved through wall at"
//                          " (%d,%d)", 
//                           moved_pos.x, 
//                           moved_pos.y);
//             }
//             moved_pos.x = x;
//             moved_pos.y = y;
//         }
// }

// /*
//  * Empty interrupt handlers beyond this point
//  */

// void visitInterrupt(ros::Time t, int visits) {
// }


// void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
//     if (current_position.y == moved_pos.y &&
//         current_position.x == moved_pos.x){
//         bump_data[curr_or] = bumped;
//     } else{
//         for(int i = 0; i < num_walls; i++){
//             bump_data[i] = true;
//         }
//     }
// }

// void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
// }
#include "monitor_interface.h"
#include <map>
#include <string>

/* size of walls checked */
const int SIZE = 4;
/* stores current pose and moved pose */
static Pose current_pose, only_pos;
/* stores current orientation */
static Orientation current_orientation;
/* stores bump data from all 4 directions */
static bool all_data[SIZE];

/* disctionary to get strings */
static std::map<int, std::string> orientation_val {
    {WEST, "West"}, {SOUTH, "South"}, {EAST, "East"}, {NORTH, "North"}
};

// Occurs every time moveTurtle is called
void tickInterrupt(ros::Time t) {

}

// Occurs every time turtle pose and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_pose.x = x;
    current_pose.y = y;
    current_orientation = o;
    if (current_pose.x != only_pos.x || current_pose.y != only_pos.y) {
        if (all_data[o] == false) {
            ROS_INFO("[[%ld ns]] 'Wall' not present, moved correctly at (%d,%d)", t.toNSec(), only_pos.x, only_pos.y);
        } else {
            ROS_WARN("Violation: Moved through a wall at (%d,%d)", only_pos.x, only_pos.y);
        }
        only_pos.x = x;
        only_pos.y = y;   
    }
}

// Occurs every time the visit count at the current location is updated
void visitInterrupt(ros::Time t, int visits) {

}

// Occurs every time a call to bumped(x1,y1,x2,y2) returns
// (t is the time of the server request, not answer)
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (current_pose.x == only_pos.x && current_pose.y == only_pos.y) {
        all_data[current_orientation] = bumped;
    } else {
    	/* resets all values */
        for (int i = 0 ; i < SIZE; i++) {
        	all_data[i] = true;
    	}
    }

}

// Occurs every time a call to atend(x,y) returns
// (t is the time of the server request, not answer)
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}