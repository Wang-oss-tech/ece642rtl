#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted (see p5 writeup)
enum turtleMove {
    MOVE_FORWARD,  // Move forward command
    TURN_LEFT,     // Turn left command
    TURN_RIGHT     // Turn right command
};

// Function declarations
QPointF translatePos(QPointF pos_, int orientation, turtleMove nextMove);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped);

// OK to change below this line
bool studentMoveTurtle(QPointF& pos_, int& nw_or);

// Declaration for the visit tracking functions
void incrementVisits(int32_t x, int32_t y);  // Increment visit count
int32_t getVisits(int32_t x, int32_t y);     // Get visit count
