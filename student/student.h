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
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Turtle movement enumeration
enum turtleMove{
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT
};

// Enum to represent direction/orientation
enum Direction {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

// Scope-preserving changes to these lines permitted (see p5 writeup)
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped);

int32_t getVisits(int32_t x, int32_t y) 
void incrementVisits(int32_t x, int32_t y)

// OK to change below this line
// bool studentMoveTurtle(QPointF& pos_, int& nw_or);