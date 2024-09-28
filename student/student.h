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

// Turtle movement enumeration
enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT
};

// Function declarations for translating position and orientation
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or);
int translateOrnt(int orientation, turtleMove nextMove);

// Function declaration for the turtle step logic
turtleMove studentTurtleStep(bool bumped);

// Visit tracking functions to be used between turtle and maze
// You can declare them here or let them be fully managed in student_turtle.cpp if you prefer encapsulation there.
int32_t getVisits(int32_t x, int32_t y);       // Get the current visit count
void incrementVisits(int32_t x, int32_t y);         // Increment the visit count when a new cell is entered

// You might want to expose these functions only in student_turtle.cpp
// OK to change below this line
