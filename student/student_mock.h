#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#ifdef testing

#include <cstdio>  // For printf

// Mock definitions for constants and enums used in testing
const int32_t START_POS = 50;

enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
};

// Mock ROS_ERROR macro to use printf in a testing environment
#define ROS_ERROR(...) printf("ERROR: " __VA_ARGS__)

// Mock function declarations
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Mock versions of functions
void mock_set_bumped(bool value);
void mock_set_atend(bool value);

// Other function prototypes from student.h for testing
int32_t getVisits(int32_t x, int32_t y);
void incrementVisits(int32_t x, int32_t y);
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or);
int translateOrnt(int orientation);
std::pair<turtleMove, int> studentTurtleStep(bool bumped, int nw_or);

#else
#include "student.h"  // Use the actual student.h if not in testing mode
#endif

#endif  // STUDENT_MOCK_H
