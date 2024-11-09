/* 
 * STUDENT NAME: William Wang
 * ANDREW ID: www2
 * LAST UPDATE: 9 Nov 2024
 */

#include <iostream>
#include <stdint.h>
#include <utility>
#include <QPointF>  // Ensure Qt is properly included

// Mock variables to store values for testing
static bool mock_bumped_value = false;
static bool mock_atend_value = false;

// Mock implementation of bumped()
bool bumped(int x1, int y1, int x2, int y2) {
    return mock_bumped_value;
}

// Mock implementation of atend()
bool atend(int x, int y) {
    return mock_atend_value;
}

// Mock helper functions to set the values of bumped and atend
void mock_set_bumped(bool value) {
    mock_bumped_value = value;
}

void mock_set_atend(bool value) {
    mock_atend_value = value;
}

// Other functions needed for the test
int32_t getVisits(int32_t x, int32_t y);
void incrementVisits(int32_t x, int32_t y);
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or);
int translateOrnt(int orientation);
std::pair<turtleMove, int> studentTurtleStep(bool bumped, int nw_or);

// Turtle movement enumeration
enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
};

// Enum to represent direction/orientation
enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

// Constants
const int32_t START_POS = 50; // starting position in center of 23x23 array

// Static variables for test purposes
static int32_t visitMap[23][23] = {0}; // Visit map to track number of visits

int32_t getVisits(int32_t x, int32_t y) {
    if (x >= 0 && x < 23 && y >= 0 && y < 23) {
        return visitMap[x][y];
    } else {
        std::cerr << "Invalid coordinates accessed in visitMap." << std::endl;
        return 0;
    }
}

void incrementVisits(int32_t x, int32_t y) {
    if (x >= 0 && x < 23 && y >= 0 && y < 23) {
        visitMap[x][y]++;
    } else {
        std::cerr << "Invalid coordinates accessed in visitMap during increment." << std::endl;
    }
}
