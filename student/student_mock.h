#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <stdint.h>
#include <utility>

// Enumeration for turtle movement actions
enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT
};

// Enumeration for directions
enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

// Starting position constant
const int32_t START_POS = 50;

// Mock functions for testing
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void incrementVisits(int32_t x, int32_t y);
int32_t getVisits(int32_t x, int32_t y);

// Functions to set mock behavior for bumped and atend
void mock_set_bumped(bool value);
void mock_set_atend(bool value);
void mock_set_numTurns(int value);
bool mock_get_bumped();
bool mock_get_atend();
int mock_get_numTurns();


// Function under test
std::pair<turtleMove, int> studentTurtleStep(bool bumped, int nw_or);

#endif // STUDENT_MOCK_H
