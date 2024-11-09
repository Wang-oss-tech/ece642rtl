#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <stdint.h>
#include <utility>

enum turtleMove {
    MOVE_FORWARD,
    TURN_LEFT
};

enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

const int32_t START_POS = 50;

bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void incrementVisits(int32_t x, int32_t y);
int32_t getVisits(int32_t x, int32_t y);

#endif // STUDENT_MOCK_H
