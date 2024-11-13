#include "student_mock.h"

// Static variables to control mock return values
static bool mock_bumped_value = false;
static bool mock_atend_value = false;
static int mock_numTurns_value = 0;  // New static variable for numTurns

// Mock implementation of bumped()
bool bumped(int x1, int y1, int x2, int y2) {
    return mock_bumped_value;
}

// Mock implementation of atend()
bool atend(int x, int y) {
    return mock_atend_value;
}

// Function to set the mock value for bumped
void mock_set_bumped(bool value) {
    mock_bumped_value = value;
}

// Function to set the mock value for atend
void mock_set_atend(bool value) {
    mock_atend_value = value;
}

// New function to set the mock value for numTurns
void mock_set_numTurns(int value) {
    mock_numTurns_value = value;
}

// Getter functions for debugging purposes
bool mock_get_bumped() {
    return mock_bumped_value;
}

bool mock_get_atend() {
    return mock_atend_value;
}

int mock_get_numTurns() {
    return mock_numTurns_value;
}
