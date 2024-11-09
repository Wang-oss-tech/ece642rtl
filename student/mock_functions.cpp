#include "student_mock.h"

// Static variables to control mock return values
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

// Function to set the mock value for bumped
void mock_set_bumped(bool value) {
    mock_bumped_value = value;
}

// Function to set the mock value for atend
void mock_set_atend(bool value) {
    mock_atend_value = value;
}
