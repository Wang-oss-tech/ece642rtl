// student_mock.h

#pragma once

#include <utility>

// Mock variables
extern bool mock_bumped_value;
extern bool mock_atend_value;

// Mock function definitions
inline bool bumped(int x1, int y1, int x2, int y2) {
    return mock_bumped_value;
}

inline bool atend(int x, int y) {
    return mock_atend_value;
}

inline void mock_set_bumped(bool value) {
    mock_bumped_value = value;
}

inline void mock_set_atend(bool value) {
    mock_atend_value = value;
}
