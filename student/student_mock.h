#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

// Mock version of the bumped function
bool bumped(int x1, int y1, int x2, int y2);

// Mock version of the atend function
bool atend(int x, int y);

// Additional mock helper functions to control mock behavior
void mock_set_bumped(bool value);
void mock_set_atend(bool value);

#endif  // STUDENT_MOCK_H
