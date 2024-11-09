/*
 * www2_student_test.cpp
 * STUDENT NAME: William Wang
 * ANDREW ID: www2
 * LAST UPDATE: 9/8/2024
 *
 * Unit tests for student_turtle using CUnit framework.
 */

#include "student_mock.h"
#include <CUnit/Basic.h>

// Mock functions for test setup
void mock_set_bumped(bool value);
void mock_set_atend(bool value);

// Test T1: Transition when atEnd is false
void test_T1_atEndFalse() {
    mock_set_atend(false);
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    
    // Check that the turtle stays in STATE_MOVE_FORWARD if it hasn't reached the end
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0); // No turns expected
}

// Test T2: Transition when numTurns < 3
void test_T2_numTurnsLessThan3() {
    mock_set_bumped(true);
    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
    
    // Check that the turtle turns if bumped and numTurns < 3
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0); // Expect a turn
}

// Test T3: Transition when numTurns == 3
void test_T3_numTurnsEquals3() {
    mock_set_bumped(false);
    // Simulate that numTurns has reached 3 (setup mock or modify state if possible)
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    
    // Verify the turtle moves forward when numTurns reaches 3
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0); // No turns expected
}

// Test T4: Transition when atEnd is true
void test_T4_atEndTrue() {
    mock_set_atend(true);
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    // Check that the turtle stops moving when it reaches the end
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0); // No turns expected, end reached
}

// Initialization function for test setup
int init_suite(void) {
    return 0;
}

// Cleanup function for test suite
int clean_suite(void) {
    return 0;
}

// Main function to run the CUnit tests
int main() {
    CU_pSuite pSuite = NULL;

    // Initialize the CUnit test registry
    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();

    // Add a suite to the registry
    pSuite = CU_add_suite("Student Turtle Tests", init_suite, clean_suite);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // Add tests to the suite
    if ((NULL == CU_add_test(pSuite, "Test T1: atEnd = false", test_T1_atEndFalse)) ||
        (NULL == CU_add_test(pSuite, "Test T2: numTurns < 3", test_T2_numTurnsLessThan3)) ||
        (NULL == CU_add_test(pSuite, "Test T3: numTurns == 3", test_T3_numTurnsEquals3)) ||
        (NULL == CU_add_test(pSuite, "Test T4: atEnd = true", test_T4_atEndTrue))) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // Run all tests using the CUnit Basic interface
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
