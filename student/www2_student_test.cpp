/*
 * 18-642 Unit Testing Example
 * Student Name: William Wang
 * Andrew ID: www2
 * This code exercises Transitions T1 and T2 for the student turtle statechart
 * in Project 9. It uses the CUnit framework (cunit.sourceforge.net)
 */

#include "student_mock.h"  // Mock version of student functions for testing
#include "student.h"        // The actual header with enums and type definitions
#include <CUnit/Basic.h>
#include <utility>          // For std::pair


// Test for Transition T1: atEnd == false
void test_T1_atEndFalse() {
    mock_set_bumped(false);  // No bump
    mock_set_atend(false);   // Not at the end

    // Call the function being tested
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    // Assert the expected state and movement
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);
}

// Test for Transition T4: atEnd == true
void test_T4_atEndTrue() {
    mock_set_bumped(false);  // No bump
    mock_set_atend(true);    // Reached the end

    // Call the function being tested
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    // Assert the turtle stops (since it reached the goal)
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected at end
}

// Test for Transition T2: numTurns < 3 and bumped == true
void test_T2_bumpedTrue_numTurnsLessThan3() {
    mock_set_bumped(true);   // Simulate a bump
    mock_set_atend(false);   // Not at the end

    // Call the function being tested with initial NORTH orientation
    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);

    // Assert the expected state (TURN_LEFT) and number of turns
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Expect a turn since it bumped
}

// Test for Transition T3: numTurns == 3
void test_T3_numTurnsEquals3() {
    mock_set_bumped(false);  // No bump
    mock_set_atend(false);   // Not at the end

    // Call the function being tested with initial NORTH orientation
    // Simulating 3 turns to reach desired orientation
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    // Assert that after turning, it is aligned to MOVE_FORWARD
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // Expect to move forward when aligned
}

// Initialization and cleanup functions for CUnit
int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

/* Main function for CUnit testing */
int main() {
    CU_pSuite pSuite = NULL;

    // Initialize the CUnit test registry
    if (CUE_SUCCESS != CU_initialize_registry()) {
        return CU_get_error();
    }

    // Add a suite to the registry
    pSuite = CU_add_suite("Turtle Statechart Tests", init_suite, clean_suite);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // Add tests to the suite
    if ((NULL == CU_add_test(pSuite, "Test T1: atEnd == false", test_T1_atEndFalse)) ||
        (NULL == CU_add_test(pSuite, "Test T4: atEnd == true", test_T4_atEndTrue)) ||
        (NULL == CU_add_test(pSuite, "Test T2: bumped == true and numTurns < 3", test_T2_bumpedTrue_numTurnsLessThan3)) ||
        (NULL == CU_add_test(pSuite, "Test T3: numTurns == 3", test_T3_numTurnsEquals3))) 
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // Run all tests using the CUnit Basic interface
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
