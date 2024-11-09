#include "student_mock.h"
#include <CUnit/Basic.h>
#include <utility>

// Test T1: atEnd == False, moving to S2 (Check functions)
void test_T1_atEndFalse() {
    // Transition T1: S1 to S2
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected
}

// Test T2: numTurns < 3, move to S3
void test_T2_numTurnsLessThan3() {
    // Setup initial state
    mock_set_atend(false);
    mock_set_bumped(false);

    // Transition T2: S2 to S3
    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Expect a turn since it bumped
}

// Test T3: numTurns == 3, move to S4
void test_T3_numTurnsEquals3() {
    // Setup to reach numTurns == 3
    mock_set_atend(false);
    mock_set_bumped(false);

    // Simulate that the number of turns reaches 3
    for (int i = 0; i < 3; i++) {
        std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
    }

    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);
}

// Test T4: atEnd == True, move to S5 (Goal STOP)
void test_T4_atEndTrue() {
    // Transition T4: S1 to S5
    mock_set_atend(true);
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected
}

// Additional Tests for Data and Branch Coverage
// Test for bumped == true and atEnd == false
void test_bumpedTrue_atEndFalse() {
    mock_set_atend(false);
    mock_set_bumped(true);

    std::pair<turtleMove, int> result = studentTurtleStep(true, EAST);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Turn left due to bump
}

// Test for bumped == false and atEnd == false, but numTurns > 0
void test_bumpedFalse_numTurnsGreaterThanZero() {
    mock_set_atend(false);
    mock_set_bumped(false);

    std::pair<turtleMove, int> result = studentTurtleStep(false, SOUTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);
}

// Test for target direction requires multiple turns
void test_multipleTurnsNeeded() {
    mock_set_atend(false);
    mock_set_bumped(false);

    std::pair<turtleMove, int> result = studentTurtleStep(false, WEST);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 1);  // Requires more than one turn
}

// Test for default case in switch (invalid direction)
void test_invalidDirection() {
    std::pair<turtleMove, int> result = studentTurtleStep(false, 999);  // Invalid direction
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);  // Default behavior
}

// Main function to run the tests
int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("TurtleTests", 0, 0);

    // Add all the tests to the suite
    CU_add_test(suite, "Test T1 atEnd False", test_T1_atEndFalse);
    CU_add_test(suite, "Test T2 numTurns < 3", test_T2_numTurnsLessThan3);
    CU_add_test(suite, "Test T3 numTurns == 3", test_T3_numTurnsEquals3);
    CU_add_test(suite, "Test T4 atEnd True", test_T4_atEndTrue);
    CU_add_test(suite, "Test Bumped True atEnd False", test_bumpedTrue_atEndFalse);
    CU_add_test(suite, "Test Bumped False, numTurns > 0", test_bumpedFalse_numTurnsGreaterThanZero);
    CU_add_test(suite, "Test Multiple Turns Needed", test_multipleTurnsNeeded);
    CU_add_test(suite, "Test Invalid Direction", test_invalidDirection);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return 0;
}
