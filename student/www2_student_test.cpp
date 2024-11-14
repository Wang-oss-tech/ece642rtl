/*

STUDENT NAME: William Wang
ANDREW ID: www2   

*/
#include "student_mock.h"
#include <CUnit/Basic.h>
#include <utility>
#include <stdio.h>  

// Test T1: atEnd == False, moving to S2 (Check functions)
void test_T1_atEndFalse() {
    // Transition T1: S1 to S2
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    // Expected Outcome: Turtle should move forward (MOVE_FORWARD) with no turns
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected
}

// Test T2: numTurns < 3, move to S3
void test_T2_numTurnsLessThan3() {
    // Setup initial state (atEnd, bumped = false)
    mock_set_atend(false);
    mock_set_bumped(false);

    // Transition T2: S2 to S3
    // turtle turns left due to bump with positive turn count
    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Expect a turn since it bumped
}

// Test T3: numTurns == 3, move to S4
void test_T3_numTurnsEquals3() {
    mock_set_atend(false);
    mock_set_bumped(false);
    mock_set_numTurns(0);


    for (int i = 0; i < 3; i++) {
        std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    }

    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);

    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT_EQUAL(result.second, 3);
}

// Test T4: atEnd == True
void test_T4_atEndTrue() {
    // Set goal state conditions
    mock_set_atend(true);
    mock_set_bumped(false);  // Ensure bumped isn’t interfering
    
    // Set numTurns to zero if needed to avoid interference
    mock_set_numTurns(0);  

    std::pair<turtleMove, int> result = studentTurtleStep(false, EAST);

    // Verify that we reach the goal state and stop with MOVE_FORWARD and no turns
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);
}


// Additional Tests for Data Coverage
// Test for bumped == true and atEnd == false
void test_bumpedTrue_atEndFalse() {
    mock_set_atend(false);
    mock_set_bumped(true);

    // Expected outcome: turtle should turn elft due to bump with positive turn count
    std::pair<turtleMove, int> result = studentTurtleStep(true, EAST);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Turn left due to bump
}

// Test for bumped == false and atEnd == false, but numTurns > 0
void test_bumpedFalse_numTurnsGreaterThanZero() {
    mock_set_atend(false);
    mock_set_bumped(false);

    // Expected outcome: should continute turning left
    std::pair<turtleMove, int> result = studentTurtleStep(false, EAST);

    printf("\n\nstudent turtle step: move = %d, number of turns = %d\n\n\n",
           result.first, result.second);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);
}

// Test for target direction requires multiple turns
void test_multipleTurnsNeeded() {
    mock_set_atend(false);
    mock_set_bumped(false);

    // Expected outcome: turtle turns left and requires more than one turn
    std::pair<turtleMove, int> result = studentTurtleStep(false, WEST);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 1);  // Requires more than one turn
}

// Test for default case in switch (invalid direction)
void test_invalidDirection() {
    std::pair<turtleMove, int> result = studentTurtleStep(false, 999);  // Invalid direction

    // Expected out come: turtle defaults to moving forward
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);  // Default behavior
}

// Test for multiple bumps without moving
void test_multipleBumpsWithoutMoving() {
    mock_set_atend(false);
    mock_set_bumped(true);
    mock_set_numTurns(0);

    for (int i = 0; i < 5; i++) {
        std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);
        CU_ASSERT_EQUAL(result.first, TURN_LEFT);
        CU_ASSERT(result.second > 0);  // Turtle keeps turning due to bump
    }
}

// Test moving to a wall (multiple attempts)
void test_moveToWall() {
    mock_set_atend(false);
    mock_set_bumped(true);
    mock_set_numTurns(0);

    for (int i = 0; i < 4; i++) {
        std::pair<turtleMove, int> result = studentTurtleStep(true, SOUTH);
        CU_ASSERT_EQUAL(result.first, TURN_LEFT);
        CU_ASSERT(result.second > 0);  // Keeps turning due to bump
    }
}

// Test random direction after several turns
void test_randomDirectionAfterTurns() {
    mock_set_atend(false);
    mock_set_bumped(false);
    mock_set_numTurns(2);

    // After some turns, try moving in a different direction
    std::pair<turtleMove, int> result = studentTurtleStep(false, WEST);
    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);
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

    // New data coverage tests
    CU_add_test(suite, "Test Multiple Bumps Without Moving", test_multipleBumpsWithoutMoving);
    CU_add_test(suite, "Test Move to Wall", test_moveToWall);
    CU_add_test(suite, "Test Random Direction After Turns", test_randomDirectionAfterTurns);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return 0;
}
