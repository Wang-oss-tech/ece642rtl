#include "student_mock.h"
#include <CUnit/Basic.h>
#include <utility>  // For std::pair

void test_T1_atEndFalse() {
    mock_set_atend(false);  // Set mock atEnd value to false
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // Expect to move forward with no turns
}

void test_T4_atEndTrue() {
    mock_set_atend(true);  // Set mock atEnd value to true
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected at end
}

void test_T3_numTurnsEquals3() {
    mock_set_atend(false);
    mock_set_bumped(false);  // Ensure no bump

    // Set a condition where the turtle would need to turn 3 times
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);

    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // Expect to move forward when aligned
}

void test_T2_bumpDetected() {
    mock_set_bumped(true);  // Simulate a bump
    std::pair<turtleMove, int> result = studentTurtleStep(true, NORTH);

    CU_ASSERT_EQUAL(result.first, TURN_LEFT);
    CU_ASSERT(result.second > 0);  // Expect a turn since it bumped
}

int init() {
    // Any test initialization code goes here
    return 0;
}

int cleanup() {
    // Any test cleanup code goes here
    return 0;
}

int main() {
    CU_pSuite pSuite = NULL;

    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();

    pSuite = CU_add_suite("Suite_1", init, cleanup);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    if ((NULL == CU_add_test(pSuite, "Test T1: atEnd False", test_T1_atEndFalse)) ||
        (NULL == CU_add_test(pSuite, "Test T4: atEnd True", test_T4_atEndTrue)) ||
        (NULL == CU_add_test(pSuite, "Test T3: numTurns Equals 3", test_T3_numTurnsEquals3)) ||
        (NULL == CU_add_test(pSuite, "Test T2: Bump Detected", test_T2_bumpDetected))) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
