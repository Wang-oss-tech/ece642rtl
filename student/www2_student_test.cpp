/* 
 * STUDENT NAME: William Wang
 * ANDREW ID: www2
 * LAST UPDATE: 9 Nov 2024
 */

#include "student_mock.h"
#include <CUnit/Basic.h>

// Test Cases for Turtle Movements

void test_T1_atEndFalse() {
    mock_set_atend(false);
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);
}

void test_T4_atEndTrue() {
    mock_set_atend(true);
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0); // No turns expected at end
}

int init() {
    // Any test initialization code goes here
    return 0;
}

int cleanup() {
    // Any test cleanup code goes here
    return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {

    CU_pSuite pSuite = NULL;

    /* initialize the CUnit test registry */
    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();

    /* add a suite to the registry */
    pSuite = CU_add_suite("Turtle_Test_Suite", init, cleanup);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* add the tests to the suite */
    if ((NULL == CU_add_test(pSuite, "test of atEnd false", test_T1_atEndFalse)) ||
        (NULL == CU_add_test(pSuite, "test of atEnd true", test_T4_atEndTrue))) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* Run all tests using the CUnit Basic interface */
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
