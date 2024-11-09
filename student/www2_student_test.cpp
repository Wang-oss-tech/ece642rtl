#include "student_mock.h"
#include <CUnit/Basic.h>
#include <utility>

void test_T1_atEndFalse() {
    std::pair<turtleMove, int> result = studentTurtleStep(false, NORTH);
    CU_ASSERT_EQUAL(result.first, MOVE_FORWARD);
    CU_ASSERT_EQUAL(result.second, 0);  // No turns expected
}

// Other tests follow similar patterns

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("TurtleTests", 0, 0);

    CU_add_test(suite, "Test T1 atEnd False", test_T1_atEndFalse);

    CU_basic_run_tests();
    CU_cleanup_registry();
    return 0;
}
