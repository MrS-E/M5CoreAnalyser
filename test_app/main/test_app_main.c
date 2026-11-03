/**
 * @file test_app_main.c
 * @brief Unity test runner entry point for M5CoreAnalyser component test suites.
 */

#include <stdio.h>
#include "unity.h"
#include "unity_test_runner.h"
#include "esp_system.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void app_main(void)
{
    printf("\n=== M5CoreAnalyser Unity Component Test Runner ===\n");
    UNITY_BEGIN();
    unity_run_all_tests();
    int failures = UNITY_END();
    printf("=== Unity Tests Complete (failures=%d) ===\n", failures);
}
