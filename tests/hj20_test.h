/**
 * @file hj20_test.h
 * @brief Minimal self-contained unit-test harness (no external dependencies).
 *
 * Tests are grouped into suites; the runner (test_main.c) executes every
 * suite and writes a single JUnit XML report that CI uploads as the
 * `junit-results` artifact.
 */
#ifndef HJ20_TEST_H
#define HJ20_TEST_H

#include <stdbool.h>
#include <stddef.h>

#include "req_map.h"

typedef struct {
    const char *name;
    void (*fn)(void);
} hj20_test_t;

typedef struct {
    const char *name;
    const hj20_test_t *tests;
    size_t count;
} hj20_suite_t;

void hj20_check_impl(bool ok, const char *expr, const char *file, int line);
void hj20_check_near_impl(double actual, double expected, double tol,
                          const char *ea, const char *ee, const char *file, int line);

#define CHECK(cond) hj20_check_impl((cond), #cond, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tol)                                       \
    hj20_check_near_impl((double)(actual), (double)(expected), (double)(tol),   \
                         #actual, #expected, __FILE__, __LINE__)

/** Define a suite object `var` named `label` from a list of {name, fn} entries. */
#define HJ20_SUITE(var, label, ...)                                             \
    static const hj20_test_t var##_cases[] = {__VA_ARGS__};                     \
    const hj20_suite_t var = {label, var##_cases,                               \
                              sizeof(var##_cases) / sizeof(var##_cases[0])}

#endif /* HJ20_TEST_H */
