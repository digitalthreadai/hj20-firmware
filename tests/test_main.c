/**
 * @file test_main.c
 * @brief Test runner: executes all suites, prints a summary, writes JUnit XML.
 *
 * Usage: hj20_tests [--junit <path>]
 * Exit code is 0 only when every test passes.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hj20_test.h"

extern const hj20_suite_t suite_foc;
extern const hj20_suite_t suite_pi;
extern const hj20_suite_t suite_torque;
extern const hj20_suite_t suite_encoder;
extern const hj20_suite_t suite_fault;

static const hj20_suite_t *const k_suites[] = {
    &suite_foc, &suite_pi, &suite_torque, &suite_encoder,
    &suite_fault,
};
#define N_SUITES (sizeof k_suites / sizeof k_suites[0])
#define MAX_TESTS 256

typedef struct {
    const char *suite;
    const char *name;
    bool failed;
    double seconds;
    char message[512];
} result_t;

static result_t g_results[MAX_TESTS];
static size_t g_n;
static result_t *g_cur;

void hj20_check_impl(bool ok, const char *expr, const char *file, int line)
{
    if (ok || g_cur == NULL) {
        return;
    }
    if (!g_cur->failed) { /* keep the first failure: it is the informative one */
        snprintf(g_cur->message, sizeof g_cur->message, "%s:%d: CHECK(%s) failed", file, line, expr);
    }
    g_cur->failed = true;
}

void hj20_check_near_impl(double actual, double expected, double tol,
                          const char *ea, const char *ee, const char *file, int line)
{
    double d = actual - expected;
    if (d < 0) {
        d = -d;
    }
    if (d <= tol || g_cur == NULL) {
        return;
    }
    if (!g_cur->failed) {
        snprintf(g_cur->message, sizeof g_cur->message,
                 "%s:%d: CHECK_NEAR(%s, %s) |%.9g - %.9g| = %.3g > %.3g",
                 file, line, ea, ee, actual, expected, d, tol);
    }
    g_cur->failed = true;
}

static void xml_escape(FILE *f, const char *s)
{
    for (; *s; s++) {
        switch (*s) {
        case '&': fputs("&amp;", f); break;
        case '<': fputs("&lt;", f); break;
        case '>': fputs("&gt;", f); break;
        case '"': fputs("&quot;", f); break;
        default: fputc(*s, f); break;
        }
    }
}

static int write_junit(const char *path, double total_s)
{
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "cannot write JUnit report to %s\n", path);
        return -1;
    }
    size_t total_fail = 0;
    for (size_t i = 0; i < g_n; i++) {
        total_fail += g_results[i].failed ? 1u : 0u;
    }
    fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(f, "<testsuites name=\"hj20-firmware\" tests=\"%zu\" failures=\"%zu\" errors=\"0\" time=\"%.6f\">\n",
            g_n, total_fail, total_s);

    for (size_t s = 0; s < N_SUITES; s++) {
        const char *sname = k_suites[s]->name;
        size_t n = 0, nf = 0;
        double t = 0;
        for (size_t i = 0; i < g_n; i++) {
            if (g_results[i].suite == sname) {
                n++;
                nf += g_results[i].failed ? 1u : 0u;
                t += g_results[i].seconds;
            }
        }
        fprintf(f, "  <testsuite name=\"%s\" tests=\"%zu\" failures=\"%zu\" errors=\"0\" skipped=\"0\" time=\"%.6f\">\n",
                sname, n, nf, t);
        for (size_t i = 0; i < g_n; i++) {
            const result_t *r = &g_results[i];
            if (r->suite != sname) {
                continue;
            }
            fprintf(f, "    <testcase name=\"");
            xml_escape(f, r->name);
            fprintf(f, "\" classname=\"hj20.%s\" time=\"%.6f\"", sname, r->seconds);
            if (!r->failed) {
                fprintf(f, "/>\n");
                continue;
            }
            fprintf(f, ">\n      <failure type=\"assertion\" message=\"");
            xml_escape(f, r->message);
            fprintf(f, "\">");
            xml_escape(f, r->message);
            fprintf(f, "</failure>\n    </testcase>\n");
        }
        fprintf(f, "  </testsuite>\n");
    }
    fprintf(f, "</testsuites>\n");
    fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    const char *junit = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--junit") == 0 && i + 1 < argc) {
            junit = argv[++i];
        }
    }

    size_t failures = 0;
    clock_t all0 = clock();
    for (size_t s = 0; s < N_SUITES; s++) {
        const hj20_suite_t *suite = k_suites[s];
        for (size_t t = 0; t < suite->count && g_n < MAX_TESTS; t++) {
            result_t *r = &g_results[g_n++];
            r->suite = suite->name;
            r->name = suite->tests[t].name;
            r->failed = false;
            r->message[0] = '\0';
            g_cur = r;
            clock_t t0 = clock();
            suite->tests[t].fn();
            r->seconds = (double)(clock() - t0) / CLOCKS_PER_SEC;
            g_cur = NULL;
            printf("[%s] %-8s %s\n", r->failed ? "FAIL" : " OK ", suite->name, r->name);
            if (r->failed) {
                printf("         %s\n", r->message);
                failures++;
            }
        }
    }
    double total_s = (double)(clock() - all0) / CLOCKS_PER_SEC;
    printf("\n%zu tests, %zu passed, %zu failed\n", g_n, g_n - failures, failures);

    if (junit != NULL && write_junit(junit, total_s) != 0) {
        return 2;
    }
    return failures == 0 ? 0 : 1;
}
