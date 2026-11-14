#include <math.h>

#include "fastmath.h"
#include "foc.h"
#include "hj20_config.h"
#include "hj20_test.h"

#define TWO_THIRDS_PI 2.09439510239320f

static hj20_abc_t balanced(float amp, float th)
{
    hj20_abc_t i = {amp * cosf(th), amp * cosf(th - TWO_THIRDS_PI), amp * cosf(th + TWO_THIRDS_PI)};
    return i;
}

static void clarke_balanced_set(void)
{
    for (int k = 0; k < 360; k += 5) {
        float th = (float)k * (HJ20_PI / 180.0f);
        hj20_ab_t ab = hj20_clarke(balanced(7.5f, th));
        CHECK_NEAR(ab.alpha, 7.5f * cosf(th), 1e-5);
        CHECK_NEAR(ab.beta, 7.5f * sinf(th), 1e-5);
    }
}

static void clarke_inverse_round_trip(void)
{
    hj20_ab_t v = {12.0f, -5.0f};
    hj20_ab_t back = hj20_clarke(hj20_inv_clarke(v));
    CHECK_NEAR(back.alpha, v.alpha, 1e-5);
    CHECK_NEAR(back.beta, v.beta, 1e-5);
}

static void park_inverse_park_round_trip(void)
{
    for (int k = -720; k <= 720; k += 7) {
        float th = (float)k * (HJ20_PI / 180.0f);
        float s, c;
        hj20_sincosf(th, &s, &c);
        hj20_ab_t x = {3.0f, -1.25f};
        hj20_ab_t back = hj20_inv_park(hj20_park(x, s, c), s, c);
        CHECK_NEAR(back.alpha, x.alpha, 1e-5);
        CHECK_NEAR(back.beta, x.beta, 1e-5);
    }
}

static void park_aligned_current_is_pure_q(void)
{
    /* A current vector 90 electrical degrees ahead of the rotor is all torque. */
    float th = 0.7f;
    float s, c;
    hj20_sincosf(th, &s, &c);
    hj20_ab_t ab = hj20_clarke(balanced(4.0f, th + 0.5f * HJ20_PI));
    hj20_dq_t dq = hj20_park(ab, s, c);
    CHECK_NEAR(dq.d, 0.0f, 1e-5);
    CHECK_NEAR(dq.q, 4.0f, 1e-5);
}

static void sincos_matches_libm(void)
{
    double worst = 0.0;
    for (int k = -20000; k <= 20000; k++) {
        float x = (float)k * (4.0f * HJ20_PI / 20000.0f);
        float s, c;
        hj20_sincosf(x, &s, &c);
        double es = fabs((double)s - sin((double)x));
        double ec = fabs((double)c - cos((double)x));
        if (es > worst) worst = es;
        if (ec > worst) worst = ec;
    }
    CHECK(worst < 2e-6);
}

static void svpwm_zero_vector_is_fifty_percent(void)
{
    hj20_duty_t d;
    hj20_ab_t zero = {0.0f, 0.0f};
    CHECK(!hj20_svpwm(zero, 48.0f, &d));
    CHECK_NEAR(d.a, 0.5f, 1e-6);
    CHECK_NEAR(d.b, 0.5f, 1e-6);
    CHECK_NEAR(d.c, 0.5f, 1e-6);
}

static void svpwm_linear_region_reproduces_line_voltages(void)
{
    const float vbus = 48.0f;
    const float mag = 0.9f * hj20_svpwm_linear_limit(vbus);
    for (int k = 0; k < 360; k += 3) {
        float th = (float)k * (HJ20_PI / 180.0f);
        hj20_ab_t v = {mag * cosf(th), mag * sinf(th)};
        hj20_duty_t d;
        CHECK(!hj20_svpwm(v, vbus, &d));
        hj20_abc_t ph = hj20_inv_clarke(v);
        /* Zero-sequence cancels in line-to-line voltages. */
        CHECK_NEAR((d.a - d.b) * vbus, ph.a - ph.b, 1e-3);
        CHECK_NEAR((d.b - d.c) * vbus, ph.b - ph.c, 1e-3);
    }
}

static void svpwm_duty_within_bounds(void)
{
    const float vbus = 48.0f;
    for (int m = 0; m <= 12; m++) {
        float mag = (float)m * 0.1f * hj20_svpwm_linear_limit(vbus);
        for (int k = 0; k < 360; k += 2) {
            float th = (float)k * (HJ20_PI / 180.0f);
            hj20_ab_t v = {mag * cosf(th), mag * sinf(th)};
            hj20_duty_t d;
            (void)hj20_svpwm(v, vbus, &d);
            CHECK(d.a >= HJ20_DUTY_MIN && d.a <= HJ20_DUTY_MAX);
            CHECK(d.b >= HJ20_DUTY_MIN && d.b <= HJ20_DUTY_MAX);
            CHECK(d.c >= HJ20_DUTY_MIN && d.c <= HJ20_DUTY_MAX);
        }
    }
}

static void svpwm_overmodulation_flagged(void)
{
    hj20_duty_t d;
    hj20_ab_t v = {1.2f * hj20_svpwm_linear_limit(48.0f), 0.0f};
    CHECK(hj20_svpwm(v, 48.0f, &d));
}

static void svpwm_no_bus_outputs_neutral(void)
{
    hj20_duty_t d;
    hj20_ab_t v = {10.0f, 0.0f};
    CHECK(hj20_svpwm(v, 0.0f, &d));
    CHECK_NEAR(d.a, 0.5f, 1e-6);
}

HJ20_SUITE(suite_foc, "foc",
    {TEST_NAME(REQ_FOC_TRANSFORM, "clarke_balanced_set"), clarke_balanced_set},
    {TEST_NAME(REQ_FOC_TRANSFORM, "clarke_inverse_round_trip"), clarke_inverse_round_trip},
    {TEST_NAME(REQ_FOC_TRANSFORM, "park_inverse_park_round_trip"), park_inverse_park_round_trip},
    {TEST_NAME(REQ_FOC_TRANSFORM, "park_aligned_current_is_pure_q"), park_aligned_current_is_pure_q},
    {TEST_NAME(REQ_FOC_TRANSFORM, "sincos_matches_libm"), sincos_matches_libm},
    {TEST_NAME(REQ_SVPWM, "zero_vector_is_fifty_percent"), svpwm_zero_vector_is_fifty_percent},
    {TEST_NAME(REQ_SVPWM, "linear_region_reproduces_line_voltages"), svpwm_linear_region_reproduces_line_voltages},
    {TEST_NAME(REQ_SVPWM, "duty_within_bounds"), svpwm_duty_within_bounds},
    {TEST_NAME(REQ_SVPWM, "overmodulation_flagged"), svpwm_overmodulation_flagged},
    {TEST_NAME(REQ_SVPWM, "no_bus_outputs_neutral"), svpwm_no_bus_outputs_neutral},
);
