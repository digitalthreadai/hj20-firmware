#include "fastmath.h"
#include "hj20_config.h"
#include "hj20_test.h"
#include "pi.h"

/* Phase R-L model matching the gains in src/control.c. */
#define R_OHM 0.45f
#define L_H   0.35e-3f
#define KP    4.4f
#define KI    (4.4f * 1285.0f)

static void current_loop_step_response(void)
{
    hj20_pi_t pi;
    const float vmax = 25.0f;
    hj20_pi_init(&pi, KP, KI, -vmax, vmax);

    const float dt = HJ20_CURRENT_LOOP_DT_S;
    const float ref = 5.0f;
    float i = 0.0f, peak = 0.0f;
    int settled_at = -1;
    for (int k = 0; k < 40; k++) {           /* 2 ms */
        float v = hj20_pi_step(&pi, ref - i, dt);
        i += dt / L_H * (v - R_OHM * i);
        if (i > peak) peak = i;
        if (settled_at < 0 && hj20_absf(i - ref) < 0.02f * ref) settled_at = k;
    }
    CHECK(settled_at >= 0 && settled_at < 20);   /* within 1 ms */
    CHECK(peak < 1.05f * ref);                   /* < 5 % overshoot */
    CHECK_NEAR(i, ref, 0.01f * ref);             /* no steady-state error */
}

static void pi_output_respects_limits(void)
{
    hj20_pi_t pi;
    hj20_pi_init(&pi, 10.0f, 100.0f, -2.0f, 3.0f);
    CHECK_NEAR(hj20_pi_step(&pi, 100.0f, 1e-3f), 3.0f, 1e-6);
    CHECK_NEAR(hj20_pi_step(&pi, -100.0f, 1e-3f), -2.0f, 1e-6);
}

static void pi_integrator_bounded_under_saturation(void)
{
    hj20_pi_t pi;
    hj20_pi_init(&pi, 1.0f, 100.0f, -1.0f, 1.0f);
    for (int k = 0; k < 10000; k++) {
        (void)hj20_pi_step(&pi, 5.0f, 1e-4f);
    }
    CHECK(pi.integ <= 1.0f && pi.integ >= -1.0f);
}

static void pi_reset_clears_integrator(void)
{
    hj20_pi_t pi;
    hj20_pi_init(&pi, 1.0f, 100.0f, -1.0f, 1.0f);
    (void)hj20_pi_step(&pi, 0.5f, 1e-3f);
    hj20_pi_reset(&pi);
    CHECK_NEAR(pi.integ, 0.0f, 0.0);
}

static void dq_limit_prioritises_d_axis(void)
{
    float vd = 3.0f, vq = 30.0f;
    hj20_dq_limit(&vd, &vq, 10.0f);
    CHECK_NEAR(vd, 3.0f, 1e-6);
    CHECK_NEAR(vd * vd + vq * vq, 100.0f, 1e-3);
    CHECK(vq > 0.0f);

    float vd2 = 2.0f, vq2 = 1.0f;     /* inside the circle: untouched */
    hj20_dq_limit(&vd2, &vq2, 10.0f);
    CHECK_NEAR(vd2, 2.0f, 0.0);
    CHECK_NEAR(vq2, 1.0f, 0.0);
}

HJ20_SUITE(suite_pi, "pi",
    {TEST_NAME(REQ_CURRENT_LOOP, "step_response_settles_within_1ms"), current_loop_step_response},
    {TEST_NAME(REQ_CURRENT_LOOP, "output_respects_limits"), pi_output_respects_limits},
    {TEST_NAME(REQ_CURRENT_LOOP, "integrator_bounded_under_saturation"), pi_integrator_bounded_under_saturation},
    {TEST_NAME(REQ_CURRENT_LOOP, "voltage_vector_limit_prioritises_d_axis"), dq_limit_prioritises_d_axis},
    {"test_pi_reset_clears_integrator", pi_reset_clears_integrator},
);
