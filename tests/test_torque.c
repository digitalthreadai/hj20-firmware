#include "fastmath.h"
#include "hj20_config.h"
#include "hj20_test.h"
#include "motion.h"
#include "torque.h"

static void torque_to_iq_uses_ratio_and_efficiency(void)
{
    /* 20 Nm / (0.08 Nm/A * 100 * 0.80) = 3.125 A */
    CHECK_NEAR(hj20_torque_to_iq(20.0f), 3.125f, 1e-5);
    CHECK_NEAR(hj20_iq_to_torque(hj20_torque_to_iq(-13.7f)), -13.7f, 1e-4);
}

static void torque_clamp_at_peak(void)
{
    CHECK_NEAR(hj20_torque_clamp(80.0f, 100.0f), HJ20_PEAK_TORQUE_NM, 0.0);
    CHECK_NEAR(hj20_torque_clamp(-80.0f, 100.0f), -HJ20_PEAK_TORQUE_NM, 0.0);
    CHECK_NEAR(hj20_torque_clamp(42.0f, HJ20_PEAK_TORQUE_NM), 42.0f, 0.0);
}

static void torque_clamp_at_requested_limit(void)
{
    CHECK_NEAR(hj20_torque_clamp(35.0f, HJ20_RATED_TORQUE_NM), HJ20_RATED_TORQUE_NM, 0.0);
    CHECK_NEAR(hj20_torque_clamp(10.0f, -5.0f), 0.0f, 0.0);  /* negative limit = no torque */
}

static void peak_torque_current_below_trip(void)
{
    /* Design consistency: peak torque must be reachable without tripping OC. */
    CHECK(hj20_torque_to_iq(HJ20_PEAK_TORQUE_NM) < 0.6f * HJ20_PHASE_OC_TRIP_A);
}

static void velocity_loop_output_within_torque_limit(void)
{
    hj20_motion_t m;
    hj20_motion_init(&m);
    for (int k = 0; k < 2000; k++) {
        float t = hj20_motion_velocity(&m, 3.0f, -3.0f, 12.5f, HJ20_SLOW_LOOP_DT_S);
        CHECK(t <= 12.5f && t >= -12.5f);
    }
}

static void position_loop_velocity_clamped_to_rated_speed(void)
{
    hj20_motion_t m;
    hj20_motion_init(&m);
    float v = hj20_motion_position(&m, 3.0f, 0.0f, 0.0f);
    CHECK_NEAR(v, HJ20_MAX_SPEED_RAD_S, 1e-6);
    v = hj20_motion_position(&m, -3.0f, 0.0f, 0.0f);
    CHECK_NEAR(v, -HJ20_MAX_SPEED_RAD_S, 1e-6);
    CHECK_NEAR(HJ20_MAX_SPEED_RAD_S, 3.14159265, 1e-5);  /* 30 rpm */
}

static void velocity_reference_clamped_to_rated_speed(void)
{
    hj20_motion_t m;
    hj20_motion_init(&m);
    /* A 100 rad/s request while already at 30 rpm must produce ~no torque. */
    float t = hj20_motion_velocity(&m, 100.0f, HJ20_MAX_SPEED_RAD_S, HJ20_PEAK_TORQUE_NM, HJ20_SLOW_LOOP_DT_S);
    CHECK_NEAR(t, 0.0f, 1e-3);
}

static void position_error_takes_shortest_path(void)
{
    hj20_motion_t m;
    hj20_motion_init(&m);
    /* ref just past +pi, measurement just short of it: small positive error. */
    float v = hj20_motion_position(&m, -3.10f, 3.10f, 0.0f);
    CHECK(v > 0.0f && v < 3.0f);
}

HJ20_SUITE(suite_torque, "torque",
    {TEST_NAME(REQ_TORQUE_LIMIT, "torque_to_iq_uses_ratio_and_efficiency"), torque_to_iq_uses_ratio_and_efficiency},
    {TEST_NAME(REQ_TORQUE_LIMIT, "torque_clamp_at_peak"), torque_clamp_at_peak},
    {TEST_NAME(REQ_TORQUE_LIMIT, "torque_clamp_at_requested_limit"), torque_clamp_at_requested_limit},
    {TEST_NAME(REQ_TORQUE_LIMIT, "peak_torque_current_below_trip"), peak_torque_current_below_trip},
    {TEST_NAME(REQ_TORQUE_LIMIT, "velocity_loop_output_within_torque_limit"), velocity_loop_output_within_torque_limit},
    {TEST_NAME(REQ_SPEED_LIMIT, "position_loop_velocity_clamped_to_rated_speed"), position_loop_velocity_clamped_to_rated_speed},
    {TEST_NAME(REQ_SPEED_LIMIT, "velocity_reference_clamped_to_rated_speed"), velocity_reference_clamped_to_rated_speed},
    {"test_position_error_takes_shortest_path", position_error_takes_shortest_path},
);
