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

/* Drive the derater at 1 kHz as the slow loop does, commanding `want` Nm. */
static float run_derate(hj20_derate_t *d, float want, int steps, float t_stage, float t_motor, float *last_cmd)
{
    float lim = HJ20_PEAK_TORQUE_NM;
    for (int k = 0; k < steps; k++) {
        float cmd = hj20_torque_clamp(want, lim);
        lim = hj20_derate_limit(d, HJ20_PEAK_TORQUE_NM, cmd, t_stage, t_motor, HJ20_SLOW_LOOP_DT_S);
        *last_cmd = cmd;
    }
    return lim;
}

static void peak_folds_back_to_rated_after_two_seconds(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    float cmd = 0.0f;
    float lim = HJ20_PEAK_TORQUE_NM;
    int fold_at = -1;
    for (int k = 0; k < 3000 && fold_at < 0; k++) {
        cmd = hj20_torque_clamp(50.0f, lim);
        lim = hj20_derate_limit(&d, HJ20_PEAK_TORQUE_NM, cmd, 40.0f, 40.0f, HJ20_SLOW_LOOP_DT_S);
        if (lim < HJ20_PEAK_TORQUE_NM) {
            fold_at = k + 1;
        }
    }
    CHECK(fold_at >= 1990 && fold_at <= 2010);   /* 2 s at 50 Nm */
    CHECK_NEAR(lim, HJ20_RATED_TORQUE_NM, 0.0);
}

static void rated_torque_is_continuous(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    float cmd;
    float lim = run_derate(&d, HJ20_RATED_TORQUE_NM, 60000, 40.0f, 40.0f, &cmd);   /* 60 s */
    CHECK_NEAR(lim, HJ20_PEAK_TORQUE_NM, 0.0);
    CHECK(!d.folded_back);
}

static void foldback_recovers_below_rated(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    float cmd;
    (void)run_derate(&d, 50.0f, 2100, 40.0f, 40.0f, &cmd);
    CHECK(d.folded_back);
    /* At rated torque nothing recovers ... */
    CHECK_NEAR(run_derate(&d, 20.0f, 10000, 40.0f, 40.0f, &cmd), HJ20_RATED_TORQUE_NM, 0.0);
    /* ... at zero torque half the budget returns in 2100 / 400 = 5.25 s. */
    CHECK_NEAR(run_derate(&d, 0.0f, 5000, 40.0f, 40.0f, &cmd), HJ20_RATED_TORQUE_NM, 0.0);
    CHECK_NEAR(run_derate(&d, 0.0f, 400, 40.0f, 40.0f, &cmd), HJ20_PEAK_TORQUE_NM, 0.0);
}

static void stage_temperature_derates_linearly(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    CHECK_NEAR(hj20_derate_limit(&d, 40.0f, 0.0f, 85.0f, 40.0f, HJ20_SLOW_LOOP_DT_S), 40.0f, 1e-5);
    CHECK_NEAR(hj20_derate_limit(&d, 40.0f, 0.0f, 95.0f, 40.0f, HJ20_SLOW_LOOP_DT_S), 20.0f, 1e-4);
    CHECK_NEAR(hj20_derate_limit(&d, 40.0f, 0.0f, 105.0f, 40.0f, HJ20_SLOW_LOOP_DT_S), 0.0f, 0.0);
    /* Derating reaches zero before the 110 degC trip. */
    CHECK(HJ20_TEMP_STAGE_DERATE_END_C < HJ20_TEMP_STAGE_TRIP_C);
}

static void winding_temperature_derates(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    CHECK_NEAR(hj20_derate_limit(&d, 50.0f, 0.0f, 40.0f, 112.5f, HJ20_SLOW_LOOP_DT_S), 25.0f, 1e-4);
    CHECK(HJ20_TEMP_MOTOR_DERATE_END_C < HJ20_TEMP_MOTOR_TRIP_C);
}

static void derated_limit_never_exceeds_request(void)
{
    hj20_derate_t d;
    hj20_derate_init(&d);
    CHECK_NEAR(hj20_derate_limit(&d, 12.0f, 0.0f, 25.0f, 25.0f, HJ20_SLOW_LOOP_DT_S), 12.0f, 0.0);
    CHECK_NEAR(hj20_derate_limit(&d, 80.0f, 0.0f, 25.0f, 25.0f, HJ20_SLOW_LOOP_DT_S), HJ20_PEAK_TORQUE_NM, 0.0);
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
    {TEST_NAME(REQ_TORQUE_DERATE, "peak_folds_back_to_rated_after_two_seconds"), peak_folds_back_to_rated_after_two_seconds},
    {TEST_NAME(REQ_TORQUE_DERATE, "rated_torque_is_continuous"), rated_torque_is_continuous},
    {TEST_NAME(REQ_TORQUE_DERATE, "foldback_recovers_below_rated"), foldback_recovers_below_rated},
    {TEST_NAME(REQ_TORQUE_DERATE, "stage_temperature_derates_linearly"), stage_temperature_derates_linearly},
    {TEST_NAME(REQ_TORQUE_DERATE, "winding_temperature_derates"), winding_temperature_derates},
    {TEST_NAME(REQ_TORQUE_DERATE, "derated_limit_never_exceeds_request"), derated_limit_never_exceeds_request},
);
