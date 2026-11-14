#include <stdint.h>

#include "encoder.h"
#include "fastmath.h"
#include "hj20_config.h"
#include "hj20_test.h"

/* Output-encoder reading that is exactly consistent with motor counts `acc`. */
static uint32_t output_raw_for(int64_t acc, uint32_t offset_counts)
{
    int64_t n = HJ20_MOTOR_COUNTS_PER_OUTPUT_REV;
    int64_t a = ((acc % n) + n) % n;
    return (uint32_t)((a * HJ20_OUTPUT_COUNTS_PER_REV) / n + offset_counts) & (HJ20_OUTPUT_COUNTS_PER_REV - 1);
}

static void counts_to_angle(void)
{
    CHECK_NEAR(hj20_counts_to_rad(0, 14), 0.0f, 0.0);
    CHECK_NEAR(hj20_counts_to_rad(8192, 14), HJ20_PI, 1e-6);
    CHECK_NEAR(hj20_counts_to_rad(16384 + 4096, 14), 0.5f * HJ20_PI, 1e-6); /* masked */
    CHECK_NEAR(hj20_counts_to_rad(1u << 18, 19), HJ20_PI, 1e-6);
}

static void motor_wrap_accumulates_across_rev_boundary(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    (void)hj20_encoder_motor_update(&e, 16300);
    (void)hj20_encoder_motor_update(&e, 16380);
    (void)hj20_encoder_motor_update(&e, 40);     /* crossed zero going forward */
    CHECK(e.acc == 16384 + 40);
    (void)hj20_encoder_motor_update(&e, 16350);  /* and back again */
    CHECK(e.acc == 16350);
}

static void output_from_motor_follows_gear_ratio(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    uint16_t raw = 0;
    (void)hj20_encoder_motor_update(&e, raw);
    /* 50 motor revolutions forward = half an output revolution. */
    for (int k = 0; k < 50 * 16384 / 64; k++) {
        raw = (uint16_t)((raw + 64u) & 0x3FFFu);
        (void)hj20_encoder_motor_update(&e, raw);
    }
    CHECK_NEAR(hj20_encoder_output_from_motor(&e), HJ20_PI, 1e-5);
}

static void electrical_angle_uses_pole_pairs(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    /* 1/28 of a mechanical turn = half an electrical turn with 14 pole pairs. */
    float th = hj20_encoder_motor_update(&e, (uint16_t)(16384 / 28));
    CHECK_NEAR(th, HJ20_PI, 2e-3);
}

static void consistent_encoders_raise_no_mismatch(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    int64_t acc = 1234;
    uint16_t raw = (uint16_t)acc;
    bool any = false;
    for (int tick = 0; tick < 3000; tick++) {
        for (int k = 0; k < 20; k++) {          /* 20 PWM periods per slow tick */
            acc += 13;
            raw = (uint16_t)((raw + 13u) & 0x3FFFu);
            (void)hj20_encoder_motor_update(&e, raw);
        }
        any |= hj20_encoder_slow_update(&e, output_raw_for(acc, 777), 0.0f, HJ20_SLOW_LOOP_DT_S);
    }
    CHECK(!any);
    CHECK(e.mismatch_count == 0);
}

static void mismatch_detected_after_debounce(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    (void)hj20_encoder_motor_update(&e, 100);
    CHECK(!hj20_encoder_slow_update(&e, output_raw_for(100, 0), 0.0f, HJ20_SLOW_LOOP_DT_S));

    /* Output flange reads 2 degrees off (a slipped flexspline tooth). */
    uint32_t skew = (uint32_t)(2.0 / 360.0 * HJ20_OUTPUT_COUNTS_PER_REV);
    for (int k = 1; k < HJ20_ENC_MISMATCH_SAMPLES; k++) {
        CHECK(!hj20_encoder_slow_update(&e, output_raw_for(100, skew), 0.0f, HJ20_SLOW_LOOP_DT_S));
    }
    CHECK(hj20_encoder_slow_update(&e, output_raw_for(100, skew), 0.0f, HJ20_SLOW_LOOP_DT_S));
}

static void mismatch_tolerance_includes_gear_windup(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    (void)hj20_encoder_motor_update(&e, 100);
    (void)hj20_encoder_slow_update(&e, output_raw_for(100, 0), 0.0f, HJ20_SLOW_LOOP_DT_S);
    /* 0.6 deg deflection: a fault at no load, legitimate wind-up at 50 Nm. */
    uint32_t skew = (uint32_t)(0.6 / 360.0 * HJ20_OUTPUT_COUNTS_PER_REV);
    bool tripped = false;
    for (int k = 0; k < 50; k++) {
        tripped |= hj20_encoder_slow_update(&e, output_raw_for(100, skew), 50.0f, HJ20_SLOW_LOOP_DT_S);
    }
    CHECK(!tripped);
}

static void velocity_estimate_from_motor_encoder(void)
{
    hj20_encoder_t e;
    hj20_encoder_init(&e, 0.0f);
    int64_t acc = 0;
    uint16_t raw = 0;
    (void)hj20_encoder_motor_update(&e, raw);
    for (int tick = 0; tick < 200; tick++) {
        for (int k = 0; k < 20; k++) {
            acc += 13;
            raw = (uint16_t)((raw + 13u) & 0x3FFFu);
            (void)hj20_encoder_motor_update(&e, raw);
        }
        (void)hj20_encoder_slow_update(&e, output_raw_for(acc, 0), 0.0f, HJ20_SLOW_LOOP_DT_S);
    }
    /* 260 counts/ms = 260 / 1 638 400 rev/ms at the output. */
    double expected = 260.0 / 1638400.0 * 6.283185307 * 1000.0;
    CHECK_NEAR(e.vel_rad_s, expected, 1e-4);
}

HJ20_SUITE(suite_encoder, "encoder",
    {"test_encoder_counts_to_angle", counts_to_angle},
    {"test_encoder_motor_wrap_accumulates_across_rev_boundary", motor_wrap_accumulates_across_rev_boundary},
    {"test_encoder_output_from_motor_follows_gear_ratio", output_from_motor_follows_gear_ratio},
    {"test_encoder_electrical_angle_uses_pole_pairs", electrical_angle_uses_pole_pairs},
    {TEST_NAME(REQ_ENCODER, "consistent_encoders_raise_no_mismatch"), consistent_encoders_raise_no_mismatch},
    {TEST_NAME(REQ_ENCODER, "mismatch_detected_after_debounce"), mismatch_detected_after_debounce},
    {TEST_NAME(REQ_ENCODER, "mismatch_tolerance_includes_gear_windup"), mismatch_tolerance_includes_gear_windup},
    {"test_encoder_velocity_estimate_from_motor_encoder", velocity_estimate_from_motor_encoder},
);
