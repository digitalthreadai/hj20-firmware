#include "encoder.h"

#include "fastmath.h"
#include "hj20_config.h"

#define VEL_FILTER_ALPHA 0.2f  /* first-order IIR on the 1 kHz difference */

float hj20_counts_to_rad(uint32_t counts, uint32_t bits)
{
    uint32_t mask = (bits >= 32u) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    float frac = (float)(counts & mask) / (float)(mask + 1u);
    return frac * HJ20_TWO_PI;
}

void hj20_encoder_init(hj20_encoder_t *e, float elec_offset_rad)
{
    e->last_raw = 0;
    e->acc = 0;
    e->acc_at_last_slow = 0;
    e->elec_offset = elec_offset_rad;
    e->out_offset = 0.0f;
    e->vel_rad_s = 0.0f;
    e->out_pos_rad = 0.0f;
    e->mismatch_count = 0;
    e->aligned = false;
    e->primed = false;
}

static int32_t wrap_acc(int32_t acc)
{
    const int32_t n = (int32_t)HJ20_MOTOR_COUNTS_PER_OUTPUT_REV;
    acc %= n;
    if (acc < 0) {
        acc += n;
    }
    return acc;
}

float hj20_encoder_motor_update(hj20_encoder_t *e, uint16_t raw14)
{
    raw14 &= (uint16_t)(HJ20_MOTOR_COUNTS_PER_REV - 1);
    if (!e->primed) {
        e->last_raw = raw14;
        e->acc = raw14;
        e->acc_at_last_slow = raw14;
        e->primed = true;
    }

    /* Signed delta with wrap: valid while |speed| < half a motor rev per
     * sample (300 000 rpm at 20 kHz), i.e. always. */
    int32_t delta = (int32_t)raw14 - (int32_t)e->last_raw;
    if (delta > (int32_t)(HJ20_MOTOR_COUNTS_PER_REV / 2)) {
        delta -= (int32_t)HJ20_MOTOR_COUNTS_PER_REV;
    } else if (delta < -(int32_t)(HJ20_MOTOR_COUNTS_PER_REV / 2)) {
        delta += (int32_t)HJ20_MOTOR_COUNTS_PER_REV;
    }
    e->last_raw = raw14;
    e->acc = wrap_acc(e->acc + delta);

    float mech = hj20_counts_to_rad(raw14, HJ20_MOTOR_ENC_BITS);
    return hj20_wrap_2pi(mech * (float)HJ20_POLE_PAIRS - e->elec_offset);
}

float hj20_encoder_output_from_motor(const hj20_encoder_t *e)
{
    float frac = (float)e->acc / (float)HJ20_MOTOR_COUNTS_PER_OUTPUT_REV;
    return hj20_wrap_2pi(frac * HJ20_TWO_PI + e->out_offset);
}

bool hj20_encoder_slow_update(hj20_encoder_t *e, uint32_t raw19, float torque_nm, float dt)
{
    e->out_pos_rad = hj20_counts_to_rad(raw19, HJ20_OUTPUT_ENC_BITS);

    /* Velocity from the motor encoder: 100x the resolution of the output one. */
    int32_t d = e->acc - e->acc_at_last_slow;
    const int32_t n = (int32_t)HJ20_MOTOR_COUNTS_PER_OUTPUT_REV;
    if (d > n / 2) {
        d -= n;
    } else if (d < -n / 2) {
        d += n;
    }
    e->acc_at_last_slow = e->acc;
    float vel = ((float)d / (float)n) * HJ20_TWO_PI / dt;
    e->vel_rad_s += VEL_FILTER_ALPHA * (vel - e->vel_rad_s);

    if (!e->aligned) {
        /* First valid pair defines the alignment (absolute encoders). */
        float frac = (float)e->acc / (float)n;
        e->out_offset = hj20_wrap_2pi(e->out_pos_rad - frac * HJ20_TWO_PI);
        e->aligned = true;
        e->mismatch_count = 0;
        return false;
    }

    float diff = hj20_wrap_pi(e->out_pos_rad - hj20_encoder_output_from_motor(e));
    float tol = HJ20_ENC_MISMATCH_TOL_RAD + hj20_absf(torque_nm) / HJ20_GEAR_STIFFNESS_NM_PER_RAD;

    if (hj20_absf(diff) > tol) {
        if (e->mismatch_count < 0xFFFFu) {
            e->mismatch_count++;
        }
    } else {
        e->mismatch_count = 0;
    }
    return e->mismatch_count >= HJ20_ENC_MISMATCH_SAMPLES;
}
