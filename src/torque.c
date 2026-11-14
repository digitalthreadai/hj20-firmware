#include "torque.h"

#include "fastmath.h"
#include "hj20_config.h"

#define NM_PER_AMP_OUTPUT (HJ20_KT_NM_PER_A * HJ20_GEAR_RATIO * HJ20_GEAR_EFFICIENCY)

float hj20_torque_to_iq(float torque_nm)
{
    return torque_nm * (1.0f / NM_PER_AMP_OUTPUT);
}

float hj20_iq_to_torque(float iq_a)
{
    return iq_a * NM_PER_AMP_OUTPUT;
}

float hj20_torque_clamp(float torque_nm, float limit_nm)
{
    float lim = hj20_clampf(limit_nm, 0.0f, HJ20_PEAK_TORQUE_NM);
    return hj20_clampf(torque_nm, -lim, lim);
}

void hj20_derate_init(hj20_derate_t *d)
{
    d->i2t_acc = 0.0f;
    d->folded_back = false;
}

float hj20_thermal_scale(float temp_c, float start_c, float end_c)
{
    if (temp_c <= start_c) {
        return 1.0f;
    }
    if (temp_c >= end_c) {
        return 0.0f;
    }
    return (end_c - temp_c) / (end_c - start_c);
}

float hj20_derate_limit(hj20_derate_t *d, float requested_limit_nm, float torque_cmd_nm,
                        float temp_stage_c, float temp_motor_c, float dt)
{
    const float rated2 = HJ20_RATED_TORQUE_NM * HJ20_RATED_TORQUE_NM;

    d->i2t_acc += (torque_cmd_nm * torque_cmd_nm - rated2) * dt;
    if (d->i2t_acc < 0.0f) {
        d->i2t_acc = 0.0f;
    }
    if (!d->folded_back && d->i2t_acc >= HJ20_I2T_BUDGET_NM2S) {
        d->folded_back = true;
    } else if (d->folded_back && d->i2t_acc <= HJ20_I2T_RELEASE_FRACTION * HJ20_I2T_BUDGET_NM2S) {
        d->folded_back = false;
    }

    float cap = d->folded_back ? HJ20_RATED_TORQUE_NM : HJ20_PEAK_TORQUE_NM;
    float s_stage = hj20_thermal_scale(temp_stage_c, HJ20_TEMP_STAGE_DERATE_START_C, HJ20_TEMP_STAGE_DERATE_END_C);
    float s_motor = hj20_thermal_scale(temp_motor_c, HJ20_TEMP_MOTOR_DERATE_START_C, HJ20_TEMP_MOTOR_DERATE_END_C);
    float scale = (s_stage < s_motor) ? s_stage : s_motor;

    float lim = hj20_clampf(requested_limit_nm, 0.0f, cap);
    return lim * scale;
}
