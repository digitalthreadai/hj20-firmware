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
