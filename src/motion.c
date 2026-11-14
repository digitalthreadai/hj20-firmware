#include "motion.h"

#include "fastmath.h"
#include "hj20_config.h"

/* Gains tuned on the lumped model: J_out = 0.35 kg m^2 (wrist + tool),
 * velocity bandwidth ~ 30 Hz, position bandwidth ~ 5 Hz. */
#define VEL_KP_NM_PER_RAD_S   60.0f
#define VEL_KI_NM_PER_RAD     400.0f
#define POS_KP_PER_S          30.0f

void hj20_motion_init(hj20_motion_t *m)
{
    hj20_pi_init(&m->vel_pi, VEL_KP_NM_PER_RAD_S, VEL_KI_NM_PER_RAD,
                 -HJ20_PEAK_TORQUE_NM, HJ20_PEAK_TORQUE_NM);
    m->pos_kp = POS_KP_PER_S;
    m->vel_limit = HJ20_MAX_SPEED_RAD_S;
}

void hj20_motion_reset(hj20_motion_t *m)
{
    hj20_pi_reset(&m->vel_pi);
}

float hj20_motion_position(const hj20_motion_t *m, float pos_ref, float pos_meas, float vel_ff)
{
    /* Shortest-path error so a joint crossing +/-pi does not unwind. */
    float err = hj20_wrap_pi(pos_ref - pos_meas);
    float vel = m->pos_kp * err + vel_ff;
    return hj20_clampf(vel, -m->vel_limit, m->vel_limit);
}

float hj20_motion_velocity(hj20_motion_t *m, float vel_ref, float vel_meas, float torque_limit, float dt)
{
    float lim = hj20_clampf(torque_limit, 0.0f, HJ20_PEAK_TORQUE_NM);
    m->vel_pi.out_min = -lim;
    m->vel_pi.out_max = lim;
    float ref = hj20_clampf(vel_ref, -m->vel_limit, m->vel_limit);
    return hj20_pi_step(&m->vel_pi, ref - vel_meas, dt);
}
