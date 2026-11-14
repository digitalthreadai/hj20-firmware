#include "pi.h"

#include "fastmath.h"

void hj20_pi_init(hj20_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integ = 0.0f;
}

void hj20_pi_reset(hj20_pi_t *pi)
{
    pi->integ = 0.0f;
}

float hj20_pi_step(hj20_pi_t *pi, float err, float dt)
{
    /* Anti-windup: the integrator is bounded to the output range. */
    pi->integ += pi->ki * err * dt;
    pi->integ = hj20_clampf(pi->integ, pi->out_min, pi->out_max);

    float u = pi->kp * err + pi->integ;
    return hj20_clampf(u, pi->out_min, pi->out_max);
}

void hj20_dq_limit(float *vd, float *vq, float vmax)
{
    float mag2 = (*vd) * (*vd) + (*vq) * (*vq);
    if (mag2 <= vmax * vmax) {
        return;
    }
    *vd = hj20_clampf(*vd, -vmax, vmax);
    float vq_room = hj20_sqrtf(vmax * vmax - (*vd) * (*vd));
    *vq = hj20_clampf(*vq, -vq_room, vq_room);
}
