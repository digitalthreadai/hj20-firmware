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
    /* Anti-windup by conditional integration: while the output is saturated
     * and the error would push it further into saturation, the integrator is
     * frozen. Bounding the integrator to the output range alone is not
     * enough: with a large error the P term saturates the output while the
     * integrator still charges to the limit, and the output then stays
     * pinned after the error reverses. */
    float p = pi->kp * err;
    float i_next = pi->integ + pi->ki * err * dt;
    float u = p + i_next;

    if ((u > pi->out_max && err > 0.0f) || (u < pi->out_min && err < 0.0f)) {
        i_next = pi->integ;
        u = p + i_next;
    }

    pi->integ = hj20_clampf(i_next, pi->out_min, pi->out_max);
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
