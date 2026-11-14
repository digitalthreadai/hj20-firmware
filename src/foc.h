/**
 * @file foc.h
 * @brief Field-oriented control transforms and space-vector PWM.
 *
 * Conventions: amplitude-invariant Clarke (|i_alpha_beta| equals the phase
 * peak), d axis aligned with rotor flux, positive q produces positive
 * (counter-clockwise, seen from the output flange) torque.
 */
#ifndef HJ20_FOC_H
#define HJ20_FOC_H

#include <stdbool.h>

typedef struct { float a, b, c; } hj20_abc_t;
typedef struct { float alpha, beta; } hj20_ab_t;
typedef struct { float d, q; } hj20_dq_t;
typedef struct { float a, b, c; } hj20_duty_t;   /* 0..1 per phase */

/** Clarke transform from three measured phase currents (no zero-sum assumption). */
hj20_ab_t hj20_clarke(hj20_abc_t i);

/** Inverse Clarke (alpha/beta to phase quantities). */
hj20_abc_t hj20_inv_clarke(hj20_ab_t v);

/** Park transform; sin/cos of the electrical angle are passed in. */
hj20_dq_t hj20_park(hj20_ab_t x, float sin_th, float cos_th);

/** Inverse Park transform. */
hj20_ab_t hj20_inv_park(hj20_dq_t x, float sin_th, float cos_th);

/**
 * Space-vector PWM by min/max zero-sequence injection.
 *
 * @param v     stator voltage command (V) in the alpha/beta frame
 * @param vbus  measured DC bus voltage (V), must be > 0
 * @param out   per-phase duty, clamped to [HJ20_DUTY_MIN, HJ20_DUTY_MAX]
 * @return true if the request exceeded the linear range and was clamped
 */
bool hj20_svpwm(hj20_ab_t v, float vbus, hj20_duty_t *out);

/** Largest voltage vector magnitude SVPWM reproduces without distortion. */
static inline float hj20_svpwm_linear_limit(float vbus)
{
    return vbus * 0.577350269f; /* vbus / sqrt(3) */
}

#endif /* HJ20_FOC_H */
