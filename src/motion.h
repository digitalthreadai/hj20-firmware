/**
 * @file motion.h
 * @brief Output-side velocity and position loops (run at HJ20_SLOW_LOOP_HZ).
 *
 * Cascade: position (P + velocity feed-forward) -> velocity (PI) -> output
 * torque command -> torque limiter -> iq reference for the current loop.
 */
#ifndef HJ20_MOTION_H
#define HJ20_MOTION_H

#include "pi.h"

typedef struct {
    hj20_pi_t vel_pi;     /* rad/s error -> Nm                    */
    float pos_kp;         /* rad error -> rad/s                   */
    float vel_limit;      /* rad/s, |velocity command| ceiling    */
} hj20_motion_t;

void hj20_motion_init(hj20_motion_t *m);
void hj20_motion_reset(hj20_motion_t *m);

/** Position loop: returns a velocity command clamped to +/- vel_limit. */
float hj20_motion_position(const hj20_motion_t *m, float pos_ref, float pos_meas, float vel_ff);

/** Velocity loop: returns an output torque command clamped to +/- torque_limit. */
float hj20_motion_velocity(hj20_motion_t *m, float vel_ref, float vel_meas, float torque_limit, float dt);

#endif /* HJ20_MOTION_H */
