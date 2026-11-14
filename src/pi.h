/**
 * @file pi.h
 * @brief Discrete PI controller with output limits, plus the dq voltage limiter.
 */
#ifndef HJ20_PI_H
#define HJ20_PI_H

typedef struct {
    float kp;        /* proportional gain                 */
    float ki;        /* integral gain (1/s)               */
    float out_min;   /* output saturation                 */
    float out_max;
    float integ;     /* integrator state, in output units */
} hj20_pi_t;

void hj20_pi_init(hj20_pi_t *pi, float kp, float ki, float out_min, float out_max);
void hj20_pi_reset(hj20_pi_t *pi);

/** One controller step: returns the saturated output for error `err`. */
float hj20_pi_step(hj20_pi_t *pi, float err, float dt);

/**
 * Limit a dq voltage vector to magnitude `vmax`, giving the d axis priority
 * (field control first, torque gets what is left).
 */
void hj20_dq_limit(float *vd, float *vq, float vmax);

#endif /* HJ20_PI_H */
