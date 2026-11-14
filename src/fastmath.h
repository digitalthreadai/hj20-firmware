/**
 * @file fastmath.h
 * @brief libm-free trigonometry for the control loop.
 *
 * The target image links without libc/libm (-nostdlib), so sin/cos are
 * provided here: quadrant reduction to [-pi/4, pi/4] followed by truncated
 * Taylor polynomials. Max abs error < 2e-6 for |x| <= 4*pi (verified by
 * the host tests against libm). Callers keep angles wrapped; float range
 * reduction of large arguments loses precision like any float32 libm.
 */
#ifndef HJ20_FASTMATH_H
#define HJ20_FASTMATH_H

#define HJ20_PI      3.14159265358979f
#define HJ20_TWO_PI  6.28318530717959f
#define HJ20_SQRT3   1.73205080756888f

/** Wrap an angle into [0, 2*pi). */
float hj20_wrap_2pi(float x);

/** Wrap an angle into [-pi, pi). */
float hj20_wrap_pi(float x);

/** Compute sin(x) and cos(x) together (one range reduction). */
void hj20_sincosf(float x, float *s, float *c);

static inline float hj20_clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float hj20_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* Compiles to a single VSQRT.F32 on Cortex-M4F with -fno-math-errno. */
static inline float hj20_sqrtf(float x)
{
    return __builtin_sqrtf(x);
}

#endif /* HJ20_FASTMATH_H */
