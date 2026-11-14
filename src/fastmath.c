#include "fastmath.h"

#include <stdint.h>

float hj20_wrap_2pi(float x)
{
    /* Truncate toward zero, then fix the sign; avoids floorf(). */
    int32_t k = (int32_t)(x * (1.0f / HJ20_TWO_PI));
    float r = x - (float)k * HJ20_TWO_PI;
    if (r < 0.0f) {
        r += HJ20_TWO_PI;
    }
    if (r >= HJ20_TWO_PI) {
        r -= HJ20_TWO_PI;
    }
    return r;
}

float hj20_wrap_pi(float x)
{
    float r = hj20_wrap_2pi(x + HJ20_PI) - HJ20_PI;
    return r;
}

void hj20_sincosf(float x, float *s, float *c)
{
    const float half_pi = 1.57079632679490f;
    float a = hj20_wrap_pi(x);

    /* Nearest quadrant: a = q * pi/2 + r, |r| <= pi/4. */
    float qf = a * (1.0f / half_pi);
    int32_t q = (int32_t)(qf + ((qf >= 0.0f) ? 0.5f : -0.5f));
    float r = a - (float)q * half_pi;

    float r2 = r * r;
    float sr = r * (1.0f + r2 * (-1.0f / 6.0f + r2 * (1.0f / 120.0f + r2 * (-1.0f / 5040.0f))));
    float cr = 1.0f + r2 * (-0.5f + r2 * (1.0f / 24.0f + r2 * (-1.0f / 720.0f + r2 * (1.0f / 40320.0f))));

    switch (q & 3) {
    case 0:  *s = sr;  *c = cr;  break;
    case 1:  *s = cr;  *c = -sr; break;
    case 2:  *s = -sr; *c = -cr; break;
    default: *s = -cr; *c = sr;  break;
    }
}
