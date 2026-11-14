#include "foc.h"

#include "fastmath.h"
#include "hj20_config.h"

hj20_ab_t hj20_clarke(hj20_abc_t i)
{
    hj20_ab_t r;
    r.alpha = (2.0f * i.a - i.b - i.c) * (1.0f / 3.0f);
    r.beta = (i.b - i.c) * (1.0f / HJ20_SQRT3);
    return r;
}

hj20_abc_t hj20_inv_clarke(hj20_ab_t v)
{
    hj20_abc_t r;
    r.a = v.alpha;
    r.b = -0.5f * v.alpha + 0.5f * HJ20_SQRT3 * v.beta;
    r.c = -0.5f * v.alpha - 0.5f * HJ20_SQRT3 * v.beta;
    return r;
}

hj20_dq_t hj20_park(hj20_ab_t x, float sin_th, float cos_th)
{
    hj20_dq_t r;
    r.d = x.alpha * cos_th + x.beta * sin_th;
    r.q = -x.alpha * sin_th + x.beta * cos_th;
    return r;
}

hj20_ab_t hj20_inv_park(hj20_dq_t x, float sin_th, float cos_th)
{
    hj20_ab_t r;
    r.alpha = x.d * cos_th - x.q * sin_th;
    r.beta = x.d * sin_th + x.q * cos_th;
    return r;
}

static float max3(float a, float b, float c)
{
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

static float min3(float a, float b, float c)
{
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

bool hj20_svpwm(hj20_ab_t v, float vbus, hj20_duty_t *out)
{
    if (vbus <= 1.0f) {
        out->a = out->b = out->c = 0.5f;
        return true;
    }

    hj20_abc_t ph = hj20_inv_clarke(v);

    /* Centre the three phase voltages in the available bus: this zero-sequence
     * offset is what extends the linear range from vbus/2 to vbus/sqrt(3). */
    float offset = -0.5f * (max3(ph.a, ph.b, ph.c) + min3(ph.a, ph.b, ph.c));
    float inv_vbus = 1.0f / vbus;

    float da = 0.5f + (ph.a + offset) * inv_vbus;
    float db = 0.5f + (ph.b + offset) * inv_vbus;
    float dc = 0.5f + (ph.c + offset) * inv_vbus;

    bool clamped = (da < HJ20_DUTY_MIN) || (da > HJ20_DUTY_MAX) ||
                   (db < HJ20_DUTY_MIN) || (db > HJ20_DUTY_MAX) ||
                   (dc < HJ20_DUTY_MIN) || (dc > HJ20_DUTY_MAX);

    out->a = hj20_clampf(da, HJ20_DUTY_MIN, HJ20_DUTY_MAX);
    out->b = hj20_clampf(db, HJ20_DUTY_MIN, HJ20_DUTY_MAX);
    out->c = hj20_clampf(dc, HJ20_DUTY_MIN, HJ20_DUTY_MAX);
    return clamped;
}
