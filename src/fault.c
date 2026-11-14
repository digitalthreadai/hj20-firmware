#include "fault.h"

#include "fastmath.h"
#include "hj20_config.h"

void hj20_fault_init(hj20_fault_t *f)
{
    f->state = HJ20_STATE_INIT;
    f->latched = 0;
    f->active = 0;
    f->first = 0;
    f->ov_count = f->uv_count = f->ot_stage_count = f->ot_motor_count = 0;
}

void hj20_fault_raise(hj20_fault_t *f, uint16_t bits)
{
    if (bits == 0u) {
        return;
    }
    if (f->latched == 0u) {
        /* Lowest set bit of this event is reported as the root cause. */
        f->first = (uint16_t)(bits & (uint16_t)(~bits + 1u));
    }
    f->latched |= bits;
    f->state = HJ20_STATE_FAULT;
}

/* Debounced trip: counts consecutive samples over the limit. */
static bool debounce(uint16_t *count, bool over, uint16_t limit)
{
    if (over) {
        if (*count < limit) {
            (*count)++;
        }
    } else {
        *count = 0;
    }
    return *count >= limit;
}

void hj20_fault_evaluate(hj20_fault_t *f, const hj20_fault_inputs_t *in)
{
    uint16_t active = 0;

    if (debounce(&f->ov_count, in->vbus_v > HJ20_VBUS_OV_TRIP_V, HJ20_OV_DEBOUNCE)) {
        active |= HJ20_FAULT_OVER_VOLTAGE;
    }
    if (debounce(&f->ot_stage_count, in->temp_stage_c > HJ20_TEMP_STAGE_TRIP_C, HJ20_OT_DEBOUNCE)) {
        active |= HJ20_FAULT_OVER_TEMP_STAGE;
    }
    if (debounce(&f->ot_motor_count, in->temp_motor_c > HJ20_TEMP_MOTOR_TRIP_C, HJ20_OT_DEBOUNCE)) {
        active |= HJ20_FAULT_OVER_TEMP_MOTOR;
    }
    if (in->encoder_mismatch) {
        active |= HJ20_FAULT_ENCODER_MISMATCH;
    }
    if (in->gate_driver_fault) {
        active |= HJ20_FAULT_GATE_DRIVER;
    }

    if (f->state == HJ20_STATE_INIT) {
        /* Power-up: wait for the bus to rise past the release threshold.
         * Under-voltage before that is expected, not a fault. */
        f->uv_count = 0;
        if (in->vbus_v >= HJ20_VBUS_UV_RELEASE_V && active == 0u) {
            f->state = HJ20_STATE_DISABLED;
        }
    } else if (debounce(&f->uv_count, in->vbus_v < HJ20_VBUS_UV_TRIP_V, HJ20_UV_DEBOUNCE)) {
        active |= HJ20_FAULT_UNDER_VOLTAGE;
    }

    /* Over-current is instantaneous and ISR-evaluated, so it is never part of
     * `active`: a clear after an OC trip is judged on these conditions. */
    f->active = active;
    if (active != 0u && f->state != HJ20_STATE_INIT) {
        hj20_fault_raise(f, active);
    }
}

void hj20_fault_check_current(hj20_fault_t *f, float ia, float ib, float ic)
{
    if (hj20_absf(ia) > HJ20_PHASE_OC_TRIP_A ||
        hj20_absf(ib) > HJ20_PHASE_OC_TRIP_A ||
        hj20_absf(ic) > HJ20_PHASE_OC_TRIP_A) {
        hj20_fault_raise(f, HJ20_FAULT_OVER_CURRENT);
    }
}

bool hj20_fault_request_enable(hj20_fault_t *f)
{
    if (f->state == HJ20_STATE_DISABLED) {
        f->state = HJ20_STATE_ENABLED;
        return true;
    }
    return f->state == HJ20_STATE_ENABLED;
}

void hj20_fault_request_disable(hj20_fault_t *f)
{
    if (f->state == HJ20_STATE_ENABLED) {
        f->state = HJ20_STATE_DISABLED;
    }
}

bool hj20_fault_clear(hj20_fault_t *f)
{
    if (f->state != HJ20_STATE_FAULT) {
        return f->state != HJ20_STATE_INIT;
    }
    if (f->active != 0u) {
        return false;
    }
    f->latched = 0;
    f->first = 0;
    f->state = HJ20_STATE_DISABLED;
    return true;
}
