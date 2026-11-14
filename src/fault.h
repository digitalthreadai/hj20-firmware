/**
 * @file fault.h
 * @brief Drive state machine and latched protection faults.
 *
 *            vbus ok              enable               any fault
 *   INIT ------------> DISABLED ----------> ENABLED -------------> FAULT
 *                         ^  <----------  disable                   |
 *                         +------------------------------------------+
 *                           clear request AND no active fault condition
 *
 * Faults latch: a condition that disappears on its own does NOT re-arm the
 * bridge. PWM is only allowed in ENABLED. Over-current is evaluated in the
 * current-loop ISR (single-sample trip); everything else in the slow loop.
 */
#ifndef HJ20_FAULT_H
#define HJ20_FAULT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HJ20_STATE_INIT = 0,
    HJ20_STATE_DISABLED = 1,
    HJ20_STATE_ENABLED = 2,
    HJ20_STATE_FAULT = 3,
} hj20_state_t;

typedef enum {
    HJ20_FAULT_NONE = 0u,
    HJ20_FAULT_OVER_CURRENT = 1u << 0,
    HJ20_FAULT_OVER_VOLTAGE = 1u << 1,
    HJ20_FAULT_UNDER_VOLTAGE = 1u << 2,
    HJ20_FAULT_OVER_TEMP_STAGE = 1u << 3,
    HJ20_FAULT_OVER_TEMP_MOTOR = 1u << 4,
    HJ20_FAULT_ENCODER_MISMATCH = 1u << 5,
    HJ20_FAULT_GATE_DRIVER = 1u << 6,   /* DRV8353RS nFAULT asserted */
} hj20_fault_bit_t;

typedef struct {
    float vbus_v;
    float temp_stage_c;
    float temp_motor_c;
    bool encoder_mismatch;
    bool gate_driver_fault;
} hj20_fault_inputs_t;

typedef struct {
    hj20_state_t state;
    uint16_t latched;      /* faults latched since the last clear         */
    uint16_t active;       /* conditions present at the last evaluation   */
    uint16_t first;        /* first fault bit of the current latch event  */
    uint16_t ov_count, uv_count, ot_stage_count, ot_motor_count;
} hj20_fault_t;

void hj20_fault_init(hj20_fault_t *f);

/** Slow-loop evaluation of bus, thermal, encoder and gate-driver conditions. */
void hj20_fault_evaluate(hj20_fault_t *f, const hj20_fault_inputs_t *in);

/** ISR-side over-current check on the three measured phase currents. */
void hj20_fault_check_current(hj20_fault_t *f, float ia, float ib, float ic);

/** Latch a fault from any context and force the FAULT state. */
void hj20_fault_raise(hj20_fault_t *f, uint16_t bits);

bool hj20_fault_request_enable(hj20_fault_t *f);
void hj20_fault_request_disable(hj20_fault_t *f);

/** Clear latched faults; refused (returns false) while a condition is still active. */
bool hj20_fault_clear(hj20_fault_t *f);

static inline bool hj20_fault_pwm_allowed(const hj20_fault_t *f)
{
    return f->state == HJ20_STATE_ENABLED;
}

#endif /* HJ20_FAULT_H */
