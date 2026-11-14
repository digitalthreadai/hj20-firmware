/**
 * @file control.h
 * @brief Joint controller: ties FOC, loops, limits, faults and CAN together.
 *
 * Execution model (see bsp/stm32g474/main.c):
 *   - hj20_ctrl_current_isr()  20 kHz, ADC injected end-of-conversion ISR
 *   - hj20_ctrl_slow_tick()     1 kHz, SysTick
 *   - hj20_ctrl_on_can()        FDCAN RX FIFO0 ISR
 * The ISR reads `iq_ref`/`id_ref` written by the slow tick; both are single
 * 32-bit float stores, which are atomic on Cortex-M4.
 */
#ifndef HJ20_CONTROL_H
#define HJ20_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "can_proto.h"
#include "encoder.h"
#include "fault.h"
#include "foc.h"
#include "motion.h"
#include "pi.h"
#include "torque.h"

typedef struct {
    float ia, ib, ic;        /* A, from the DRV8353RS current-sense amplifiers */
    float vbus;              /* V                                              */
    uint16_t motor_enc_raw;  /* 14-bit                                         */
} hj20_fast_inputs_t;

typedef struct {
    float vbus;
    float temp_stage_c;
    float temp_motor_c;
    uint32_t output_enc_raw; /* 19-bit                                         */
    bool gate_driver_fault;  /* nFAULT pin, active                             */
    uint32_t now_ms;
} hj20_slow_inputs_t;

typedef struct {
    hj20_duty_t duty;
    bool pwm_enable;         /* false -> BSP disables the bridge (MOE = 0)     */
} hj20_fast_outputs_t;

typedef struct {
    uint8_t node_id;

    hj20_fault_t fault;
    hj20_encoder_t enc;
    hj20_motion_t motion;
    hj20_pi_t pi_d, pi_q;
    hj20_derate_t derate;

    /* command (from CAN) */
    uint32_t last_cmd_ms;     /* slow-loop time of the last valid COMMAND */
    uint16_t reported_faults; /* bits already announced in a FAULT frame  */
    hj20_mode_t mode;
    uint8_t last_seq;
    float setpoint;          /* SI: Nm | rad/s | rad */
    float torque_ff_nm;
    float torque_limit_nm;   /* as commanded over CAN                    */
    float torque_limit_eff;  /* after peak budget and thermal derating   */

    /* references for the current loop */
    volatile float id_ref, iq_ref;

    /* measurements for telemetry */
    float iq_meas, vbus, temp_stage_c, temp_motor_c, torque_cmd_nm;
    uint32_t now_ms;
} hj20_ctrl_t;

void hj20_ctrl_init(hj20_ctrl_t *c, uint8_t node_id, float elec_offset_rad);
void hj20_ctrl_current_isr(hj20_ctrl_t *c, const hj20_fast_inputs_t *in, hj20_fast_outputs_t *out);
void hj20_ctrl_slow_tick(hj20_ctrl_t *c, const hj20_slow_inputs_t *in);

/** Handle a received frame; returns the decode status (non-OK frames are ignored). */
hj20_can_status_t hj20_ctrl_on_can(hj20_ctrl_t *c, const hj20_can_frame_t *f);

void hj20_ctrl_build_telemetry(const hj20_ctrl_t *c, hj20_can_frame_t *f);

/**
 * If faults were latched since the last call, build the FAULT frame and
 * return true (send it immediately). Each latch event is reported once.
 */
bool hj20_ctrl_take_fault_event(hj20_ctrl_t *c, hj20_can_frame_t *f);

#endif /* HJ20_CONTROL_H */
