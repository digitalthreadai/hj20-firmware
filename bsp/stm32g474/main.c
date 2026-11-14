/**
 * @file main.c
 * @brief HJ-20 firmware entry point and interrupt wiring.
 *
 *   ADC1_2 (JEOS, 20 kHz)  -> hj20_ctrl_current_isr  -> TIM1 compare registers
 *   SysTick (1 kHz)        -> hj20_ctrl_slow_tick    (+ telemetry every 10 ms,
 *                                                     FAULT frame on each new latch)
 *   FDCAN1 IT0 (RX FIFO0)  -> hj20_ctrl_on_can
 *   TIM1 BRK (nFAULT/OCP)  -> hardware already cut the gates; latch the fault
 */
#include "board.h"
#include "control.h"
#include "drv8353.h"
#include "hj20_config.h"

#define HJ20_CCM __attribute__((section(".ccmbss")))
#define TELEMETRY_PERIOD_MS 10u

/* Elec. zero from commissioning (stored in option bytes in production). */
#define ELEC_OFFSET_RAD 0.0f

static HJ20_CCM hj20_ctrl_t g_ctrl;
static volatile uint32_t g_ms;

void ADC1_2_IRQHandler(void)
{
    board_adc_fast_t raw;
    board_adc_read_fast(&raw);

    hj20_fast_inputs_t in = {
        .ia = board_adc_to_phase_current(raw.ia),
        .ib = board_adc_to_phase_current(raw.ib),
        .ic = board_adc_to_phase_current(raw.ic),
        .vbus = board_adc_to_vbus(raw.vbus),
        .motor_enc_raw = board_motor_encoder_read(),
    };
    hj20_fast_outputs_t out;
    hj20_ctrl_current_isr(&g_ctrl, &in, &out);
    board_pwm_set(out.duty.a, out.duty.b, out.duty.c, out.pwm_enable);
    /* TODO(bring-up): clear ADC1 ISR.JEOS here. */
}

void SysTick_Handler(void)
{
    uint32_t now = ++g_ms;

    uint16_t t_stage, t_motor;
    board_adc_read_temps(&t_stage, &t_motor);
    board_adc_fast_t raw;
    board_adc_read_fast(&raw);

    hj20_slow_inputs_t in = {
        .vbus = board_adc_to_vbus(raw.vbus),
        .temp_stage_c = board_adc_to_ntc_celsius(t_stage),
        .temp_motor_c = board_adc_to_ntc_celsius(t_motor),
        .output_enc_raw = board_output_encoder_read(),
        .gate_driver_fault = board_gate_fault_active(),
        .now_ms = now,
    };
    hj20_ctrl_slow_tick(&g_ctrl, &in);

    hj20_can_frame_t ev;
    if (hj20_ctrl_take_fault_event(&g_ctrl, &ev)) {
        (void)board_can_send(&ev);
    }

    if ((now % TELEMETRY_PERIOD_MS) == 0u) {
        hj20_can_frame_t f;
        hj20_ctrl_build_telemetry(&g_ctrl, &f);
        (void)board_can_send(&f);
    }
    board_led(g_ctrl.fault.state == HJ20_STATE_ENABLED);
}

void FDCAN1_IT0_IRQHandler(void)
{
    hj20_can_frame_t f;
    while (board_can_receive(&f)) {
        (void)hj20_ctrl_on_can(&g_ctrl, &f);
    }
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    hj20_fault_raise(&g_ctrl.fault, HJ20_FAULT_GATE_DRIVER);
    /* TODO(bring-up): clear TIM1 SR.BIF. */
}

void HardFault_Handler(void)
{
    board_pwm_kill();
    for (;;) {
    }
}

int main(void)
{
    board_init();
    hj20_ctrl_init(&g_ctrl, board_node_id(), ELEC_OFFSET_RAD);
    if (!drv8353_init()) {
        /* Gate driver did not read back its configuration: never arm. */
        hj20_fault_raise(&g_ctrl.fault, HJ20_FAULT_GATE_DRIVER);
    }
    board_start_control();

    for (;;) {
        __asm volatile("wfi");
    }
}
