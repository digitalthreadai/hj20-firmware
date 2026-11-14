/**
 * @file board.h
 * @brief HJ-20 controller board (rev A) pin map and board services.
 *
 *  Function                 Pin    Peripheral / AF     Part
 *  ---------------------------------------------------------------------------
 *  PWM phase A high         PA8    TIM1_CH1   AF6      DRV8353RS INHA
 *  PWM phase B high         PA9    TIM1_CH2   AF6      DRV8353RS INHB
 *  PWM phase C high         PA10   TIM1_CH3   AF6      DRV8353RS INHC
 *  PWM phase A low          PB13   TIM1_CH1N  AF6      DRV8353RS INLA
 *  PWM phase B low          PB14   TIM1_CH2N  AF6      DRV8353RS INLB
 *  PWM phase C low          PB15   TIM1_CH3N  AF6      DRV8353RS INLC
 *  Gate driver SPI SCK      PA5    SPI1_SCK   AF5      DRV8353RS SCLK
 *  Gate driver SPI MISO     PA6    SPI1_MISO  AF5      DRV8353RS SDO
 *  Gate driver SPI MOSI     PA7    SPI1_MOSI  AF5      DRV8353RS SDI
 *  Gate driver SPI nCS      PA4    GPIO out            DRV8353RS nSCS
 *  Gate driver enable       PC6    GPIO out            DRV8353RS ENABLE
 *  Gate driver fault        PC7    GPIO in (EXTI)      DRV8353RS nFAULT
 *  Phase current A          PA0    ADC1_IN1            DRV8353RS SOA
 *  Phase current B          PA1    ADC1_IN2            DRV8353RS SOB
 *  Phase current C          PA2    ADC1_IN3            DRV8353RS SOC
 *  Bus voltage              PA3    ADC1_IN4            100k / 4.7k divider
 *  Power-stage NTC          PB1    ADC1_IN12           10k B3435, 10k pull-up
 *  Winding NTC              PC0    ADC12_IN6           10k B3435, 10k pull-up
 *  CAN-FD RX                PA11   FDCAN1_RX  AF9      TCAN1044 RXD
 *  CAN-FD TX                PA12   FDCAN1_TX  AF9      TCAN1044 TXD
 *  CAN transceiver standby  PB4    GPIO out            TCAN1044 STB
 *  Encoder SPI SCK          PC10   SPI3_SCK   AF6      motor + output encoder
 *  Encoder SPI MISO         PC11   SPI3_MISO  AF6
 *  Encoder SPI MOSI         PC12   SPI3_MOSI  AF6
 *  Motor encoder nCS        PD2    GPIO out            14-bit on-axis magnetic
 *  Output encoder nCS       PA15   GPIO out            19-bit absolute
 *  Node-ID strap 0..3       PC1..PC4 GPIO in pull-up   CAN node address (PB8 is BOOT0)
 *  Status LED               PC9    GPIO out
 *  SWD                      PA13/PA14 (reserved)
 *
 * Alternate-function numbers are from the DS12288 pin table; re-check each
 * against the datasheet before the first power-up.
 *
 * Clock: 24 MHz HSE -> PLL (M=6, N=85, R=2) -> 170 MHz SYSCLK.
 */
#ifndef HJ20_BOARD_H
#define HJ20_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "can_proto.h"

#define BOARD_SYSCLK_HZ        170000000u
#define BOARD_HSE_HZ           24000000u

/* Current sense: 5 mOhm low-side shunts, DRV8353RS CSA gain 10 V/V,
 * VREF/2 bias, 12-bit ADC on 3.3 V -> +/- 33 A full scale. */
#define BOARD_SHUNT_OHM        0.005f
#define BOARD_CSA_GAIN         10.0f
#define BOARD_VBUS_DIVIDER     ((100.0f + 4.7f) / 4.7f)

typedef struct {
    uint16_t ia, ib, ic;   /* raw injected conversions */
    uint16_t vbus;
} board_adc_fast_t;

void board_init(void);
void board_start_control(void);      /* arm TIM1 + ADC trigger, enable IRQs */

/** Safe torque off: immediately clears TIM1 MOE (all six gates low). */
void board_pwm_kill(void);
void board_pwm_set(float da, float db, float dc, bool enable);

uint8_t board_node_id(void);
bool board_gate_fault_active(void);
void board_led(bool on);

/* Conversions (raw ADC -> SI). */
float board_adc_to_phase_current(uint16_t raw);
float board_adc_to_vbus(uint16_t raw);
float board_adc_to_ntc_celsius(uint16_t raw);

/* Peripheral hooks, implemented as TODO stubs until bring-up. */
void board_adc_read_fast(board_adc_fast_t *out);
void board_adc_read_temps(uint16_t *stage, uint16_t *motor);
uint16_t board_motor_encoder_read(void);
uint32_t board_output_encoder_read(void);
uint16_t board_spi1_xfer16(uint16_t word);
bool board_can_receive(hj20_can_frame_t *f);
bool board_can_send(const hj20_can_frame_t *f);

#endif /* HJ20_BOARD_H */
