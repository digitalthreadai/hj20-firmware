/**
 * @file drv8353.h
 * @brief TI DRV8353RS smart gate driver: SPI register access and configuration.
 *
 * SPI word (16 bit, mode 1): [15] R/W (1 = read) | [14:11] address | [10:0] data.
 */
#ifndef HJ20_DRV8353_H
#define HJ20_DRV8353_H

#include <stdbool.h>
#include <stdint.h>

#define DRV8353_REG_FAULT_STATUS1  0x0u
#define DRV8353_REG_VGS_STATUS2    0x1u
#define DRV8353_REG_DRIVER_CTRL    0x2u
#define DRV8353_REG_GATE_HS        0x3u
#define DRV8353_REG_GATE_LS        0x4u
#define DRV8353_REG_OCP_CTRL       0x5u
#define DRV8353_REG_CSA_CTRL       0x6u

uint16_t drv8353_frame(bool read, uint8_t addr, uint16_t data);

/** Configure 6x PWM, CSA gain 10 V/V bidirectional, latched OCP; verify by read-back. */
bool drv8353_init(void);

/** Read both status registers; returns true if any fault bit is set. */
bool drv8353_read_faults(uint16_t *status1, uint16_t *status2);

#endif /* HJ20_DRV8353_H */
