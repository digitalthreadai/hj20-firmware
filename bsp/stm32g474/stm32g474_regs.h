/**
 * @file stm32g474_regs.h
 * @brief The handful of STM32G474 / Cortex-M4 registers this board layer touches.
 *
 * Written from RM0440 (STM32G4 reference manual) and the ARMv7-M ARM; this
 * is deliberately NOT the vendor CMSIS device header. Addresses and bit
 * positions are to be cross-checked against RM0440 at board bring-up.
 */
#ifndef HJ20_STM32G474_REGS_H
#define HJ20_STM32G474_REGS_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* ---- Cortex-M4 system ---------------------------------------------------- */
#define SCB_VTOR        REG32(0xE000ED08u)
#define SCB_CPACR       REG32(0xE000ED88u)
#define SYST_CSR        REG32(0xE000E010u)
#define SYST_RVR        REG32(0xE000E014u)
#define SYST_CVR        REG32(0xE000E018u)
#define NVIC_ISER(n)    REG32(0xE000E100u + 4u * (n))
#define NVIC_IPR_BYTE(irq) (*(volatile uint8_t *)(uintptr_t)(0xE000E400u + (irq)))
#define SCB_SHPR3       REG32(0xE000ED20u)

/* ---- RCC (0x4002 1000) --------------------------------------------------- */
#define RCC_BASE        0x40021000u
#define RCC_CR          REG32(RCC_BASE + 0x00u)
#define RCC_CFGR        REG32(RCC_BASE + 0x08u)
#define RCC_PLLCFGR     REG32(RCC_BASE + 0x0Cu)
#define RCC_AHB2ENR     REG32(RCC_BASE + 0x4Cu)
#define RCC_APB1ENR1    REG32(RCC_BASE + 0x58u)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x60u)
#define RCC_CCIPR       REG32(RCC_BASE + 0x88u)

#define RCC_CR_HSEON    (1u << 16)
#define RCC_CR_HSERDY   (1u << 17)
#define RCC_CR_PLLON    (1u << 24)
#define RCC_CR_PLLRDY   (1u << 25)

#define RCC_AHB2ENR_GPIOAEN  (1u << 0)
#define RCC_AHB2ENR_GPIOBEN  (1u << 1)
#define RCC_AHB2ENR_GPIOCEN  (1u << 2)
#define RCC_AHB2ENR_GPIODEN  (1u << 3)
#define RCC_AHB2ENR_ADC12EN  (1u << 13)
#define RCC_APB1ENR1_SPI3EN  (1u << 15)
#define RCC_APB1ENR1_FDCANEN (1u << 25)
#define RCC_APB1ENR1_PWREN   (1u << 28)
#define RCC_APB2ENR_TIM1EN   (1u << 11)
#define RCC_APB2ENR_SPI1EN   (1u << 12)

/* ---- FLASH / PWR ---------------------------------------------------------- */
#define FLASH_ACR       REG32(0x40022000u)
#define PWR_CR5         REG32(0x40007000u + 0x80u)
#define PWR_CR5_R1MODE  (1u << 8)   /* 0 = range-1 boost (required for 170 MHz) */

/* ---- GPIO (AHB2) ------------------------------------------------------------ */
#define GPIOA_BASE      0x48000000u
#define GPIOB_BASE      0x48000400u
#define GPIOC_BASE      0x48000800u
#define GPIOD_BASE      0x48000C00u

#define GPIO_MODER(p)   REG32((p) + 0x00u)
#define GPIO_OSPEEDR(p) REG32((p) + 0x08u)
#define GPIO_PUPDR(p)   REG32((p) + 0x0Cu)
#define GPIO_IDR(p)     REG32((p) + 0x10u)
#define GPIO_BSRR(p)    REG32((p) + 0x18u)
#define GPIO_AFRL(p)    REG32((p) + 0x20u)
#define GPIO_AFRH(p)    REG32((p) + 0x24u)

/* ---- TIM1 (advanced timer driving the bridge) ------------------------------ */
#define TIM1_BASE       0x40012C00u
#define TIM1_BDTR       REG32(TIM1_BASE + 0x44u)
#define TIM1_BDTR_MOE   (1u << 15)
#define TIM1_CCR1       REG32(TIM1_BASE + 0x34u)
#define TIM1_CCR2       REG32(TIM1_BASE + 0x38u)
#define TIM1_CCR3       REG32(TIM1_BASE + 0x3Cu)

/* ---- IRQ numbers used (RM0440 vector table) -------------------------------- */
#define IRQN_ADC1_2         18u
#define IRQN_FDCAN1_IT0     21u
#define IRQN_TIM1_BRK_TIM15 24u

#endif /* HJ20_STM32G474_REGS_H */
