/**
 * @file startup.c
 * @brief Reset handler and vector table for the STM32G474 (Cortex-M4F).
 *
 * Written for this project (no vendor startup code). Order in Reset_Handler
 * matters: the FPU is enabled before any C code that might touch it, then
 * .data is copied, .bss and the CCM block are zeroed, and main() runs.
 */
#include <stdint.h>

#include "stm32g474_regs.h"

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _sccmbss, _eccmbss;

int main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Exceptions and IRQs default to Default_Handler unless the firmware defines them. */
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_ALIAS;
void HardFault_Handler(void) WEAK_ALIAS;
void MemManage_Handler(void) WEAK_ALIAS;
void BusFault_Handler(void) WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;
void ADC1_2_IRQHandler(void) WEAK_ALIAS;
void FDCAN1_IT0_IRQHandler(void) WEAK_ALIAS;
void FDCAN1_IT1_IRQHandler(void) WEAK_ALIAS;
void TIM1_BRK_TIM15_IRQHandler(void) WEAK_ALIAS;
void TIM1_UP_TIM16_IRQHandler(void) WEAK_ALIAS;

#define N_IRQ 102u   /* STM32G474: IRQ 0..101 */
#define D Default_Handler

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_t g_vector_table[16u + N_IRQ] = {
    (vector_t)(uintptr_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    /* IRQ 0..15: WWDG, PVD, RTC, RTC_WKUP, FLASH, RCC, EXTI0..4, DMA1_CH1..5 */
    D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D,
    /* IRQ 16, 17 */ D, D,
    /* IRQ 18 */ ADC1_2_IRQHandler,
    /* IRQ 19, 20: USB_HP, USB_LP */ D, D,
    /* IRQ 21 */ FDCAN1_IT0_IRQHandler,
    /* IRQ 22 */ FDCAN1_IT1_IRQHandler,
    /* IRQ 23: EXTI9_5 */ D,
    /* IRQ 24 */ TIM1_BRK_TIM15_IRQHandler,
    /* IRQ 25 */ TIM1_UP_TIM16_IRQHandler,
    /* IRQ 26..101: remaining peripherals, unused by this firmware */
    [16u + 26u ... 16u + N_IRQ - 1u] = D,
};

void Reset_Handler(void)
{
    /* CP10/CP11 full access before any floating-point instruction. */
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb\n\tisb" ::: "memory");

    volatile uint32_t *src = &_sidata;
    volatile uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0u;
    }
    for (dst = &_sccmbss; dst < &_eccmbss;) {
        *dst++ = 0u;
    }

    SCB_VTOR = (uint32_t)(uintptr_t)g_vector_table;

    (void)main();
    for (;;) {
        /* main() must not return; park here if it does. */
    }
}

void Default_Handler(void)
{
    /* Unexpected exception: stop the bridge first, then hang for the debugger. */
    TIM1_BDTR &= ~TIM1_BDTR_MOE;
    for (;;) {
    }
}
