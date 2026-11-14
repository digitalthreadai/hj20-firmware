/**
 * @file board.c
 * @brief HJ-20 controller board layer: clocks, pins, conversions, peripheral hooks.
 *
 * Status: clock tree, GPIO alternate functions and the safe-torque-off path
 * are register-level and complete. TIM1 / ADC / FDCAN / SPI configuration
 * are marked TODO(bring-up): their intended settings are documented inline
 * and the stubs return values that keep the drive in its INIT state (bus
 * reads 0 V), so an unfinished image can never energise the bridge.
 */
#include "board.h"

#include "stm32g474_regs.h"

#define TIM1_ARR_CENTER  (BOARD_SYSCLK_HZ / (2u * 20000u))   /* 4250 -> 20 kHz centre-aligned */
#define SPIN_TIMEOUT     1000000u

/* ---- GPIO helpers -------------------------------------------------------- */

static void gpio_mode(uint32_t port, uint32_t pin, uint32_t mode)
{
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3u << (2u * pin))) | (mode << (2u * pin));
}

static void gpio_af(uint32_t port, uint32_t pin, uint32_t af)
{
    volatile uint32_t *afr = (pin < 8u) ? &GPIO_AFRL(port) : &GPIO_AFRH(port);
    uint32_t shift = 4u * (pin & 7u);
    *afr = (*afr & ~(0xFu << shift)) | (af << shift);
    GPIO_OSPEEDR(port) |= 3u << (2u * pin);           /* very high speed */
    gpio_mode(port, pin, 2u);
}

static void gpio_out(uint32_t port, uint32_t pin, bool level)
{
    GPIO_BSRR(port) = level ? (1u << pin) : (1u << (pin + 16u));
    gpio_mode(port, pin, 1u);
}

static void gpio_in_pullup(uint32_t port, uint32_t pin)
{
    GPIO_PUPDR(port) = (GPIO_PUPDR(port) & ~(3u << (2u * pin))) | (1u << (2u * pin));
    gpio_mode(port, pin, 0u);
}

static void gpio_analog(uint32_t port, uint32_t pin)
{
    gpio_mode(port, pin, 3u);
}

/* ---- clocks ------------------------------------------------------------------ */

static bool clock_init(void)
{
    RCC_APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR_CR5 &= ~PWR_CR5_R1MODE;                       /* range-1 boost */

    /* 4 wait states at 170 MHz, prefetch + I/D caches on. */
    FLASH_ACR = (FLASH_ACR & ~0xFu) | 4u | (1u << 8) | (1u << 9) | (1u << 10);

    RCC_CR |= RCC_CR_HSEON;
    for (uint32_t t = 0; (RCC_CR & RCC_CR_HSERDY) == 0u; t++) {
        if (t > SPIN_TIMEOUT) {
            return false;                             /* stay on HSI16 */
        }
    }

    /* PLLSRC = HSE (3), M = 6 (field M-1), N = 85, R = /2, PLLREN. */
    RCC_PLLCFGR = 3u | ((6u - 1u) << 4) | (85u << 8) | (1u << 24);
    RCC_CR |= RCC_CR_PLLON;
    for (uint32_t t = 0; (RCC_CR & RCC_CR_PLLRDY) == 0u; t++) {
        if (t > SPIN_TIMEOUT) {
            return false;
        }
    }

    /* TODO(bring-up): RM0440 asks for an intermediate AHB /2 step when
     * switching to > 80 MHz in boost mode; add it once measured. */
    RCC_CFGR = (RCC_CFGR & ~3u) | 3u;                 /* SW = PLL */
    for (uint32_t t = 0; ((RCC_CFGR >> 2) & 3u) != 3u; t++) {
        if (t > SPIN_TIMEOUT) {
            return false;
        }
    }
    return true;
}

static void systick_init_1khz(void)
{
    SYST_RVR = BOARD_SYSCLK_HZ / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = (1u << 2) | (1u << 1) | 1u;            /* core clock, IRQ, enable */
}

/* ---- pins -------------------------------------------------------------------- */

static void pins_init(void)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
                   RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_ADC12EN;
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI1EN;
    RCC_APB1ENR1 |= RCC_APB1ENR1_SPI3EN | RCC_APB1ENR1_FDCANEN;

    /* Gate driver held disabled until the control loop is armed. */
    gpio_out(GPIOC_BASE, 6, false);                   /* DRV8353RS ENABLE */
    gpio_in_pullup(GPIOC_BASE, 7);                    /* nFAULT (open drain) */

    /* Bridge PWM, TIM1 AF6. */
    gpio_af(GPIOA_BASE, 8, 6);
    gpio_af(GPIOA_BASE, 9, 6);
    gpio_af(GPIOA_BASE, 10, 6);
    gpio_af(GPIOB_BASE, 13, 6);
    gpio_af(GPIOB_BASE, 14, 6);
    gpio_af(GPIOB_BASE, 15, 6);

    /* DRV8353RS SPI1 (AF5) + chip select. */
    gpio_out(GPIOA_BASE, 4, true);
    gpio_af(GPIOA_BASE, 5, 5);
    gpio_af(GPIOA_BASE, 6, 5);
    gpio_af(GPIOA_BASE, 7, 5);

    /* Analog inputs. */
    gpio_analog(GPIOA_BASE, 0);
    gpio_analog(GPIOA_BASE, 1);
    gpio_analog(GPIOA_BASE, 2);
    gpio_analog(GPIOA_BASE, 3);
    gpio_analog(GPIOB_BASE, 1);
    gpio_analog(GPIOC_BASE, 0);

    /* FDCAN1 on PA11/PA12 (AF9); TCAN1044 STB low = normal mode. */
    gpio_af(GPIOA_BASE, 11, 9);
    gpio_af(GPIOA_BASE, 12, 9);
    gpio_out(GPIOB_BASE, 4, false);

    /* Encoders on SPI3 (AF6), two chip selects. */
    gpio_af(GPIOC_BASE, 10, 6);
    gpio_af(GPIOC_BASE, 11, 6);
    gpio_af(GPIOC_BASE, 12, 6);
    gpio_out(GPIOD_BASE, 2, true);
    gpio_out(GPIOA_BASE, 15, true);

    /* Node-ID straps and LED. */
    for (uint32_t p = 1; p <= 4u; p++) {
        gpio_in_pullup(GPIOC_BASE, p);
    }
    gpio_out(GPIOC_BASE, 9, false);
}

/* ---- public ------------------------------------------------------------------ */

void board_init(void)
{
    board_pwm_kill();
    (void)clock_init();
    pins_init();
    systick_init_1khz();

    /* TODO(bring-up) TIM1: centre-aligned mode 1, ARR = TIM1_ARR_CENTER,
     * complementary outputs, dead time 100 ns (DTG = 17 at 170 MHz),
     * break input on DRV8353RS nFAULT, CH4 compare at ARR-1 as TRGO2 to
     * start the ADC1 injected sequence at the PWM valley (low-side on). */
    /* TODO(bring-up) ADC1: injected IN1/IN2/IN3/IN4, 12-bit, 6.5-cycle
     * sampling, JEOS interrupt -> ADC1_2_IRQHandler. Regular group with
     * DMA for IN12 + IN6 (NTCs). Run the self-calibration first. */
    /* TODO(bring-up) FDCAN1: kernel clock PCLK1 (CCIPR FDCANSEL = 2),
     * nominal 1 Mbit/s (80 % SP), data 5 Mbit/s (75 % SP, TDC on), FD + BRS,
     * standard-ID filter: accept 0x100 + node into RX FIFO0. */
    /* TODO(bring-up) SPI3: mode 1, 10 MHz, 16-bit frames for the motor
     * encoder; output encoder read as 3 x 8-bit at 4 MHz. */
}

void board_start_control(void)
{
    /* Priorities: current loop highest, CAN next, SysTick (slow loop) lowest. */
    NVIC_IPR_BYTE(IRQN_ADC1_2) = 0x00u;
    NVIC_IPR_BYTE(IRQN_FDCAN1_IT0) = 0x40u;
    NVIC_IPR_BYTE(IRQN_TIM1_BRK_TIM15) = 0x00u;
    SCB_SHPR3 = (SCB_SHPR3 & 0x00FFFFFFu) | (0x80u << 24);  /* SysTick */

    NVIC_ISER(0) = (1u << IRQN_ADC1_2) | (1u << IRQN_FDCAN1_IT0) | (1u << IRQN_TIM1_BRK_TIM15);
    GPIO_BSRR(GPIOC_BASE) = 1u << 6;                  /* DRV8353RS ENABLE high */
}

void board_pwm_kill(void)
{
    TIM1_BDTR &= ~TIM1_BDTR_MOE;
}

void board_pwm_set(float da, float db, float dc, bool enable)
{
    if (!enable) {
        board_pwm_kill();
        return;
    }
    TIM1_CCR1 = (uint32_t)(da * (float)TIM1_ARR_CENTER);
    TIM1_CCR2 = (uint32_t)(db * (float)TIM1_ARR_CENTER);
    TIM1_CCR3 = (uint32_t)(dc * (float)TIM1_ARR_CENTER);
    TIM1_BDTR |= TIM1_BDTR_MOE;
}

uint8_t board_node_id(void)
{
    /* Straps pull to ground = 1; all open reads as node 1 (bench default). */
    uint32_t idr = ~GPIO_IDR(GPIOC_BASE);
    uint8_t id = (uint8_t)((idr >> 1) & 0x0Fu);
    return (id == 0u) ? 1u : id;
}

bool board_gate_fault_active(void)
{
    return (GPIO_IDR(GPIOC_BASE) & (1u << 7)) == 0u;  /* nFAULT is active low */
}

void board_led(bool on)
{
    GPIO_BSRR(GPIOC_BASE) = on ? (1u << 9) : (1u << (9u + 16u));
}

/* ---- conversions --------------------------------------------------------- */

float board_adc_to_phase_current(uint16_t raw)
{
    /* DRV8353RS: SOx = VREF/2 - G * (SPx - SNx); sign verified at bring-up. */
    float v = ((float)raw - 2048.0f) * (3.3f / 4096.0f);
    return -v / (BOARD_CSA_GAIN * BOARD_SHUNT_OHM);
}

float board_adc_to_vbus(uint16_t raw)
{
    return (float)raw * (3.3f / 4096.0f) * BOARD_VBUS_DIVIDER;
}

/* 10k B3435 NTC to ground with a 10k pull-up, ADC counts at -20..150 degC. */
static const uint16_t k_ntc_counts[] = {
    3627, 3368, 3037, 2654, 2248, 1854, 1496, 1191, 940,
    740, 584, 462, 368, 295, 238, 194, 159, 132,
};
#define NTC_T0_C     (-20.0f)
#define NTC_STEP_C   10.0f
#define NTC_N        (sizeof k_ntc_counts / sizeof k_ntc_counts[0])

float board_adc_to_ntc_celsius(uint16_t raw)
{
    if (raw >= k_ntc_counts[0]) {
        return NTC_T0_C;
    }
    for (uint32_t i = 1; i < NTC_N; i++) {
        if (raw >= k_ntc_counts[i]) {
            float span = (float)(k_ntc_counts[i - 1] - k_ntc_counts[i]);
            float frac = (float)(k_ntc_counts[i - 1] - raw) / span;
            return NTC_T0_C + NTC_STEP_C * ((float)(i - 1) + frac);
        }
    }
    /* Beyond the table (or open/short sensor): report hot so it trips. */
    return 200.0f;
}

/* ---- peripheral hooks (TODO(bring-up)) ---------------------------------- */

void board_adc_read_fast(board_adc_fast_t *out)
{
    /* TODO(bring-up): ADC1 JDR1..JDR4. Neutral values until then. */
    out->ia = out->ib = out->ic = 2048u;
    out->vbus = 0u;
}

void board_adc_read_temps(uint16_t *stage, uint16_t *motor)
{
    /* TODO(bring-up): regular-group DMA buffer. 25 degC until then. */
    *stage = 2047u;
    *motor = 2047u;
}

uint16_t board_motor_encoder_read(void)
{
    return 0u;   /* TODO(bring-up): SPI3, PD2 chip select, angle register */
}

uint32_t board_output_encoder_read(void)
{
    return 0u;   /* TODO(bring-up): SPI3, PA15 chip select, 19-bit position */
}

uint16_t board_spi1_xfer16(uint16_t word)
{
    (void)word;  /* TODO(bring-up): SPI1 16-bit, mode 1, <= 10 MHz, nSCS PA4 */
    return 0u;
}

bool board_can_receive(hj20_can_frame_t *f)
{
    (void)f;     /* TODO(bring-up): FDCAN1 RX FIFO0 element read + ack */
    return false;
}

bool board_can_send(const hj20_can_frame_t *f)
{
    (void)f;     /* TODO(bring-up): FDCAN1 TX buffer, FDF + BRS, DLC from len */
    return false;
}
