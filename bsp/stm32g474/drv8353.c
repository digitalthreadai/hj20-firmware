#include "drv8353.h"

#include "board.h"

/* Driver Control (0x02) */
#define DRV_CTRL_PWM_MODE_6X   (0u << 5)
#define DRV_CTRL_CLR_FLT       (1u << 0)

/* OCP Control (0x05) */
#define OCP_DEAD_TIME_100NS    (1u << 8)
#define OCP_MODE_LATCHED       (0u << 6)
#define OCP_DEG_4US            (2u << 4)
#define OCP_VDS_LVL            0x9u      /* TODO(bring-up): size from FET Rds(on) at 15 A */

/* CSA Control (0x06) */
#define CSA_VREF_DIV           (1u << 9) /* bidirectional: bias at VREF/2 */
#define CSA_GAIN_10            (1u << 6)
#define CSA_SEN_LVL_1V         (3u << 0)

uint16_t drv8353_frame(bool read, uint8_t addr, uint16_t data)
{
    return (uint16_t)((read ? 0x8000u : 0u) | ((uint16_t)(addr & 0x0Fu) << 11) | (data & 0x07FFu));
}

static uint16_t reg_read(uint8_t addr)
{
    return board_spi1_xfer16(drv8353_frame(true, addr, 0)) & 0x07FFu;
}

static bool reg_write_verify(uint8_t addr, uint16_t value)
{
    (void)board_spi1_xfer16(drv8353_frame(false, addr, value));
    return reg_read(addr) == (value & 0x07FFu);
}

bool drv8353_init(void)
{
    bool ok = true;
    ok &= reg_write_verify(DRV8353_REG_DRIVER_CTRL, DRV_CTRL_PWM_MODE_6X);
    ok &= reg_write_verify(DRV8353_REG_OCP_CTRL,
                           OCP_DEAD_TIME_100NS | OCP_MODE_LATCHED | OCP_DEG_4US | OCP_VDS_LVL);
    ok &= reg_write_verify(DRV8353_REG_CSA_CTRL, CSA_VREF_DIV | CSA_GAIN_10 | CSA_SEN_LVL_1V);
    /* Gate drive strength (0x03/0x04) stays at reset defaults until the
     * switching-node ringing has been measured. */
    (void)board_spi1_xfer16(drv8353_frame(false, DRV8353_REG_DRIVER_CTRL,
                                          DRV_CTRL_PWM_MODE_6X | DRV_CTRL_CLR_FLT));
    return ok;
}

bool drv8353_read_faults(uint16_t *status1, uint16_t *status2)
{
    *status1 = reg_read(DRV8353_REG_FAULT_STATUS1);
    *status2 = reg_read(DRV8353_REG_VGS_STATUS2);
    return ((*status1 | *status2) & 0x07FFu) != 0u;
}
