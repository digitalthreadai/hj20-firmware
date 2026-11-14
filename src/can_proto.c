#include "can_proto.h"

#include "hj20_config.h"

#define TORQUE_MAX_MNM    ((int32_t)(HJ20_PEAK_TORQUE_NM * 1000.0f))
#define VELOCITY_MAX_MRAD ((int32_t)(HJ20_MAX_SPEED_RAD_S * 1000.0f) + 1)
#define POSITION_MAX_URAD 6283185L  /* one output turn either way */

/* ---- little-endian helpers --------------------------------------------- */

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void zero(uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        p[i] = 0;
    }
}

/* ---- DLC ---------------------------------------------------------------- */

static const uint8_t k_dlc_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

uint8_t hj20_can_len_to_dlc(uint8_t len)
{
    for (uint8_t dlc = 0; dlc < 16u; dlc++) {
        if (k_dlc_len[dlc] >= len) {
            return dlc;
        }
    }
    return 15u;
}

uint8_t hj20_can_dlc_to_len(uint8_t dlc)
{
    return k_dlc_len[dlc & 0x0Fu];
}

/* ---- command ------------------------------------------------------------ */

void hj20_can_encode_cmd(uint8_t node, const hj20_cmd_t *cmd, hj20_can_frame_t *f)
{
    f->id = HJ20_CAN_ID_CMD_BASE + node;
    f->len = HJ20_CAN_CMD_LEN;
    zero(f->data, sizeof f->data);
    f->data[0] = (uint8_t)cmd->mode;
    f->data[1] = cmd->seq;
    put_u32(&f->data[4], (uint32_t)cmd->setpoint);
    put_u32(&f->data[8], (uint32_t)cmd->torque_ff_mnm);
    put_u16(&f->data[12], cmd->torque_limit_mnm);
}

static bool in_range(int32_t v, int32_t lim)
{
    return v >= -lim && v <= lim;
}

hj20_can_status_t hj20_can_decode_cmd(uint8_t node, const hj20_can_frame_t *f, hj20_cmd_t *cmd)
{
    if (node == 0u || node > HJ20_CAN_NODE_MAX || f->id != HJ20_CAN_ID_CMD_BASE + node) {
        return HJ20_CAN_ERR_ID;
    }
    if (f->len < HJ20_CAN_CMD_LEN) {
        return HJ20_CAN_ERR_LEN;
    }

    uint8_t mode = f->data[0];
    if (mode > (uint8_t)HJ20_MODE_CLEAR_FAULTS) {
        return HJ20_CAN_ERR_MODE;
    }

    int32_t sp = (int32_t)get_u32(&f->data[4]);
    int32_t ff = (int32_t)get_u32(&f->data[8]);
    uint16_t lim = get_u16(&f->data[12]);

    bool ok = in_range(ff, TORQUE_MAX_MNM) && (int32_t)lim <= TORQUE_MAX_MNM;
    switch ((hj20_mode_t)mode) {
    case HJ20_MODE_TORQUE:   ok = ok && in_range(sp, TORQUE_MAX_MNM); break;
    case HJ20_MODE_VELOCITY: ok = ok && in_range(sp, VELOCITY_MAX_MRAD); break;
    case HJ20_MODE_POSITION: ok = ok && in_range(sp, (int32_t)POSITION_MAX_URAD); break;
    default: break;
    }
    if (!ok) {
        return HJ20_CAN_ERR_RANGE;
    }

    cmd->mode = (hj20_mode_t)mode;
    cmd->seq = f->data[1];
    cmd->setpoint = sp;
    cmd->torque_ff_mnm = ff;
    cmd->torque_limit_mnm = lim;
    return HJ20_CAN_OK;
}

/* ---- telemetry ---------------------------------------------------------- */

void hj20_can_encode_telemetry(uint8_t node, const hj20_telemetry_t *t, hj20_can_frame_t *f)
{
    f->id = HJ20_CAN_ID_TELEM_BASE + node;
    f->len = HJ20_CAN_TELEM_LEN;
    zero(f->data, sizeof f->data);
    f->data[0] = t->state;
    f->data[1] = t->seq_echo;
    put_u16(&f->data[2], t->faults);
    put_u32(&f->data[4], (uint32_t)t->position_urad);
    put_u32(&f->data[8], (uint32_t)t->velocity_mrad_s);
    put_u32(&f->data[12], (uint32_t)t->torque_mnm);
    put_u16(&f->data[16], (uint16_t)t->iq_ma);
    put_u16(&f->data[18], t->vbus_mv);
    put_u16(&f->data[20], (uint16_t)t->temp_stage_dc);
    put_u16(&f->data[22], (uint16_t)t->temp_motor_dc);
    put_u32(&f->data[24], t->uptime_ms);
    f->data[28] = t->fw_major;
    f->data[29] = t->fw_minor;
    f->data[30] = t->fw_patch;
}

hj20_can_status_t hj20_can_decode_telemetry(uint8_t node, const hj20_can_frame_t *f, hj20_telemetry_t *t)
{
    if (f->id != HJ20_CAN_ID_TELEM_BASE + node) {
        return HJ20_CAN_ERR_ID;
    }
    if (f->len < HJ20_CAN_TELEM_LEN) {
        return HJ20_CAN_ERR_LEN;
    }
    t->state = f->data[0];
    t->seq_echo = f->data[1];
    t->faults = get_u16(&f->data[2]);
    t->position_urad = (int32_t)get_u32(&f->data[4]);
    t->velocity_mrad_s = (int32_t)get_u32(&f->data[8]);
    t->torque_mnm = (int32_t)get_u32(&f->data[12]);
    t->iq_ma = (int16_t)get_u16(&f->data[16]);
    t->vbus_mv = get_u16(&f->data[18]);
    t->temp_stage_dc = (int16_t)get_u16(&f->data[20]);
    t->temp_motor_dc = (int16_t)get_u16(&f->data[22]);
    t->uptime_ms = get_u32(&f->data[24]);
    t->fw_major = f->data[28];
    t->fw_minor = f->data[29];
    t->fw_patch = f->data[30];
    return HJ20_CAN_OK;
}
