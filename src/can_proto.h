/**
 * @file can_proto.h
 * @brief HJ-20 CAN-FD application protocol (11-bit IDs, little-endian payloads).
 *
 * ID map (node = joint address 1..15, set by strap/NVM):
 *
 *   ID              Dir          Len  Content
 *   0x080 + node    joint->host   8   FAULT     latched bits, first fault, time
 *   0x100 + node    host->joint  16   COMMAND   mode, seq, setpoint, ff, limit
 *   0x180 + node    joint->host  32   TELEMETRY state, faults, pos, vel, torque...
 *
 * Bit rates: 1 Mbit/s arbitration, 5 Mbit/s data (BRS on). Lower IDs win
 * arbitration: fault frames pre-empt commands, commands pre-empt telemetry.
 * The FAULT frame is 8 bytes so classic-CAN monitoring tools can read it.
 *
 * Units on the wire are integers: torque mNm, velocity mrad/s, position urad,
 * voltage mV, temperature 0.1 degC, current mA.
 */
#ifndef HJ20_CAN_PROTO_H
#define HJ20_CAN_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#define HJ20_CAN_ID_FAULT_BASE  0x080u
#define HJ20_CAN_ID_CMD_BASE    0x100u
#define HJ20_CAN_ID_TELEM_BASE  0x180u
#define HJ20_CAN_NODE_MAX       15u

#define HJ20_CAN_CMD_LEN        16u
#define HJ20_CAN_TELEM_LEN      32u
#define HJ20_CAN_FAULT_LEN      8u

typedef struct {
    uint32_t id;
    uint8_t len;          /* payload bytes (0..64) */
    uint8_t data[64];
} hj20_can_frame_t;

typedef enum {
    HJ20_MODE_DISABLE = 0,
    HJ20_MODE_TORQUE = 1,
    HJ20_MODE_VELOCITY = 2,
    HJ20_MODE_POSITION = 3,
    HJ20_MODE_CLEAR_FAULTS = 4,
} hj20_mode_t;

typedef struct {
    hj20_mode_t mode;
    uint8_t seq;
    int32_t setpoint;      /* mNm | mrad/s | urad depending on mode */
    int32_t torque_ff_mnm; /* feed-forward, added after the velocity loop */
    uint16_t torque_limit_mnm; /* 0 = use the rated default */
} hj20_cmd_t;

typedef struct {
    uint8_t state;
    uint8_t seq_echo;
    uint16_t faults;
    int32_t position_urad;
    int32_t velocity_mrad_s;
    int32_t torque_mnm;
    int16_t iq_ma;
    uint16_t vbus_mv;
    int16_t temp_stage_dc;
    int16_t temp_motor_dc;
    uint32_t uptime_ms;
    uint8_t fw_major, fw_minor, fw_patch;
} hj20_telemetry_t;

typedef struct {
    uint16_t latched;     /* hj20_fault_bit_t mask                */
    uint16_t first;       /* root-cause bit of this fault event   */
    uint32_t time_ms;     /* uptime when the event was reported   */
} hj20_fault_frame_t;

typedef enum {
    HJ20_CAN_OK = 0,
    HJ20_CAN_ERR_ID = -1,      /* not addressed to this node / unknown */
    HJ20_CAN_ERR_LEN = -2,
    HJ20_CAN_ERR_MODE = -3,
    HJ20_CAN_ERR_RANGE = -4,   /* setpoint outside the actuator rating */
} hj20_can_status_t;

/** Map a payload length to the CAN-FD DLC code (rounds up to the next valid size). */
uint8_t hj20_can_len_to_dlc(uint8_t len);
uint8_t hj20_can_dlc_to_len(uint8_t dlc);

void hj20_can_encode_cmd(uint8_t node, const hj20_cmd_t *cmd, hj20_can_frame_t *f);
hj20_can_status_t hj20_can_decode_cmd(uint8_t node, const hj20_can_frame_t *f, hj20_cmd_t *cmd);

void hj20_can_encode_telemetry(uint8_t node, const hj20_telemetry_t *t, hj20_can_frame_t *f);
hj20_can_status_t hj20_can_decode_telemetry(uint8_t node, const hj20_can_frame_t *f, hj20_telemetry_t *t);

void hj20_can_encode_fault(uint8_t node, const hj20_fault_frame_t *e, hj20_can_frame_t *f);
hj20_can_status_t hj20_can_decode_fault(uint8_t node, const hj20_can_frame_t *f, hj20_fault_frame_t *e);

#endif /* HJ20_CAN_PROTO_H */
