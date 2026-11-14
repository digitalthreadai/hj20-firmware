#include <string.h>

#include "can_proto.h"
#include "hj20_test.h"

static hj20_cmd_t torque_cmd(int32_t mnm)
{
    hj20_cmd_t c = {HJ20_MODE_TORQUE, 7, mnm, -250, 30000};
    return c;
}

static void command_round_trip(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = {HJ20_MODE_POSITION, 200, -1570796, 1500, 45000};
    hj20_can_encode_cmd(3, &in, &f);
    CHECK(f.id == 0x103u);
    CHECK(f.len == HJ20_CAN_CMD_LEN);

    hj20_cmd_t out;
    memset(&out, 0, sizeof out);
    CHECK(hj20_can_decode_cmd(3, &f, &out) == HJ20_CAN_OK);
    CHECK(out.mode == in.mode);
    CHECK(out.seq == in.seq);
    CHECK(out.setpoint == in.setpoint);
    CHECK(out.torque_ff_mnm == in.torque_ff_mnm);
    CHECK(out.torque_limit_mnm == in.torque_limit_mnm);
}

static void command_wire_format_little_endian(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = torque_cmd(0x01020304);
    hj20_can_encode_cmd(1, &in, &f);
    CHECK(f.data[0] == 1u && f.data[1] == 7u);
    CHECK(f.data[4] == 0x04u && f.data[5] == 0x03u && f.data[6] == 0x02u && f.data[7] == 0x01u);
}

static void command_wrong_node_rejected(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = torque_cmd(1000);
    hj20_can_encode_cmd(2, &in, &f);
    hj20_cmd_t out;
    CHECK(hj20_can_decode_cmd(5, &f, &out) == HJ20_CAN_ERR_ID);
}

static void command_torque_above_peak_rejected(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = torque_cmd(50001);
    hj20_can_encode_cmd(1, &in, &f);
    hj20_cmd_t out;
    CHECK(hj20_can_decode_cmd(1, &f, &out) == HJ20_CAN_ERR_RANGE);

    in = torque_cmd(-50000);                 /* exactly peak is legal */
    hj20_can_encode_cmd(1, &in, &f);
    CHECK(hj20_can_decode_cmd(1, &f, &out) == HJ20_CAN_OK);
}

static void command_velocity_above_rated_speed_rejected(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = {HJ20_MODE_VELOCITY, 1, 3500, 0, 0}; /* 33 rpm */
    hj20_can_encode_cmd(1, &in, &f);
    hj20_cmd_t out;
    CHECK(hj20_can_decode_cmd(1, &f, &out) == HJ20_CAN_ERR_RANGE);
}

static void command_short_frame_rejected(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = torque_cmd(1000);
    hj20_can_encode_cmd(1, &in, &f);
    f.len = 8;                               /* classic-CAN sender */
    hj20_cmd_t out;
    CHECK(hj20_can_decode_cmd(1, &f, &out) == HJ20_CAN_ERR_LEN);
}

static void command_unknown_mode_rejected(void)
{
    hj20_can_frame_t f;
    hj20_cmd_t in = torque_cmd(1000);
    hj20_can_encode_cmd(1, &in, &f);
    f.data[0] = 9;
    hj20_cmd_t out;
    CHECK(hj20_can_decode_cmd(1, &f, &out) == HJ20_CAN_ERR_MODE);
}

static void telemetry_round_trip(void)
{
    hj20_telemetry_t in = {
        .state = 2, .seq_echo = 77, .faults = 0x0021,
        .position_urad = -2094395, .velocity_mrad_s = -3141, .torque_mnm = -48250,
        .iq_ma = -7540, .vbus_mv = 47850, .temp_stage_dc = -105, .temp_motor_dc = 1187,
        .uptime_ms = 3600000u, .fw_major = 0, .fw_minor = 1, .fw_patch = 0,
    };
    hj20_can_frame_t f;
    hj20_can_encode_telemetry(4, &in, &f);
    CHECK(f.id == 0x184u);
    CHECK(f.len == HJ20_CAN_TELEM_LEN);

    hj20_telemetry_t out;
    CHECK(hj20_can_decode_telemetry(4, &f, &out) == HJ20_CAN_OK);
    CHECK(out.state == in.state && out.seq_echo == in.seq_echo && out.faults == in.faults);
    CHECK(out.position_urad == in.position_urad);
    CHECK(out.velocity_mrad_s == in.velocity_mrad_s);
    CHECK(out.torque_mnm == in.torque_mnm);
    CHECK(out.iq_ma == in.iq_ma && out.vbus_mv == in.vbus_mv);
    CHECK(out.temp_stage_dc == in.temp_stage_dc && out.temp_motor_dc == in.temp_motor_dc);
    CHECK(out.uptime_ms == in.uptime_ms);
    CHECK(out.fw_minor == 1u);
}

static void fd_dlc_mapping(void)
{
    CHECK(hj20_can_len_to_dlc(8) == 8u);
    CHECK(hj20_can_len_to_dlc(9) == 9u);     /* rounds up to 12 bytes */
    CHECK(hj20_can_len_to_dlc(HJ20_CAN_CMD_LEN) == 10u);
    CHECK(hj20_can_len_to_dlc(HJ20_CAN_TELEM_LEN) == 13u);
    CHECK(hj20_can_len_to_dlc(64) == 15u);
    for (uint8_t d = 0; d < 16u; d++) {
        CHECK(hj20_can_len_to_dlc(hj20_can_dlc_to_len(d)) == d);
    }
}

HJ20_SUITE(suite_can, "can",
    {TEST_NAME(REQ_CAN_COMMAND, "command_round_trip"), command_round_trip},
    {TEST_NAME(REQ_CAN_COMMAND, "command_wire_format_little_endian"), command_wire_format_little_endian},
    {TEST_NAME(REQ_CAN_COMMAND, "command_wrong_node_rejected"), command_wrong_node_rejected},
    {TEST_NAME(REQ_CAN_COMMAND, "command_torque_above_peak_rejected"), command_torque_above_peak_rejected},
    {TEST_NAME(REQ_CAN_COMMAND, "command_velocity_above_rated_speed_rejected"), command_velocity_above_rated_speed_rejected},
    {TEST_NAME(REQ_CAN_COMMAND, "command_short_frame_rejected"), command_short_frame_rejected},
    {TEST_NAME(REQ_CAN_COMMAND, "command_unknown_mode_rejected"), command_unknown_mode_rejected},
    {TEST_NAME(REQ_CAN_TELEMETRY, "telemetry_round_trip"), telemetry_round_trip},
    {"test_can_fd_dlc_mapping", fd_dlc_mapping},
);
