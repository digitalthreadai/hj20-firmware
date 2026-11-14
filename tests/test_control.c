#include "control.h"
#include "fastmath.h"
#include "hj20_config.h"
#include "hj20_test.h"
#include "torque.h"

static hj20_slow_inputs_t slow_nominal(uint32_t now_ms)
{
    hj20_slow_inputs_t in = {
        .vbus = 48.0f, .temp_stage_c = 40.0f, .temp_motor_c = 45.0f,
        .output_enc_raw = 0, .gate_driver_fault = false, .now_ms = now_ms,
    };
    return in;
}

static void ctrl_ready(hj20_ctrl_t *c)
{
    hj20_ctrl_init(c, 1, 0.0f);
    hj20_fast_inputs_t fi = {0.0f, 0.0f, 0.0f, 48.0f, 0};
    hj20_fast_outputs_t fo;
    hj20_ctrl_current_isr(c, &fi, &fo);
    hj20_slow_inputs_t si = slow_nominal(1);
    hj20_ctrl_slow_tick(c, &si);
}

static void send(hj20_ctrl_t *c, hj20_mode_t mode, int32_t sp, uint16_t limit_mnm)
{
    hj20_cmd_t cmd = {mode, 1, sp, 0, limit_mnm};
    hj20_can_frame_t f;
    hj20_can_encode_cmd(c->node_id, &cmd, &f);
    CHECK(hj20_ctrl_on_can(c, &f) == HJ20_CAN_OK);
}

static void torque_command_maps_to_iq_reference(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    CHECK(c.fault.state == HJ20_STATE_DISABLED);

    send(&c, HJ20_MODE_TORQUE, 10000, 0);          /* 10 Nm, default limit */
    hj20_slow_inputs_t si = slow_nominal(2);
    hj20_ctrl_slow_tick(&c, &si);
    CHECK(c.fault.state == HJ20_STATE_ENABLED);
    CHECK_NEAR(c.iq_ref, 1.5625f, 1e-5);
}

static void default_limit_is_rated_torque(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 40000, 0);          /* 40 Nm with no explicit limit */
    hj20_slow_inputs_t si = slow_nominal(2);
    hj20_ctrl_slow_tick(&c, &si);
    CHECK_NEAR(c.torque_cmd_nm, HJ20_RATED_TORQUE_NM, 1e-4);
    CHECK_NEAR(c.iq_ref, hj20_torque_to_iq(HJ20_RATED_TORQUE_NM), 1e-5);
}

static void explicit_limit_allows_peak(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 45000, 50000);
    hj20_slow_inputs_t si = slow_nominal(2);
    hj20_ctrl_slow_tick(&c, &si);
    CHECK_NEAR(c.torque_cmd_nm, 45.0f, 1e-3);
}

static void hot_power_stage_reduces_torque(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 40000, 50000);
    hj20_slow_inputs_t si = slow_nominal(2);
    si.temp_stage_c = 95.0f;                       /* half-way down the derate ramp */
    hj20_ctrl_slow_tick(&c, &si);
    CHECK_NEAR(c.torque_cmd_nm, 25.0f, 1e-3);
}

static void isr_disables_pwm_on_over_current(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 5000, 0);
    hj20_fast_inputs_t fi = {1.0f, -0.5f, -0.5f, 48.0f, 0};
    hj20_fast_outputs_t fo;
    hj20_ctrl_current_isr(&c, &fi, &fo);
    CHECK(fo.pwm_enable);
    fi.ia = 18.0f;
    hj20_ctrl_current_isr(&c, &fi, &fo);
    CHECK(!fo.pwm_enable);
    CHECK(c.fault.state == HJ20_STATE_FAULT);
}

static void current_loop_drives_q_axis_voltage(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 10000, 0);
    hj20_slow_inputs_t si = slow_nominal(2);
    hj20_ctrl_slow_tick(&c, &si);
    /* Rotor at electrical zero, zero measured current: the output voltage
     * vector must point along +q (= +beta at theta 0), so phase B leads. */
    hj20_fast_inputs_t fi = {0.0f, 0.0f, 0.0f, 48.0f, 0};
    hj20_fast_outputs_t fo;
    hj20_ctrl_current_isr(&c, &fi, &fo);
    CHECK(fo.pwm_enable);
    CHECK(fo.duty.b > fo.duty.a && fo.duty.a > fo.duty.c);
    CHECK_NEAR(fo.duty.a, 0.5f, 1e-4);
}

static void disable_command_stops_torque(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 10000, 0);
    send(&c, HJ20_MODE_DISABLE, 0, 0);
    hj20_slow_inputs_t si = slow_nominal(3);
    hj20_ctrl_slow_tick(&c, &si);
    CHECK(c.fault.state == HJ20_STATE_DISABLED);
    CHECK_NEAR(c.iq_ref, 0.0f, 0.0);
}

static void command_timeout_stops_torque(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    hj20_slow_inputs_t si = slow_nominal(2);
    hj20_ctrl_slow_tick(&c, &si);
    send(&c, HJ20_MODE_TORQUE, 10000, 0);          /* last command at t = 2 ms */
    for (uint32_t t = 3; t <= 2u + HJ20_CAN_CMD_TIMEOUT_MS; t++) {
        si = slow_nominal(t);
        hj20_ctrl_slow_tick(&c, &si);
    }
    CHECK(c.fault.state == HJ20_STATE_ENABLED);    /* exactly 100 ms: still ok */
    si = slow_nominal(3u + HJ20_CAN_CMD_TIMEOUT_MS);
    hj20_ctrl_slow_tick(&c, &si);
    CHECK(c.fault.state == HJ20_STATE_FAULT);
    CHECK((c.fault.latched & HJ20_FAULT_CAN_TIMEOUT) != 0u);
    CHECK_NEAR(c.iq_ref, 0.0f, 0.0);
}

static void periodic_commands_keep_drive_enabled(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    for (uint32_t t = 2; t < 2000; t++) {
        if (t % 20u == 2u) {
            send(&c, HJ20_MODE_TORQUE, 10000, 0);  /* 50 Hz host rate */
        }
        hj20_slow_inputs_t si = slow_nominal(t);
        hj20_ctrl_slow_tick(&c, &si);
    }
    CHECK(c.fault.state == HJ20_STATE_ENABLED);
}

static void fault_event_reported_once(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    send(&c, HJ20_MODE_TORQUE, 5000, 0);
    hj20_can_frame_t f;
    CHECK(!hj20_ctrl_take_fault_event(&c, &f));
    hj20_fault_check_current(&c.fault, 0.0f, 16.0f, -16.0f);
    CHECK(hj20_ctrl_take_fault_event(&c, &f));
    CHECK(!hj20_ctrl_take_fault_event(&c, &f));   /* same event, no repeat */

    hj20_fault_frame_t e;
    CHECK(hj20_can_decode_fault(1, &f, &e) == HJ20_CAN_OK);
    CHECK(e.first == HJ20_FAULT_OVER_CURRENT);

    send(&c, HJ20_MODE_CLEAR_FAULTS, 0, 0);
    CHECK(c.fault.state == HJ20_STATE_DISABLED);
    CHECK(!hj20_ctrl_take_fault_event(&c, &f));
    send(&c, HJ20_MODE_TORQUE, 5000, 0);
    hj20_fault_check_current(&c.fault, 20.0f, 0.0f, 0.0f);
    CHECK(hj20_ctrl_take_fault_event(&c, &f));    /* a new event is reported */
}

static void telemetry_reports_state_and_version(void)
{
    hj20_ctrl_t c;
    ctrl_ready(&c);
    hj20_can_frame_t f;
    hj20_ctrl_build_telemetry(&c, &f);
    hj20_telemetry_t t;
    CHECK(hj20_can_decode_telemetry(1, &f, &t) == HJ20_CAN_OK);
    CHECK(t.state == (uint8_t)HJ20_STATE_DISABLED);
    CHECK(t.vbus_mv == 48000u);
    CHECK(t.fw_major == HJ20_FW_VERSION_MAJOR && t.fw_minor == HJ20_FW_VERSION_MINOR);
}

HJ20_SUITE(suite_control, "control",
    {TEST_NAME(REQ_TORQUE_LIMIT, "torque_command_maps_to_iq_reference"), torque_command_maps_to_iq_reference},
    {TEST_NAME(REQ_TORQUE_LIMIT, "default_limit_is_rated_torque"), default_limit_is_rated_torque},
    {TEST_NAME(REQ_TORQUE_LIMIT, "explicit_limit_allows_peak"), explicit_limit_allows_peak},
    {TEST_NAME(REQ_TORQUE_DERATE, "hot_power_stage_reduces_torque"), hot_power_stage_reduces_torque},
    {TEST_NAME(REQ_OVER_CURRENT, "isr_disables_pwm_on_over_current"), isr_disables_pwm_on_over_current},
    {TEST_NAME(REQ_CURRENT_LOOP, "current_loop_drives_q_axis_voltage"), current_loop_drives_q_axis_voltage},
    {"test_control_disable_command_stops_torque", disable_command_stops_torque},
    {TEST_NAME(REQ_CAN_TIMEOUT, "command_timeout_stops_torque"), command_timeout_stops_torque},
    {TEST_NAME(REQ_CAN_TIMEOUT, "periodic_commands_keep_drive_enabled"), periodic_commands_keep_drive_enabled},
    {TEST_NAME(REQ_CAN_FAULT_FRAME, "fault_event_reported_once"), fault_event_reported_once},
    {TEST_NAME(REQ_CAN_TELEMETRY, "telemetry_reports_state_and_version"), telemetry_reports_state_and_version},
);
