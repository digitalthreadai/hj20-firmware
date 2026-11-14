#include "control.h"

#include "fastmath.h"
#include "hj20_config.h"
#include "torque.h"

/* Current-loop gains from the motor model: R = 0.45 ohm, L = 0.35 mH per
 * phase; pole-zero cancellation at ~2 kHz bandwidth: kp = wc*L, ki = R/L. */
#define CUR_KP_V_PER_A   4.4f
#define CUR_KI_PER_S     1285.0f
#define CUR_KI_V_PER_A_S (CUR_KP_V_PER_A * CUR_KI_PER_S)

static void reset_loops(hj20_ctrl_t *c)
{
    hj20_pi_reset(&c->pi_d);
    hj20_pi_reset(&c->pi_q);
    hj20_motion_reset(&c->motion);
    c->id_ref = 0.0f;
    c->iq_ref = 0.0f;
    c->torque_cmd_nm = 0.0f;
}

void hj20_ctrl_init(hj20_ctrl_t *c, uint8_t node_id, float elec_offset_rad)
{
    c->node_id = node_id;
    hj20_fault_init(&c->fault);
    hj20_encoder_init(&c->enc, elec_offset_rad);
    hj20_motion_init(&c->motion);
    float vmax = hj20_svpwm_linear_limit(HJ20_VBUS_NOMINAL_V);
    hj20_pi_init(&c->pi_d, CUR_KP_V_PER_A, CUR_KI_V_PER_A_S, -vmax, vmax);
    hj20_pi_init(&c->pi_q, CUR_KP_V_PER_A, CUR_KI_V_PER_A_S, -vmax, vmax);

    c->mode = HJ20_MODE_DISABLE;
    c->last_seq = 0;
    c->setpoint = 0.0f;
    c->torque_ff_nm = 0.0f;
    c->torque_limit_nm = HJ20_RATED_TORQUE_NM;
    c->iq_meas = 0.0f;
    c->vbus = 0.0f;
    c->temp_stage_c = 25.0f;
    c->temp_motor_c = 25.0f;
    c->now_ms = 0;
    reset_loops(c);
}

void hj20_ctrl_current_isr(hj20_ctrl_t *c, const hj20_fast_inputs_t *in, hj20_fast_outputs_t *out)
{
    float theta_e = hj20_encoder_motor_update(&c->enc, in->motor_enc_raw);

    hj20_fault_check_current(&c->fault, in->ia, in->ib, in->ic);
    if (!hj20_fault_pwm_allowed(&c->fault)) {
        hj20_pi_reset(&c->pi_d);
        hj20_pi_reset(&c->pi_q);
        out->duty.a = out->duty.b = out->duty.c = 0.5f;
        out->pwm_enable = false;
        return;
    }

    float s, co;
    hj20_sincosf(theta_e, &s, &co);

    hj20_abc_t iabc = {in->ia, in->ib, in->ic};
    hj20_dq_t idq = hj20_park(hj20_clarke(iabc), s, co);
    c->iq_meas = idq.q;

    float vmax = hj20_svpwm_linear_limit(in->vbus) * HJ20_VOLTAGE_HEADROOM;
    c->pi_d.out_min = c->pi_q.out_min = -vmax;
    c->pi_d.out_max = c->pi_q.out_max = vmax;

    hj20_dq_t v;
    v.d = hj20_pi_step(&c->pi_d, c->id_ref - idq.d, HJ20_CURRENT_LOOP_DT_S);
    v.q = hj20_pi_step(&c->pi_q, c->iq_ref - idq.q, HJ20_CURRENT_LOOP_DT_S);
    hj20_dq_limit(&v.d, &v.q, vmax);

    (void)hj20_svpwm(hj20_inv_park(v, s, co), in->vbus, &out->duty);
    out->pwm_enable = true;
}

void hj20_ctrl_slow_tick(hj20_ctrl_t *c, const hj20_slow_inputs_t *in)
{
    c->now_ms = in->now_ms;
    c->vbus = in->vbus;
    c->temp_stage_c = in->temp_stage_c;
    c->temp_motor_c = in->temp_motor_c;

    bool mismatch = hj20_encoder_slow_update(&c->enc, in->output_enc_raw,
                                             c->torque_cmd_nm, HJ20_SLOW_LOOP_DT_S);

    hj20_fault_inputs_t fi = {
        .vbus_v = in->vbus,
        .temp_stage_c = in->temp_stage_c,
        .temp_motor_c = in->temp_motor_c,
        .encoder_mismatch = mismatch,
        .gate_driver_fault = in->gate_driver_fault,
    };
    hj20_fault_evaluate(&c->fault, &fi);

    if (!hj20_fault_pwm_allowed(&c->fault)) {
        reset_loops(c);
        return;
    }

    float limit = c->torque_limit_nm;
    float torque;
    switch (c->mode) {
    case HJ20_MODE_TORQUE:
        torque = c->setpoint;
        break;
    case HJ20_MODE_VELOCITY:
        torque = hj20_motion_velocity(&c->motion, c->setpoint, c->enc.vel_rad_s,
                                      limit, HJ20_SLOW_LOOP_DT_S) + c->torque_ff_nm;
        break;
    case HJ20_MODE_POSITION: {
        float vref = hj20_motion_position(&c->motion, c->setpoint, c->enc.out_pos_rad, 0.0f);
        torque = hj20_motion_velocity(&c->motion, vref, c->enc.vel_rad_s,
                                      limit, HJ20_SLOW_LOOP_DT_S) + c->torque_ff_nm;
        break;
    }
    default:
        torque = 0.0f;
        break;
    }

    c->torque_cmd_nm = hj20_torque_clamp(torque, limit);
    c->id_ref = 0.0f;  /* surface PM motor: no reluctance torque, no field weakening */
    c->iq_ref = hj20_torque_to_iq(c->torque_cmd_nm);
}

hj20_can_status_t hj20_ctrl_on_can(hj20_ctrl_t *c, const hj20_can_frame_t *f)
{
    hj20_cmd_t cmd;
    hj20_can_status_t st = hj20_can_decode_cmd(c->node_id, f, &cmd);
    if (st != HJ20_CAN_OK) {
        return st;
    }
    c->last_seq = cmd.seq;

    switch (cmd.mode) {
    case HJ20_MODE_DISABLE:
        hj20_fault_request_disable(&c->fault);
        c->mode = HJ20_MODE_DISABLE;
        reset_loops(c);
        return HJ20_CAN_OK;
    case HJ20_MODE_CLEAR_FAULTS:
        (void)hj20_fault_clear(&c->fault);
        c->mode = HJ20_MODE_DISABLE;
        return HJ20_CAN_OK;
    case HJ20_MODE_TORQUE:
        c->setpoint = (float)cmd.setpoint * 1e-3f;
        break;
    case HJ20_MODE_VELOCITY:
        c->setpoint = (float)cmd.setpoint * 1e-3f;
        break;
    case HJ20_MODE_POSITION:
        c->setpoint = (float)cmd.setpoint * 1e-6f;
        break;
    default:
        return HJ20_CAN_ERR_MODE;
    }

    if (c->mode != cmd.mode) {
        hj20_motion_reset(&c->motion);
    }
    c->mode = cmd.mode;
    c->torque_ff_nm = (float)cmd.torque_ff_mnm * 1e-3f;
    c->torque_limit_nm = (cmd.torque_limit_mnm == 0u)
                             ? HJ20_RATED_TORQUE_NM
                             : (float)cmd.torque_limit_mnm * 1e-3f;
    (void)hj20_fault_request_enable(&c->fault);
    return HJ20_CAN_OK;
}

static int32_t round_i32(float x)
{
    return (int32_t)(x + ((x >= 0.0f) ? 0.5f : -0.5f));
}

void hj20_ctrl_build_telemetry(const hj20_ctrl_t *c, hj20_can_frame_t *f)
{
    hj20_telemetry_t t;
    t.state = (uint8_t)c->fault.state;
    t.seq_echo = c->last_seq;
    t.faults = c->fault.latched;
    t.position_urad = round_i32(c->enc.out_pos_rad * 1e6f);
    t.velocity_mrad_s = round_i32(c->enc.vel_rad_s * 1e3f);
    t.torque_mnm = round_i32(hj20_iq_to_torque(c->iq_meas) * 1e3f);
    t.iq_ma = (int16_t)round_i32(c->iq_meas * 1e3f);
    t.vbus_mv = (uint16_t)round_i32(hj20_clampf(c->vbus, 0.0f, 65.0f) * 1e3f);
    t.temp_stage_dc = (int16_t)round_i32(c->temp_stage_c * 10.0f);
    t.temp_motor_dc = (int16_t)round_i32(c->temp_motor_c * 10.0f);
    t.uptime_ms = c->now_ms;
    t.fw_major = (uint8_t)HJ20_FW_VERSION_MAJOR;
    t.fw_minor = (uint8_t)HJ20_FW_VERSION_MINOR;
    t.fw_patch = (uint8_t)HJ20_FW_VERSION_PATCH;
    hj20_can_encode_telemetry(c->node_id, &t, f);
}
