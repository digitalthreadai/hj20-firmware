#include "fault.h"
#include "hj20_config.h"
#include "hj20_test.h"

static hj20_fault_inputs_t nominal(void)
{
    hj20_fault_inputs_t in = {
        .vbus_v = 48.0f, .temp_stage_c = 45.0f, .temp_motor_c = 50.0f,
        .encoder_mismatch = false, .gate_driver_fault = false,
    };
    return in;
}

/* Power up and enable: INIT -> DISABLED -> ENABLED. */
static void bring_up(hj20_fault_t *f)
{
    hj20_fault_init(f);
    hj20_fault_inputs_t in = nominal();
    hj20_fault_evaluate(f, &in);
    (void)hj20_fault_request_enable(f);
}

static void init_waits_for_bus_voltage(void)
{
    hj20_fault_t f;
    hj20_fault_init(&f);
    hj20_fault_inputs_t in = nominal();
    in.vbus_v = 20.0f;                       /* bus still charging */
    for (int k = 0; k < 100; k++) hj20_fault_evaluate(&f, &in);
    CHECK(f.state == HJ20_STATE_INIT);
    CHECK(f.latched == 0u);                  /* no UV fault during power-up */
    CHECK(!hj20_fault_request_enable(&f));
    in.vbus_v = 40.0f;
    hj20_fault_evaluate(&f, &in);
    CHECK(f.state == HJ20_STATE_DISABLED);
}

static void over_current_trips_in_one_sample(void)
{
    hj20_fault_t f;
    bring_up(&f);
    CHECK(hj20_fault_pwm_allowed(&f));
    hj20_fault_check_current(&f, 2.0f, -15.5f, 13.5f);
    CHECK(f.state == HJ20_STATE_FAULT);
    CHECK((f.latched & HJ20_FAULT_OVER_CURRENT) != 0u);
    CHECK(!hj20_fault_pwm_allowed(&f));
}

static void current_below_trip_is_ignored(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_check_current(&f, 14.9f, -7.0f, -7.9f);
    CHECK(f.state == HJ20_STATE_ENABLED);
}

static void over_voltage_debounced(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.vbus_v = 59.0f;
    for (int k = 1; k < HJ20_OV_DEBOUNCE; k++) {
        hj20_fault_evaluate(&f, &in);
        CHECK(f.state == HJ20_STATE_ENABLED);
    }
    hj20_fault_evaluate(&f, &in);
    CHECK(f.state == HJ20_STATE_FAULT);
    CHECK((f.latched & HJ20_FAULT_OVER_VOLTAGE) != 0u);
}

static void under_voltage_trips_when_running(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.vbus_v = 30.0f;
    for (int k = 0; k < HJ20_UV_DEBOUNCE; k++) hj20_fault_evaluate(&f, &in);
    CHECK(f.state == HJ20_STATE_FAULT);
    CHECK((f.latched & HJ20_FAULT_UNDER_VOLTAGE) != 0u);
}

static void stage_over_temperature_trips(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.temp_stage_c = 112.0f;
    for (int k = 0; k < HJ20_OT_DEBOUNCE; k++) hj20_fault_evaluate(&f, &in);
    CHECK((f.latched & HJ20_FAULT_OVER_TEMP_STAGE) != 0u);
    CHECK(f.state == HJ20_STATE_FAULT);
}

static void winding_over_temperature_trips(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.temp_motor_c = 131.0f;
    for (int k = 0; k < HJ20_OT_DEBOUNCE; k++) hj20_fault_evaluate(&f, &in);
    CHECK((f.latched & HJ20_FAULT_OVER_TEMP_MOTOR) != 0u);
}

static void fault_latches_after_condition_clears(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.gate_driver_fault = true;
    hj20_fault_evaluate(&f, &in);
    in.gate_driver_fault = false;           /* nFAULT released by itself */
    for (int k = 0; k < 100; k++) hj20_fault_evaluate(&f, &in);
    CHECK(f.state == HJ20_STATE_FAULT);
    CHECK(!hj20_fault_request_enable(&f));  /* no silent re-arm */
    CHECK(hj20_fault_clear(&f));
    CHECK(f.state == HJ20_STATE_DISABLED);
    CHECK(f.latched == 0u);
}

static void clear_refused_while_condition_active(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_inputs_t in = nominal();
    in.encoder_mismatch = true;
    hj20_fault_evaluate(&f, &in);
    CHECK(!hj20_fault_clear(&f));
    CHECK(f.state == HJ20_STATE_FAULT);
    in.encoder_mismatch = false;
    hj20_fault_evaluate(&f, &in);
    CHECK(hj20_fault_clear(&f));
}

static void first_fault_recorded(void)
{
    hj20_fault_t f;
    bring_up(&f);
    hj20_fault_check_current(&f, 20.0f, 0.0f, 0.0f);
    hj20_fault_inputs_t in = nominal();
    in.gate_driver_fault = true;            /* consequential fault */
    hj20_fault_evaluate(&f, &in);
    CHECK(f.first == HJ20_FAULT_OVER_CURRENT);
    CHECK((f.latched & HJ20_FAULT_GATE_DRIVER) != 0u);
}

HJ20_SUITE(suite_fault, "fault",
    {TEST_NAME(REQ_FAULT_LATCH, "init_waits_for_bus_voltage"), init_waits_for_bus_voltage},
    {TEST_NAME(REQ_OVER_CURRENT, "over_current_trips_in_one_sample"), over_current_trips_in_one_sample},
    {TEST_NAME(REQ_OVER_CURRENT, "current_below_trip_is_ignored"), current_below_trip_is_ignored},
    {TEST_NAME(REQ_BUS_VOLTAGE, "over_voltage_debounced"), over_voltage_debounced},
    {TEST_NAME(REQ_BUS_VOLTAGE, "under_voltage_trips_when_running"), under_voltage_trips_when_running},
    {TEST_NAME(REQ_OVER_TEMP, "stage_over_temperature_trips"), stage_over_temperature_trips},
    {TEST_NAME(REQ_OVER_TEMP, "winding_over_temperature_trips"), winding_over_temperature_trips},
    {TEST_NAME(REQ_FAULT_LATCH, "fault_latches_after_condition_clears"), fault_latches_after_condition_clears},
    {TEST_NAME(REQ_FAULT_LATCH, "clear_refused_while_condition_active"), clear_refused_while_condition_active},
    {TEST_NAME(REQ_FAULT_LATCH, "first_fault_recorded"), first_fault_recorded},
);
