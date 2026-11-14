/**
 * @file hj20_config.h
 * @brief HJ-20 wrist joint actuator: plant parameters and safety limits.
 *
 * Every number the control core relies on lives here so a parameter review
 * is one file. Torques are OUTPUT-side (after the 100:1 strain-wave gear)
 * unless the name says MOTOR. Currents are phase-peak amps in the
 * amplitude-invariant dq frame.
 */
#ifndef HJ20_CONFIG_H
#define HJ20_CONFIG_H

/* ---- mechanics --------------------------------------------------------- */
#define HJ20_GEAR_RATIO               100.0f   /* strain-wave reduction       */
#define HJ20_GEAR_EFFICIENCY          0.80f    /* at rated load, 25 degC      */
#define HJ20_GEAR_STIFFNESS_NM_PER_RAD 25000.0f /* torsional, output side     */

/* ---- frameless BLDC motor ---------------------------------------------- */
#define HJ20_POLE_PAIRS               14
#define HJ20_KT_NM_PER_A              0.080f   /* motor torque per amp of iq  */

/* ---- output ratings ----------------------------------------------------- */
#define HJ20_RATED_TORQUE_NM          20.0f    /* continuous                  */
#define HJ20_PEAK_TORQUE_NM           50.0f    /* short-term                  */
#define HJ20_MAX_SPEED_RPM            30.0f    /* output shaft                */
#define HJ20_MAX_SPEED_RAD_S          (HJ20_MAX_SPEED_RPM * 0.104719755f)

/* ---- power stage (DRV8353RS + 6 MOSFETs, 48 V bus) ---------------------- */
#define HJ20_VBUS_NOMINAL_V           48.0f
#define HJ20_VBUS_OV_TRIP_V           58.0f
#define HJ20_VBUS_UV_TRIP_V           36.0f
#define HJ20_VBUS_UV_RELEASE_V        38.0f    /* hysteresis for power-up     */
#define HJ20_PHASE_OC_TRIP_A          15.0f
#define HJ20_PWM_FREQ_HZ              20000
#define HJ20_DUTY_MIN                 0.02f    /* low-side shunt sample window */
#define HJ20_DUTY_MAX                 0.98f
#define HJ20_VOLTAGE_HEADROOM         0.95f    /* of the linear SVPWM limit   */

/* ---- thermal ------------------------------------------------------------ */
#define HJ20_TEMP_STAGE_TRIP_C        110.0f   /* MOSFET / board NTC          */
#define HJ20_TEMP_MOTOR_TRIP_C        130.0f   /* winding NTC                 */

/* ---- sensors ------------------------------------------------------------ */
#define HJ20_MOTOR_ENC_BITS           14       /* on-axis magnetic, SPI       */
#define HJ20_OUTPUT_ENC_BITS          19       /* absolute output, SPI/SSI    */
#define HJ20_ENC_MISMATCH_TOL_RAD     0.0087f  /* 0.5 deg at the output       */
#define HJ20_ENC_MISMATCH_SAMPLES     10       /* consecutive slow ticks      */

/* ---- loop rates --------------------------------------------------------- */
#define HJ20_CURRENT_LOOP_DT_S        (1.0f / (float)HJ20_PWM_FREQ_HZ)
#define HJ20_SLOW_LOOP_HZ             1000
#define HJ20_SLOW_LOOP_DT_S           (1.0f / (float)HJ20_SLOW_LOOP_HZ)

/* ---- debounce (slow-loop samples) ---------------------------------------- */
#define HJ20_OV_DEBOUNCE              3
#define HJ20_UV_DEBOUNCE              10
#define HJ20_OT_DEBOUNCE              50

#endif /* HJ20_CONFIG_H */
