/**
 * @file req_map.h
 * @brief THE single place that binds unit tests to software requirements.
 *
 * Every requirement-verifying test is named TEST_NAME(REQ_x, "what"), which
 * expands at compile time to "test_<ID>_what", e.g. test_SRS_104_torque_clamp_at_peak.
 * The PLM (Kepler) extracts `<2-6 letters>[-_]<2-5 digits>` from each JUnit
 * test-case name, canonicalises it to e.g. SRS-104 and links the result to
 * the requirement whose spec id (rec_id) or item number matches.
 *
 * To re-point tests when the requirement ids are baselined, change ONLY the
 * string values below (keep the letters_digits shape, '_' as separator),
 * update tests/requirements-map.md, and rebuild. No test source changes.
 *
 * Avoid "<letters>_<digits>" in the descriptive part of a test name: it would
 * be read as an extra requirement reference.
 */
#ifndef HJ20_REQ_MAP_H
#define HJ20_REQ_MAP_H

/* FOC and modulation */
#define REQ_FOC_TRANSFORM   "SRS_101"  /* Clarke/Park transforms accurate to 1e-5      */
#define REQ_SVPWM           "SRS_102"  /* SVPWM duty bounded, linear to vbus/sqrt(3)    */
#define REQ_CURRENT_LOOP    "SRS_103"  /* dq current loop: settle < 1 ms, anti-windup   */

/* Torque and speed */
#define REQ_TORQUE_LIMIT    "SRS_104"  /* 20 Nm rated / 50 Nm peak output torque limit  */
#define REQ_SPEED_LIMIT     "SRS_106"  /* output speed limited to 30 rpm                */

/* Sensors */
#define REQ_ENCODER         "SRS_110"  /* output/motor encoder consistency monitor      */

/* Protection */
#define REQ_OVER_CURRENT    "SRS_120"  /* phase over-current trip within one PWM period */
#define REQ_BUS_VOLTAGE     "SRS_121"  /* over/under-voltage trip (58 V / 36 V)         */
#define REQ_OVER_TEMP       "SRS_122"  /* power-stage and winding over-temperature trip */
#define REQ_FAULT_LATCH     "SRS_123"  /* faults latch until explicitly cleared         */

/* CAN-FD interface */
#define REQ_CAN_COMMAND     "SRS_130"  /* command frame decode + range validation       */
#define REQ_CAN_TELEMETRY   "SRS_131"  /* telemetry frame content and encoding          */

#define TEST_NAME(req, what) "test_" req "_" what

#endif /* HJ20_REQ_MAP_H */
