/**
 * @file encoder.h
 * @brief Motor + output encoder handling and the dual-encoder consistency check.
 *
 * Motor encoder: 14-bit single-turn absolute, read every PWM period. Its
 * counts are accumulated modulo one OUTPUT revolution (2^14 * 100 counts)
 * so the integer state never overflows on a continuously rotating joint.
 *
 * Output encoder: 19-bit single-turn absolute on the output flange, read in
 * the slow loop. The two must agree (through the gear ratio, plus the
 * torque-dependent wind-up of the strain-wave flexspline); persistent
 * disagreement means a slipped gear, broken magnet or bad read, and is a
 * latched fault.
 */
#ifndef HJ20_ENCODER_H
#define HJ20_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define HJ20_MOTOR_COUNTS_PER_REV   (1L << 14)
#define HJ20_OUTPUT_COUNTS_PER_REV  (1L << 19)
#define HJ20_MOTOR_COUNTS_PER_OUTPUT_REV (HJ20_MOTOR_COUNTS_PER_REV * 100L)

/** Single-turn counts to an angle in [0, 2*pi). */
float hj20_counts_to_rad(uint32_t counts, uint32_t bits);

typedef struct {
    uint16_t last_raw;        /* last 14-bit sample                         */
    int32_t acc;              /* motor counts mod one output rev, [0, N*2^14) */
    int32_t acc_at_last_slow; /* snapshot for velocity estimation           */
    float elec_offset;        /* rad, electrical zero from commissioning    */
    float out_offset;         /* rad, output-vs-motor alignment             */
    float vel_rad_s;          /* filtered output velocity                   */
    float out_pos_rad;        /* output position from the output encoder    */
    uint16_t mismatch_count;
    bool aligned;
    bool primed;
} hj20_encoder_t;

void hj20_encoder_init(hj20_encoder_t *e, float elec_offset_rad);

/** Fast path (PWM rate): ingest a motor sample, return the electrical angle. */
float hj20_encoder_motor_update(hj20_encoder_t *e, uint16_t raw14);

/** Output angle implied by the motor encoder, [0, 2*pi). */
float hj20_encoder_output_from_motor(const hj20_encoder_t *e);

/**
 * Slow path: ingest the output encoder, update velocity, run the consistency
 * check. `torque_nm` is the current output torque estimate (gear wind-up).
 * @return true when the encoders have disagreed for HJ20_ENC_MISMATCH_SAMPLES
 *         consecutive calls.
 */
bool hj20_encoder_slow_update(hj20_encoder_t *e, uint32_t raw19, float torque_nm, float dt);

#endif /* HJ20_ENCODER_H */
