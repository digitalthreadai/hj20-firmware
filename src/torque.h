/**
 * @file torque.h
 * @brief Output torque <-> motor current conversion and torque limiting.
 *
 * Output torque T_out = Kt * iq * N * eta, with N = 100 and eta = 0.80, so
 * the 50 Nm peak corresponds to iq = 7.81 A (well inside the 15 A trip).
 */
#ifndef HJ20_TORQUE_H
#define HJ20_TORQUE_H

#include <stdbool.h>

/** q-axis current (A) needed for an output torque (Nm). */
float hj20_torque_to_iq(float torque_nm);

/** Output torque (Nm) produced by a q-axis current (A). */
float hj20_iq_to_torque(float iq_a);

/**
 * Clamp a torque command to +/- `limit_nm`; the limit itself is never
 * allowed above the peak rating, so a bad CAN limit cannot raise it.
 */
float hj20_torque_clamp(float torque_nm, float limit_nm);

/**
 * Torque derating state.
 *
 * Peak budget (I2t): torque above the 20 Nm continuous rating consumes a
 * budget of (50^2 - 20^2) * 2 s, i.e. 2 s at full peak from cold. When it
 * is spent the limit folds back to the rated torque until the budget has
 * recovered to HJ20_I2T_RELEASE_FRACTION (recovery only below rated torque).
 *
 * Thermal: the limit scales linearly to zero between the derate start and
 * end temperatures of the power stage and of the winding (the lower of
 * the two scales wins), ahead of the hard over-temperature trips.
 */
typedef struct {
    float i2t_acc;      /* Nm^2 s consumed above the continuous rating */
    bool folded_back;
} hj20_derate_t;

void hj20_derate_init(hj20_derate_t *d);

/** Linear thermal scale in [0, 1]: 1 at or below `start`, 0 at or above `end`. */
float hj20_thermal_scale(float temp_c, float start_c, float end_c);

/**
 * One slow-loop step: account for the torque just commanded and return the
 * torque limit to apply next, never above `requested_limit_nm`.
 */
float hj20_derate_limit(hj20_derate_t *d, float requested_limit_nm, float torque_cmd_nm,
                        float temp_stage_c, float temp_motor_c, float dt);

#endif /* HJ20_TORQUE_H */
