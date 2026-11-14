/**
 * @file torque.h
 * @brief Output torque <-> motor current conversion and torque limiting.
 *
 * Output torque T_out = Kt * iq * N * eta, with N = 100 and eta = 0.80, so
 * the 50 Nm peak corresponds to iq = 7.81 A (well inside the 15 A trip).
 */
#ifndef HJ20_TORQUE_H
#define HJ20_TORQUE_H

/** q-axis current (A) needed for an output torque (Nm). */
float hj20_torque_to_iq(float torque_nm);

/** Output torque (Nm) produced by a q-axis current (A). */
float hj20_iq_to_torque(float iq_a);

/**
 * Clamp a torque command to +/- `limit_nm`; the limit itself is never
 * allowed above the peak rating, so a bad CAN limit cannot raise it.
 */
float hj20_torque_clamp(float torque_nm, float limit_nm);

#endif /* HJ20_TORQUE_H */
