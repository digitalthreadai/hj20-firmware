# Test to requirement map

Source of truth: [`req_map.h`](req_map.h). A test named
`TEST_NAME(REQ_TORQUE_LIMIT, "torque_clamp_at_peak")` is emitted in the JUnit
report as `test_SRS_104_torque_clamp_at_peak`; the PLM reads `SRS_104` as
requirement **SRS-104** (pattern: 2 to 6 letters, `-` or `_`, 2 to 5 digits)
and matches it against the requirement's spec id (`rec_id`) or item number.

| Macro               | Requirement | Statement (short form)                                   | Tests |
|---------------------|-------------|----------------------------------------------------------|-------|
| `REQ_FOC_TRANSFORM` | SRS-101     | Clarke/Park transforms accurate to 1e-5                  | 5 |
| `REQ_SVPWM`         | SRS-102     | SVPWM duty bounded; linear up to vbus/sqrt(3)            | 5 |
| `REQ_CURRENT_LOOP`  | SRS-103     | dq current loop settles < 1 ms; anti-windup              | 5 |
| `REQ_TORQUE_LIMIT`  | SRS-104     | Output torque limited to 20 Nm rated / 50 Nm peak        | 8 |
| `REQ_SPEED_LIMIT`   | SRS-106     | Output speed limited to 30 rpm                           | 2 |
| `REQ_ENCODER`       | SRS-110     | Output/motor encoder consistency monitored               | 3 |
| `REQ_OVER_CURRENT`  | SRS-120     | Phase over-current trips within one PWM period           | 3 |
| `REQ_BUS_VOLTAGE`   | SRS-121     | Bus over-voltage (58 V) / under-voltage (36 V) trip      | 2 |
| `REQ_OVER_TEMP`     | SRS-122     | Power-stage (110 degC) / winding (130 degC) trip         | 2 |
| `REQ_FAULT_LATCH`   | SRS-123     | Faults latch until explicitly cleared                    | 4 |
| `REQ_CAN_COMMAND`   | SRS-130     | CAN-FD command decode with range validation              | 7 |
| `REQ_CAN_TELEMETRY` | SRS-131     | CAN-FD telemetry content and encoding                    | 2 |

Tests without a requirement macro (e.g. `test_encoder_counts_to_angle`) are
unit-level checks and are intentionally not traced.

## Re-pointing to baselined requirement ids

1. Edit the string values in `req_map.h` only, e.g.
   `#define REQ_TORQUE_LIMIT "SRS_104"` becomes `"HJR_204"` or `"REQ_00042"`.
   Use `_` as the separator and keep the id to 2 to 6 letters plus 2 to 5 digits.
2. Update the table above.
3. Commit as `test: re-point requirement map to <baseline>`; the next CI run
   produces the new names.
