# HJ-20 actuator firmware

Firmware for the **HJ-20 cobot wrist joint actuator** by Helix Robotics: a
frameless BLDC motor driving a 100:1 strain-wave gear, 48 V, **20 Nm rated /
50 Nm peak** output torque, **30 rpm** output speed, CAN-FD, absolute output
encoder plus motor encoder.

> Status: **skeleton, v0.x**. The control core is host-tested; the STM32 board
> layer links and boots but its peripheral drivers are `TODO(bring-up)` stubs
> that hold the drive in a safe state. Not qualified on hardware: do not
> connect a motor.

## Controller hardware

| Function            | Part                                   |
|---------------------|----------------------------------------|
| MCU                 | STM32G474RE (Cortex-M4F, 170 MHz)      |
| Gate driver         | TI DRV8353RS (SPI, 3 current-sense amps) |
| Power stage         | 6 N-channel MOSFETs, low-side shunts (5 mOhm) |
| CAN-FD transceiver  | TI TCAN1044                            |
| Motor encoder       | 14-bit on-axis magnetic (SPI)          |
| Output encoder      | 19-bit absolute (SPI/SSI)              |

The full pin map is in [`bsp/stm32g474/board.h`](bsp/stm32g474/board.h).

## Architecture

```
src/                  portable control core (C11, no HAL, no heap, no libc)
  hj20_config.h       every plant parameter and safety limit, in one file
  fastmath.[ch]       wrap, sin/cos without libm
  foc.[ch]            Clarke / Park / inverse, SVPWM
  pi.[ch]             PI controller, dq voltage limiter
  motion.[ch]         velocity PI, position P (30 rpm clamp)
  torque.[ch]         torque <-> iq, torque limits, peak budget + thermal derating
  encoder.[ch]        motor + output encoders, consistency monitor
  fault.[ch]          drive state machine, latched faults
  can_proto.[ch]      CAN-FD frame codec
  control.[ch]        joint controller tying the above together
bsp/stm32g474/        board layer: startup, vector table, linker script,
                      clocks, pins, DRV8353RS, interrupt wiring (main.c)
tests/                host unit tests, self-contained runner, JUnit output
tools/gen_sbom.py     SPDX 2.3 SBOM + dependency-graph snapshot
cmake/                arm-none-eabi toolchain file
```

Control cascade and rates:

```
CAN command --> position P --> velocity PI --> torque limit --> iq ref
 (FDCAN ISR)      1 kHz           1 kHz           1 kHz           |
                                                                  v
            phase currents --> Clarke/Park --> dq PI --> inv Park --> SVPWM --> TIM1
                (ADC ISR, 20 kHz, triggered at the PWM valley)
```

Output torque relates to motor current as `T = Kt * iq * N * eta` with
`Kt = 0.08 Nm/A`, `N = 100`, `eta = 0.80`: 20 Nm is 3.13 A, 50 Nm is 7.81 A.

## Build

Requirements: CMake 3.20+, a host C compiler (gcc or clang) and, for the
target image, the GNU Arm toolchain (`gcc-arm-none-eabi`).

```sh
# host build + unit tests (writes build-host/test-results/junit.xml)
cmake -S . -B build-host -DCMAKE_BUILD_TYPE=Debug -DHJ20_WERROR=ON
cmake --build build-host
ctest --test-dir build-host --output-on-failure

# STM32G474RE image: build-arm/bsp/stm32g474/hj20-firmware.{elf,bin,map}
cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-arm
```

The image links with `-nostdlib`: no newlib, no libgcc, no vendor HAL, so
nothing third-party is in the binary. Flash with any SWD probe, e.g.
`st-flash write hj20-firmware.bin 0x08000000`.

## Tests and requirement traceability

`tests/` holds a zero-dependency test runner. Every test that verifies a
software requirement is named `test_<REQ-ID>_<behaviour>`, for example
`test_SRS_104_torque_clamp_at_peak`; the ids live in one header,
[`tests/req_map.h`](tests/req_map.h), described in
[`tests/requirements-map.md`](tests/requirements-map.md). CI uploads the
JUnit report as the `junit-results` artifact.

## CAN-FD protocol

11-bit identifiers, 1 Mbit/s arbitration, 5 Mbit/s data phase (BRS),
little-endian payloads. `node` is the joint address 1 to 15 (board straps).

| ID            | Direction    | Bytes | Frame     |
|---------------|--------------|-------|-----------|
| 0x080 + node  | joint to host | 8    | FAULT (once per latched fault event) |
| 0x100 + node  | host to joint | 16   | COMMAND   |
| 0x180 + node  | joint to host | 32   | TELEMETRY (every 10 ms) |

COMMAND: `[0]` mode (0 disable, 1 torque, 2 velocity, 3 position, 4 clear
faults), `[1]` sequence, `[4..7]` setpoint int32 (mNm, mrad/s or urad),
`[8..11]` torque feed-forward int32 mNm, `[12..13]` torque limit uint16 mNm
(0 = rated 20 Nm). Frames outside the actuator rating are rejected, not clamped.

FAULT: `[0..1]` latched fault bits, `[2..3]` root-cause bit, `[4..7]` uptime
ms. Eight bytes, so classic-CAN monitors can read it; its ID out-ranks every
COMMAND.

TELEMETRY: `[0]` state, `[1]` sequence echo, `[2..3]` latched fault bits,
`[4..7]` output position urad, `[8..11]` velocity mrad/s, `[12..15]` torque
mNm, `[16..17]` iq mA, `[18..19]` bus mV, `[20..23]` stage / winding
temperature 0.1 degC, `[24..27]` uptime ms, `[28..30]` firmware version.

## Safety limits

| Limit                       | Value            | Action                     |
|-----------------------------|------------------|----------------------------|
| Output torque, default      | 20 Nm            | clamp                      |
| Output torque, ceiling      | 50 Nm            | clamp; CAN request > 50 Nm rejected |
| Peak torque budget          | 50 Nm for 2 s    | fold back to 20 Nm, re-arm at 50 % budget |
| Thermal derating            | 85-105 degC stage, 100-125 degC winding | linear torque reduction to zero |
| Output speed                | 30 rpm           | clamp; CAN request rejected |
| Phase current               | 15 A             | trip in one PWM period (latched) |
| Bus over-voltage            | 58 V, 3 ms       | trip (latched)             |
| Bus under-voltage           | 36 V, 10 ms      | trip (latched); power-up waits for 38 V |
| Power stage temperature     | 110 degC, 50 ms  | trip (latched)             |
| Winding temperature         | 130 degC, 50 ms  | trip (latched)             |
| Encoder disagreement        | 0.5 deg + wind-up, 10 ms | trip (latched)     |
| DRV8353RS nFAULT / OCP      | hardware         | TIM1 break cuts gates; fault latched |
| Host command timeout        | 100 ms           | torque off (latched CAN_TIMEOUT) |

A latched fault keeps the bridge off until a CLEAR command arrives **and**
the cause is gone.

## CI

`.github/workflows/ci.yml` runs on every push, pull request and `v*` tag:
host build + tests (`junit-results`), cross build (`hj20-firmware`: elf, bin,
map), SBOM (`sbom-spdx`, also submitted to the dependency graph), and for tags a
GitHub Release with the binary, ELF, SBOM and checksums. Release notes are
taken from this repository's `CHANGELOG.md`.

## License

Proprietary. Copyright (c) 2026 Helix Robotics (fictional demonstration
company, DigitalEng.ai). All rights reserved. See [LICENSE](LICENSE).
