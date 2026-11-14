# Changelog

All notable changes to the HJ-20 actuator firmware. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
semantic versioning. Requirement ids refer to the HJ-20 software
requirements specification (see `tests/requirements-map.md`).

## [Unreleased]

## [0.1.0] - 2026-11-14

First buildable skeleton: host-tested control core plus an STM32G474RE image
that links and boots to a safe (bridge disabled) state.

### Added
- FOC core: amplitude-invariant Clarke/Park transforms, libm-free sin/cos,
  min/max-injection SVPWM with duty clamping (SRS-101, SRS-102).
- dq current PI loops with voltage-vector limiting, output velocity PI and
  position P loops; output speed limited to 30 rpm (SRS-103, SRS-106).
- Output torque limit: 20 Nm default, 50 Nm peak ceiling, torque to iq
  through the 100:1 gear and 80 % efficiency (SRS-104).
- Dual-encoder handling and consistency monitor with gear wind-up tolerance (SRS-110).
- Latched fault state machine: over-current, over/under-voltage,
  over-temperature, encoder mismatch, gate driver (SRS-120 to SRS-123).
- CAN-FD protocol: command (0x100 + node) and telemetry (0x180 + node) frames (SRS-130, SRS-131).
- STM32G474RE board layer: vector table, startup, linker script, 170 MHz
  clock tree, pin map, DRV8353RS SPI configuration.
- Host unit tests with JUnit XML output; CI with host tests, cross build,
  SPDX SBOM and tagged releases.
