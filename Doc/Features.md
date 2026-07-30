# Feature status

## Implemented features

| ID | Feature | Interface/module | Status | Notes |
|---|---|---|---|---|
| F-01 | STM32L552ZET6 STM32CubeIDE project and IOC | Project | Done | ELF, HEX, and BIN generated |
| F-02 | Three stepper axes | Motion | Done | Independent STEP/DIR pins |
| F-03 | Coordinated XYZ interpolation | Motion | Done | Shared DDA/Bresenham schedule |
| F-04 | Speed and acceleration handling | Motion | Done | Per-axis settings and coordinated ramp |
| F-05 | G0/G1/G4/G28/G90/G91/G92 commands | G-code | Done | Steps or calibrated millimetres |
| F-06 | Cycle engine and built-in macros | Automation | Done | Non-blocking execution |
| F-07 | USB CDC command interface | Communication | Done | Source-routed responses |
| F-08 | LPUART1 interrupt command interface | Communication | Done | PG7/PG8, 115200 8-N-1, VDDIO2 enabled |
| F-09 | Classic CAN ASCII command interface | Communication | Done | Multi-frame request/response stream |
| F-10 | Runtime CAN command/response IDs | Communication | Done | `CANID`, safe old-ID acknowledgement |
| F-11 | STM32MP2 SocketCAN tool | Tools | Done | Interactive, one-shot, file, ID-change modes |
| F-12 | Desktop PyQt serial controller | Tools | Done | Jog, sync move, cycle, script, settings |
| F-13 | Independent X/Y/Z direction inversion | Settings | Done | `$20-$22` and `DIR` aliases |
| F-14 | Two Z limit inputs | Safety | Done | PC0/PC1 raw state and enforcement |
| F-15 | Limit enable and polarity settings | Safety | Done | Default disabled; `$23/$24` and aliases |
| F-16 | Physical E-stop input | Safety | Done | PC2 EXTI priority 0, compile-time enable macro |
| F-17 | Remote E-stop | Safety | Done | `ESTOP` and `M112` from every command source |
| F-18 | Latched alarm and safe clear | Safety | Done | `CLEAR ALARM`, `ALARM CLEAR`, `$X` |
| F-19 | Startup command helper | Diagnostics | Done | USB/LPUART boot output and `HELP` command |
| F-20 | Connection and engineering documentation | Documentation | Done | Diagram, requirements, phases, feature matrix |
| F-21 | CAN clock, 500 kbit/s timing, status, and internal loopback diagnostic | Diagnostics | Done | `CAN STATUS` and `CAN TEST`; normal mode restored after test |

## Suggested features

| Priority | Suggested feature | Status | Benefit | Proposed acceptance criterion |
|---|---|---|---|---|
| High | Persistent settings in internal flash with schema version and CRC | Suggested | Retains CAN IDs, calibration, direction, and polarity safely | Settings survive power cycle and corrupt records fall back to defaults |
| High | Hardware motor-enable outputs and driver-fault inputs | Suggested | Removes drive torque and detects driver faults | E-stop disables every driver and reports the fault source |
| High | Independent safety-rated E-stop power chain | Suggested | Safety does not depend solely on firmware | Motor energy is removed even with MCU firmware halted |
| High | Independent watchdog and brownout/fault log | Suggested | Recovers and explains field failures | Fault injection produces reset reason and retained event record |
| Medium | Full homing state machine with seek, latch, pull-off, and timeout | Suggested | Repeatable machine coordinates | Each axis homes repeatably and faults on missing switch |
| Medium | Automated hardware-in-loop regression tests | Suggested | Prevents interface and motion regressions | USB/UART/CAN/motion safety suite runs unattended |
| Medium | CAN heartbeat, node ID, sequence number, and access ownership | Suggested | Detects disconnects and competing controllers | Lost heartbeat stops motion under configured policy |
| Medium | Motion look-ahead and junction planning | Suggested | Smoother multi-segment paths | Continuous paths meet configured acceleration limits |
| Medium | Encoder or position-feedback support | Suggested | Detects missed steps and mechanical blockage | Position mismatch raises a controlled fault |
| Medium | CANopen or J1939-compatible application profile | Suggested | Easier system integration | Interoperability test passes with selected ecosystem |
| Low | Secure signed firmware update and rollback | Suggested | Safer deployed upgrades | Invalid image is rejected and previous release boots |
| Low | Telemetry counters and timestamped event history | Suggested | Better diagnostics | Host can read motion, limit, E-stop, CAN, and reset counters |
