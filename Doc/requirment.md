# Dexter STM32L552ZET6 requirements

## Product objective

The controller shall operate three stepper axes and one H-bridge linear actuator on STM32L552ZET6, accept the
same command language from USB CDC, LPUART1, and CAN, execute coordinated XYZ
motion and cycle scripts, and place safety inputs ahead of motion continuity.

## Hardware requirements

| ID | Requirement | Verification |
|---|---|---|
| HW-01 | MCU shall be STM32L552ZET6 in the project target configuration. | CubeIDE target and IOC inspection |
| HW-02 | X/Y/Z STEP and DIR shall use PA0/PA4, PA1/PA5, and PB10/PB11. | Pin inspection and oscilloscope test |
| HW-03 | Z minimum and maximum switches shall use PC0 and PC1 with pull-ups. | GPIO read and motion-stop test |
| HW-04 | E-stop shall use PC2 with a high-priority EXTI path. | Press during three-axis motion |
| HW-05 | LPUART1 shall use PG7/PG8 with VDDIO2 enabled. | UART loop/terminal test |
| HW-06 | FDCAN1 shall use PB9/PB8 through an external CAN transceiver. | CAN analyzer test |
| HW-07 | CAN-H/CAN-L shall be terminated with 120 ohm only at the two bus ends. | Powered-off resistance and wiring inspection |
| HW-08 | All logic interfaces shall share a valid common ground and remain within 3.3 V GPIO ratings. | Electrical inspection |
| HW-09 | Linear actuator ENA, IN1, and IN2 shall use PC6, PC7, and PC8 GPIO outputs through an external H-bridge. | IOC inspection and unloaded actuator test |
| HW-10 | AS5600 Z feedback shall use I2C1 on PB6 SCL and PB7 SDA at address 0x36 with external pull-ups to 3.3 V. | IOC inspection, I2C analyzer, and angle test |

## Functional requirements

| ID | Requirement | Acceptance criterion |
|---|---|---|
| FR-01 | `G0/G1 X... Y... Z...` shall start selected axes together and complete them on one coordinated interpolation schedule. | Equal XYZ request starts and finishes together |
| FR-02 | Commands shall be accepted independently from USB CDC, LPUART1, and CAN. | `?` returns a response on each source interface |
| FR-03 | CAN shall transport newline-terminated ASCII commands across multiple classic-CAN frames. | Long XYZ command split across frames executes once |
| FR-04 | CAN command and response IDs shall be queryable and runtime configurable. | `CANID?` and `CANID 0x620 0x621` pass |
| FR-05 | CAN-ID changes requested over CAN shall acknowledge on the old response ID before switching. | Old-ID acknowledgement is captured by analyzer |
| FR-06 | Axis direction shall be configurable independently. | `$20-$22` and `DIR` commands invert selected axis only |
| FR-07 | Z-limit enforcement shall default OFF and be runtime configurable. | Reset reports `$24=0`; `LIMIT ON/OFF` changes behavior |
| FR-08 | Z-limit polarity shall be runtime configurable. | `$23=0/1` changes active interpretation |
| FR-09 | Physical or commanded E-stop shall immediately stop all STEP generation, stop cycles, clear queued motion, and latch an alarm. | PC2, `ESTOP`, and `M112` tests |
| FR-10 | Alarm clear shall fail while the E-stop input remains active. | `CLEAR ALARM` reports an error while PC2 is held |
| FR-11 | Cycle macros and command files shall run without blocking communication polling. | Macro progress and emergency interruption test |
| FR-12 | Firmware shall print a concise startup helper on USB and LPUART1 and respond to `HELP` on every command interface. | Boot and `HELP` test |
| FR-13 | `M3 S<ms>` shall extend and `M4 S<ms>` shall retract the linear actuator at full DC output, with `S=0` meaning continuous operation. | Direction, enable-level, and timed-stop tests |
| FR-14 | `M5` shall stop the linear actuator immediately from USB, LPUART1, CAN, or a script. | Execute during actuator operation on every interface |
| FR-15 | Hold, STOP, program end, remote E-stop, and physical E-stop shall set the linear-actuator enable and both H-bridge direction outputs low. | Oscilloscope and fault-interruption test |
| FR-16 | AS5600 feedback shall report raw 12-bit angle, magnetic status, relative multi-turn Z position, commanded-position error, and I2C errors. | Rotate through wraparound and query `AS5600?` on USB, UART, and CAN |
| FR-17 | AS5600 feedback zero and direction shall be runtime configurable without changing motor direction. | `AS5600 ZERO`, `AS5600 DIR`, and `$25` tests |
| FR-18 | After a commanded Z move, valid AS5600 error outside the configured tolerance shall produce additional Z-only pulses until measured Z reaches the original target. | Introduce known missed motion and verify encoder convergence without changing the logical target |
| FR-19 | Z correction shall block queued/cycle progression and shall be bounded by correction travel, timeout, no-progress detection, limits, STOP, and E-stop. | Fault injection for every termination path |
| FR-20 | Missing or invalid AS5600 feedback with correction enabled shall stop recovery and latch a Z-feedback alarm. | Disconnect sensor after a Z move and verify alarm/clear behavior |

## Software and build requirements

| ID | Requirement |
|---|---|
| SW-01 | Project shall import and build in STM32CubeIDE using `dexter_l552.ioc`. |
| SW-02 | Firmware build shall produce ELF, HEX, and BIN artifacts without compiler warnings. |
| SW-03 | LPUART1 RX/TX shall use interrupt mode without UART DMA. |
| SW-04 | MP2 CAN tooling shall use Linux SocketCAN and Python 3 standard-library APIs. |
| SW-05 | Desktop serial tooling shall support USB CDC or LPUART1 through PySerial and PyQt5. |
| SW-06 | Runtime CAN-ID changes are not persistent and shall return to compile-time defaults after reset. |

## Safety constraints

- Software E-stop is a motion-control safety layer, not a certified removal of
  motor energy. Production hardware should include an independent safety-rated
  power/enable chain.
- A CAN transceiver is mandatory at both MCU endpoints.
- An external current-rated H-bridge with flyback protection is mandatory for
  the linear actuator; PC6/PC7/PC8 are logic signals only.
- Limit and E-stop behavior must be validated with motors mechanically unloaded
  before full-force operation.
- Commands received from different interfaces share one motion queue; system
  integration shall prevent conflicting simultaneous controllers.
- AS5600 correction is a software recovery layer, not a safety-rated servo.
  Direction, scaling, coupling, tolerance, maximum travel, jam behavior, and all
  stop paths must be validated mechanically unloaded before production use.
