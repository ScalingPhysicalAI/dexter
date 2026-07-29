# Dexter STM32L552ZET6 requirements

## Product objective

The controller shall operate three stepper axes on STM32L552ZET6, accept the
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
- Limit and E-stop behavior must be validated with motors mechanically unloaded
  before full-force operation.
- Commands received from different interfaces share one motion queue; system
  integration shall prevent conflicting simultaneous controllers.
