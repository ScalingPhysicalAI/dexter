# Dexter STM32L552ZET6 firmware

Self-contained STM32CubeIDE project foundation for STM32L552ZET6 using
STM32CubeL5 V1.6.0.

## Interfaces

- X STEP/DIR: PA0/PA4
- Y STEP/DIR: PA1/PA5
- Z STEP/DIR: PB10/PB11
- Z minimum/maximum limits: PC0/PC1, normally closed, pull-up enabled
- Emergency-stop button: PC2, active-low with pull-up, EXTI priority 0
- FDCAN1 RX/TX: PB8/PB9, classic CAN
- USB CDC DM/DP: PA11/PA12
- LPUART1 debug TX/RX: PG7/PG8, 115200 8-N-1
- TIM2: 50 kHz coordinated-motion interrupt from the 110 MHz timer clock

The CAN pins require an external CAN transceiver. Install 120 ohm termination
only at the two physical ends of the CAN bus.

`can_interface.c` accepts standard 11-bit CAN frames. STM32L552 provides three
hardware elements in RX FIFO 0 and the TX FIFO/queue. `limit_switches.c` treats
HIGH as tripped/open wire.

CAN is also a complete command interface at 500 kbit/s. It uses standard 11-bit
ID `0x600` for received command bytes and ID `0x601` for response bytes by
default. Query the active IDs with `CANID?`, change them at runtime with
`CANID <RX> <TX>`, and restore defaults with `CANID DEFAULT`. A CAN-originated
change is acknowledged on the old response ID before the new IDs become active.
Runtime changes return to the compile-time defaults after reset. Each
classic-CAN frame carries 1-8 raw ASCII bytes; commands can span multiple frames
and must end with `\n`. Responses are split into 8-byte frames and retain the
same CR/LF text used by USB and LPUART1. For example, transmit the bytes of
`G0 X1000 Y1000 Z1000\n` across consecutive ID `0x600` frames and reassemble
the ID `0x601` response. Only one CAN command sender should use the stream at a
time. Override `CAN_COMMAND_RX_ID` and `CAN_COMMAND_TX_ID` at compile time to
change the boot defaults.

Use `CAN STATUS` to report the live FDCAN kernel clock, calculated bitrate,
error counters, last error code, and bus-off state. Use `CAN TEST` from USB,
LPUART1, or CAN to run an internal-controller loopback test; the firmware then
restores normal bus mode automatically.

Limit input polarity is configurable over either command port. Use `$23=0` for
active-low, `$23=1` for active-high, or `$23` to read the current polarity. The
default polarity is active-high, matching normally-closed switches with pull-ups
where an open wire or actuated switch reads HIGH. Limit enforcement is disabled
at every boot; use `LIMIT ON` or `$24=1` to enable it and `LIMIT OFF` or `$24=0`
to disable it.
The same commands are accepted from USB CDC, LPUART1, and the CAN ASCII stream;
the response is returned through the interface that issued the command. LPUART1
uses interrupt-driven receive and transmit (no DMA): PG7 and PG8 use AF8,
leaving PB10/PB11 available for the Z motor. Firmware enables the STM32L552
VDDIO2 domain required by these GPIOG pins. A compact command guide is printed
on both ports at boot and can be printed again with `HELP`.

## Motion and commands

`G0` and `G1` support X, Y, and Z in the same command. A shared DDA/Bresenham
scheduler starts the selected axes together and completes them on the same
master interpolation tick. For example:

```text
G0 X1000 Y1000 Z1000
```

Core commands include `G0`, `G1`, `G4`, `G28`, `G90`, `G91`, `G92`, `M17`,
`M18`, `M84`, `M112`, `ESTOP`, `ESTOP?`, `ESTOP RESET`, `CLEAR ALARM`, `?`,
`!`, `~`, `$H`, `$X`, and `HELP`. Units are raw
steps by default; `$14=1` enables calibrated millimetres.

`M112` or `ESTOP` from USB CDC, LPUART1, or a script immediately stops every
axis, stops the cycle engine, clears queued motion, and latches the controller in
alarm. The PC2 button performs the motor stop directly in its high-priority EXTI
handler. Release the button and send `CLEAR ALARM`, `ESTOP RESET`, or `$X`
before moving again. Alarm clearing is rejected while the PC2 button is active.
Set `ESTOP_BUTTON_ENABLE` to `0` in `Core/Inc/main.h` to compile out physical
button handling; remote emergency-stop commands remain enabled.

Direction and limit settings are runtime settings:

- `$20=0/1`: normal/inverted X positive direction
- `$21=0/1`: normal/inverted Y positive direction
- `$22=0/1`: normal/inverted Z positive direction
- `$23=0/1`: active-low/active-high Z limit inputs
- `$24=0/1`: disable/enable Z limit enforcement; default is disabled
- `DIR X|Y|Z NORMAL|REVERSE|TOGGLE`: readable motor-direction command
- `LIMIT ON|OFF`: readable limit enable/disable command
- `$20` through `$24`, `DIR?`, or `LIMIT?`: query settings
- `$`: print all motion and calibration settings

The Z minimum and maximum inputs are checked in the timer ISR before every Z
step. A limit event stops the coordinated move and reports `ALARM:Z limit` on
both command interfaces. Clear it with `$X` after moving the mechanism to a
safe condition.

The `?` status response includes `LimZ:min,max`, making it possible to verify
both switches and the selected polarity before energizing the motor drivers.

## Cycle engine and host scripts

The built-in cycle engine provides `INIT`, `KITCHEN`, `LIVING_ROOM`, `BEDROOM`,
`DOOR`, `CHARGE`, `PATROL`, and `HEIGHT_CAL` scripts. Use `LIST`, `RUN <name>`,
`MACRO <name>`, and `STOP`. It runs non-blocking and waits for each coordinated
move or dwell before advancing.

The desktop controller in `Tools/humanoid_control.py` supports USB CDC or
LPUART1, XYZ jog, synchronized targets, macro control, queued multi-line
scripts, settings, live status, and emergency stop. Installation instructions
are in `Tools/README.md`.

Open `dexter_l552.ioc` or import this folder as an existing STM32CubeIDE
project. Regenerating from the IOC may update generated HAL and USB files; keep
the user application modules in the marked USER CODE sections.

Project documentation is in [`Doc`](Doc/connection_overview.md), including the
connection image, requirements, development phases, completed feature matrix,
and suggested next features.

In STM32CubeIDE use **File > Import > General > Existing Projects into
Workspace**, select this directory, and import `Dexter_STM32L552ZET6`. Both
Debug and Release managed-build configurations are included.
