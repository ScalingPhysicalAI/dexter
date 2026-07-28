# Dexter STM32L552ZET6 firmware

Self-contained STM32CubeIDE project foundation for STM32L552ZET6 using
STM32CubeL5 V1.6.0.

## Interfaces

- X STEP/DIR: PA0/PA4
- Y STEP/DIR: PA1/PA5
- Z STEP/DIR: PB10/PB11
- Z minimum/maximum limits: PC0/PC1, normally closed, pull-up enabled
- FDCAN1 RX/TX: PB8/PB9, classic CAN
- USB CDC DM/DP: PA11/PA12
- TIM2: 10 kHz motion interrupt from the 110 MHz timer clock

The CAN pins require an external CAN transceiver. Install 120 ohm termination
only at the two physical ends of the CAN bus.

`can_interface.c` accepts standard 11-bit CAN frames and provides an eight-frame
RX FIFO and TX FIFO. `limit_switches.c` treats HIGH as tripped/open wire.

Open `dexter_l552.ioc` or import this folder as an existing STM32CubeIDE
project. Regenerating from the IOC may update generated HAL and USB files; keep
the user application modules in the marked USER CODE sections.

In STM32CubeIDE use **File > Import > General > Existing Projects into
Workspace**, select this directory, and import `Dexter_STM32L552ZET6`. Both
Debug and Release managed-build configurations are included.
