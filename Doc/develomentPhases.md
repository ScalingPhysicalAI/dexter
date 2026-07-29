# Development phases

| Phase | Scope | Status | Exit evidence |
|---|---|---|---|
| 1 | STM32L552ZET6 CubeIDE project, IOC, clocks, GPIO, linker/startup | Done | Clean ELF/HEX/BIN build |
| 2 | Three-axis STEP/DIR driver and coordinated DDA scheduler | Done | Synchronized XYZ firmware path |
| 3 | G-code parser, settings, command queue, dwell, absolute/relative modes | Done | Command and build validation |
| 4 | Cycle engine, built-in macros, script execution | Done | Non-blocking macro progression |
| 5 | USB CDC and interrupt-driven LPUART1 command interfaces | Done | Shared parser and source-routed replies |
| 6 | FDCAN1 and STM32MP2 SocketCAN command transport | Done | Multi-frame ASCII fragmentation test |
| 7 | Z limits, direction control, E-stop, alarm latch/clear | Done | Compile and safety-path inspection |
| 8 | Runtime CAN-ID change, tools, connection image, project documentation | Done | `CANID` command and documentation package |
| 9 | Hardware-in-loop regression rig and automated interface tests | Suggested | Repeatable USB/UART/CAN test report |
| 10 | Persistent configuration with versioning and CRC | Suggested | Power-cycle retention and corruption test |
| 11 | Production safety hardware, watchdog, driver-fault feedback | Suggested | Safety review and fault-injection report |
| 12 | Secure update/recovery and release manufacturing flow | Suggested | Signed update and rollback demonstration |

## Recommended validation sequence

1. Build the Debug target and flash the generated HEX.
2. Verify startup help and `?` over LPUART1, then USB CDC.
3. Test each axis separately at low speed with limits disabled.
4. Verify direction settings and synchronized XYZ motion.
5. Verify raw limit readings, polarity, enable/disable, and Z stop behavior.
6. Press E-stop during the longest synchronized move and confirm every STEP
   output stops before attempting alarm clear.
7. Verify CAN defaults, long multi-frame commands, runtime ID switching, and
   reset-to-default behavior from the STM32MP2 client.
8. Run cycle and command-file tests, including E-stop interruption.
9. Repeat with expected bus load, disconnected sensors, and power cycling.
