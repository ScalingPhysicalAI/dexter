# Dexter STM32L552 control tool

The GUI connects to either the USB CDC virtual COM port or the LPUART1 debug
adapter. Both interfaces accept the same command set.

```powershell
py -m pip install -r requirements.txt
py humanoid_control.py
```

Features include manual XYZ jogging, synchronized move entry, cycle-engine
macro control, queued script execution, settings, status polling, and an
emergency stop.

LPUART1 is fixed at 115200 baud on PG7 (TX) and PG8 (RX) and uses interrupt-mode
receive/transmit without DMA. USB CDC ignores the selected baud rate but 115200
is recommended for consistency. Both ports print the command guide at boot;
send `HELP` to print it again.

## STM32MP2/Linux CAN command client

`mp2_can_command.py` communicates with the L5 board through Linux SocketCAN and
uses only the Python standard library. The L5 firmware expects classic CAN at
500 kbit/s, command ID `0x600`, and response ID `0x601`.

Bring up the STM32MP2 CAN interface (change `can0` if required):

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Send one command:

```sh
python3 mp2_can_command.py -i can0 "G0 X1000 Y1000 Z1000"
python3 mp2_can_command.py -i can0 "ESTOP"
python3 mp2_can_command.py -i can0 "CLEAR ALARM"
```

Query or change the L5 runtime CAN IDs. The change command uses the current IDs
for its acknowledgement and then switches the tool to the new IDs:

```sh
python3 mp2_can_command.py -i can0 "CANID?"
python3 mp2_can_command.py -i can0 --set-ids 0x620 0x621
python3 mp2_can_command.py -i can0 --command-id 0x620 --response-id 0x621 "?"
```

Start an interactive console or run a command file:

```sh
python3 mp2_can_command.py -i can0
python3 mp2_can_command.py -i can0 --file movement.gcode
```

The MP2 and L5 each require a CAN transceiver. Connect CAN-H, CAN-L, and common
ground, and fit 120-ohm termination only at the two ends of the bus. Override
IDs with `--command-id` and `--response-id` if the firmware macros were changed.
