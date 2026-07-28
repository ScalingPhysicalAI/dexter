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
