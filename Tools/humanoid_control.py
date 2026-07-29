#!/usr/bin/env python3
"""Desktop controller for Dexter STM32L552 three-axis firmware."""

import sys
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QSpinBox, QTabWidget,
    QVBoxLayout, QWidget,
)


class DexterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dexter STM32L552 Control")
        self.resize(980, 700)
        self.serial_port = None
        self.rx_buffer = ""
        self.script_queue = deque()
        self.script_running = False
        self._build_ui()

        self.io_timer = QTimer(self)
        self.io_timer.timeout.connect(self._poll_serial)
        self.io_timer.start(20)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(lambda: self.send("?", log=False))

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        connection = QHBoxLayout()
        self.port_box = QComboBox()
        self.port_box.setMinimumWidth(180)
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        estop = QPushButton("EMERGENCY STOP")
        estop.setStyleSheet("background:#b22;color:white;font-weight:bold")
        estop.clicked.connect(self.stop_script)
        connection.addWidget(QLabel("COM port:"))
        connection.addWidget(self.port_box)
        connection.addWidget(refresh)
        connection.addWidget(self.connect_button)
        connection.addStretch()
        connection.addWidget(estop)
        layout.addLayout(connection)
        self.refresh_ports()

        body = QHBoxLayout()
        tabs = QTabWidget()
        tabs.addTab(self._jog_tab(), "Jog")
        tabs.addTab(self._cycle_tab(), "Cycle engine")
        tabs.addTab(self._script_tab(), "Script")
        tabs.addTab(self._settings_tab(), "Settings")
        body.addWidget(tabs, 2)

        right = QVBoxLayout()
        self.state_label = QLabel("Disconnected")
        right.addWidget(self.state_label)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        right.addWidget(self.log, 1)
        command_row = QHBoxLayout()
        self.command = QLineEdit()
        self.command.setPlaceholderText("G0 X1000 Y1000 Z1000")
        self.command.returnPressed.connect(self.send_command_box)
        send_button = QPushButton("Send")
        send_button.clicked.connect(self.send_command_box)
        command_row.addWidget(self.command)
        command_row.addWidget(send_button)
        right.addLayout(command_row)
        body.addLayout(right, 3)
        layout.addLayout(body, 1)

    def _jog_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        form = QFormLayout()
        self.jog_distance = QDoubleSpinBox()
        self.jog_distance.setRange(0.1, 10000.0)
        self.jog_distance.setValue(10.0)
        self.jog_feed = QSpinBox()
        self.jog_feed.setRange(20, 10000)
        self.jog_feed.setValue(500)
        form.addRow("Distance:", self.jog_distance)
        form.addRow("Feed (steps/s):", self.jog_feed)
        layout.addLayout(form)

        grid = QGridLayout()
        moves = [
            ("Y+", 0, 1, "Y", 1), ("X-", 1, 0, "X", -1),
            ("X+", 1, 2, "X", 1), ("Y-", 2, 1, "Y", -1),
            ("Z+", 0, 3, "Z", 1), ("Z-", 2, 3, "Z", -1),
        ]
        for label, row, column, axis, sign in moves:
            button = QPushButton(label)
            button.setMinimumHeight(45)
            button.clicked.connect(
                lambda checked=False, a=axis, s=sign: self.jog(a, s)
            )
            grid.addWidget(button, row, column)
        home = QPushButton("Go to zero")
        home.clicked.connect(lambda: self.start_script(["G90", "G0 X0 Y0 Z0"]))
        grid.addWidget(home, 1, 1)
        layout.addLayout(grid)

        sync = QGroupBox("Synchronized target")
        sync_form = QFormLayout(sync)
        self.sync_x = QLineEdit("1000")
        self.sync_y = QLineEdit("1000")
        self.sync_z = QLineEdit("1000")
        sync_form.addRow("X", self.sync_x)
        sync_form.addRow("Y", self.sync_y)
        sync_form.addRow("Z", self.sync_z)
        run = QPushButton("Run G0 XYZ")
        run.clicked.connect(lambda: self.send(
            f"G0 X{self.sync_x.text()} Y{self.sync_y.text()} Z{self.sync_z.text()}"
        ))
        sync_form.addRow(run)
        layout.addWidget(sync)
        layout.addStretch()
        return tab

    def _cycle_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.macro_list = QListWidget()
        layout.addWidget(self.macro_list)
        row = QHBoxLayout()
        for label, callback in (
            ("Refresh", lambda: self.send("LIST")),
            ("Run", self.run_selected_macro),
            ("View", self.view_selected_macro),
            ("Stop", lambda: self.send("STOP")),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        return tab

    def _script_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(QLabel("One command per line. Lines are sent after each firmware acknowledgement."))
        self.script = QPlainTextEdit()
        self.script.setPlainText("G90\nG0 X1000 Y1000 Z1000\nG4 P500\nG0 X0 Y0 Z0")
        layout.addWidget(self.script)
        row = QHBoxLayout()
        run = QPushButton("Run script")
        run.clicked.connect(self.run_script)
        stop = QPushButton("Stop script and motors")
        stop.clicked.connect(self.stop_script)
        row.addWidget(run)
        row.addWidget(stop)
        layout.addLayout(row)
        return tab

    def _settings_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        settings = [
            ("$0", "X max speed", "1000"), ("$1", "Y max speed", "1000"),
            ("$6", "Z max speed", "500"), ("$2", "X acceleration", "5000"),
            ("$3", "Y acceleration", "5000"), ("$7", "Z acceleration", "2000"),
            ("$4", "Default feed", "500"), ("$5", "Rapid speed", "1000"),
            ("$10", "X pulses/rev", "3200"), ("$11", "X micrometres/rev", "8000"),
            ("$12", "Y pulses/rev", "3200"), ("$13", "Y micrometres/rev", "8000"),
            ("$15", "Z pulses/rev", "3200"), ("$16", "Z micrometres/rev", "8000"),
            ("$20", "Invert X direction", "0"), ("$21", "Invert Y direction", "0"),
            ("$22", "Invert Z direction", "0"), ("$23", "Limit active-high", "1"),
            ("$14", "Units: 0=steps, 1=mm", "0"),
        ]
        form = QGridLayout()
        for row, (key, label, default) in enumerate(settings):
            editor = QLineEdit(default)
            button = QPushButton("Set")
            button.clicked.connect(
                lambda checked=False, k=key, e=editor: self.send(f"{k}={e.text()}")
            )
            form.addWidget(QLabel(key), row, 0)
            form.addWidget(QLabel(label), row, 1)
            form.addWidget(editor, row, 2)
            form.addWidget(button, row, 3)
        layout.addLayout(form)
        query = QPushButton("Read all settings")
        query.clicked.connect(lambda: self.send("$"))
        layout.addWidget(query)

        can_ids = QGroupBox("CAN command IDs (runtime)")
        can_form = QGridLayout(can_ids)
        self.can_rx_id = QLineEdit("0x600")
        self.can_tx_id = QLineEdit("0x601")
        set_can_ids = QPushButton("Set CAN IDs")
        set_can_ids.clicked.connect(lambda: self.send(
            f"CANID {self.can_rx_id.text()} {self.can_tx_id.text()}"
        ))
        read_can_ids = QPushButton("Read CAN IDs")
        read_can_ids.clicked.connect(lambda: self.send("CANID?"))
        can_form.addWidget(QLabel("L5 receive ID"), 0, 0)
        can_form.addWidget(self.can_rx_id, 0, 1)
        can_form.addWidget(QLabel("L5 response ID"), 1, 0)
        can_form.addWidget(self.can_tx_id, 1, 1)
        can_form.addWidget(set_can_ids, 2, 0)
        can_form.addWidget(read_can_ids, 2, 1)
        layout.addWidget(can_ids)
        layout.addStretch()
        return tab

    def refresh_ports(self):
        selected = self.port_box.currentText()
        ports = [item.device for item in serial.tools.list_ports.comports()]
        self.port_box.clear()
        self.port_box.addItems(ports)
        if selected in ports:
            self.port_box.setCurrentText(selected)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.status_timer.stop()
            self.connect_button.setText("Connect")
            self.state_label.setText("Disconnected")
            return
        port = self.port_box.currentText()
        if not port:
            QMessageBox.warning(self, "No port", "Select a COM port first.")
            return
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0)
        except serial.SerialException as error:
            QMessageBox.critical(self, "Connection failed", str(error))
            return
        self.connect_button.setText("Disconnect")
        self.state_label.setText(f"Connected: {port}")
        self.status_timer.start(500)
        QTimer.singleShot(300, lambda: self.send("LIST"))

    def send(self, text, log=True):
        if not self.serial_port or not self.serial_port.is_open:
            if log:
                self.log.appendPlainText("[not connected]")
            return False
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        for line in lines:
            self.serial_port.write((line + "\n").encode("ascii", errors="ignore"))
            if log:
                self.log.appendPlainText(f"> {line}")
        return True

    def send_command_box(self):
        command = self.command.text().strip()
        if command:
            self.send(command)
            self.command.clear()

    def jog(self, axis, sign):
        distance = self.jog_distance.value() * sign
        self.start_script(["G91", f"G1 {axis}{distance:g} F{self.jog_feed.value()}", "G90"])

    def run_selected_macro(self):
        item = self.macro_list.currentItem()
        if item:
            self.send(f"RUN {item.text()}")

    def view_selected_macro(self):
        item = self.macro_list.currentItem()
        if item:
            self.send(f"MACRO {item.text()}")

    def run_script(self):
        lines = []
        for raw in self.script.toPlainText().splitlines():
            line = raw.strip()
            if line and not line.startswith(";"):
                lines.append(line)
        self.start_script(lines)

    def start_script(self, lines):
        if self.script_running:
            QMessageBox.information(self, "Script busy", "Stop the current script first.")
            return
        self.script_queue = deque(lines)
        self.script_running = bool(self.script_queue)
        self._send_next_script_line()

    def _send_next_script_line(self):
        if not self.script_running:
            return
        if not self.script_queue:
            self.script_running = False
            self.log.appendPlainText("[script complete]")
            return
        self.send(self.script_queue.popleft())

    def stop_script(self):
        self.script_queue.clear()
        self.script_running = False
        self.send("M112")

    def _poll_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                self.rx_buffer += self.serial_port.read(waiting).decode("utf-8", errors="replace")
        except serial.SerialException as error:
            self.log.appendPlainText(f"[serial error] {error}")
            self.toggle_connection()
            return
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip()
            if line:
                self._handle_line(line)

    def _handle_line(self, line):
        self.log.appendPlainText(line)
        if line == "ok" and self.script_running:
            self._send_next_script_line()
        if line == "[CE:macros]":
            self.macro_list.clear()
        elif line.endswith("lines)") and "(" in line:
            name = line.split("(", 1)[0].strip()
            if name and not self.macro_list.findItems(name, Qt.MatchExactly):
                self.macro_list.addItem(name)
        elif line.startswith("<"):
            self.state_label.setText(line)
        elif line.startswith("CANID RX=") and " TX=" in line:
            receive_id, transmit_id = line[9:].split(" TX=", 1)
            self.can_rx_id.setText(receive_id.strip())
            self.can_tx_id.setText(transmit_id.strip())

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = DexterWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
