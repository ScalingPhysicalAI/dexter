#!/usr/bin/env python3
"""Send Dexter ASCII commands from Linux/STM32MP2 over SocketCAN."""

import argparse
import select
import socket
import struct
import sys
import time


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_FILTER = struct.Struct("=II")
CAN_SFF_MASK = 0x7FF
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FILTER = getattr(socket, "CAN_RAW_FILTER", 1)
PF_CAN = getattr(socket, "PF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)


def parse_can_id(value):
    parsed = int(value, 0)
    if not 0 <= parsed <= CAN_SFF_MASK:
        raise argparse.ArgumentTypeError("standard CAN ID must be 0x000..0x7FF")
    return parsed


class DexterCanClient:
    def __init__(self, interface, command_id, response_id, timeout, quiet_time):
        self.interface = interface
        self.command_id = command_id
        self.response_id = response_id
        self.timeout = timeout
        self.quiet_time = quiet_time
        self.sock = None

    def open(self):
        try:
            self.sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
            response_filter = CAN_FILTER.pack(self.response_id, CAN_SFF_MASK)
            self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, response_filter)
            self.sock.bind((self.interface,))
        except (AttributeError, OSError) as error:
            self.close()
            raise RuntimeError(
                "cannot open SocketCAN interface {!r}: {}".format(self.interface, error)
            ) from error

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _send_frame(self, payload):
        frame = CAN_FRAME.pack(
            self.command_id,
            len(payload),
            payload.ljust(8, b"\x00"),
        )
        sent = self.sock.send(frame)
        if sent != CAN_FRAME.size:
            raise RuntimeError("short SocketCAN frame write")

    def send_command(self, command):
        command = command.rstrip("\r\n")
        try:
            payload = (command + "\n").encode("ascii")
        except UnicodeEncodeError as error:
            raise ValueError("commands must contain ASCII characters only") from error
        if len(payload) > 96:
            raise ValueError("command exceeds firmware limit of 95 characters")
        for offset in range(0, len(payload), 8):
            self._send_frame(payload[offset:offset + 8])

    def read_response(self):
        chunks = []
        first_deadline = time.monotonic() + self.timeout
        last_frame_time = None

        while True:
            now = time.monotonic()
            if last_frame_time is None:
                remaining = first_deadline - now
            else:
                remaining = self.quiet_time - (now - last_frame_time)
            if remaining <= 0:
                break

            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                break
            packet = self.sock.recv(CAN_FRAME.size)
            if len(packet) != CAN_FRAME.size:
                continue
            can_id, dlc, data = CAN_FRAME.unpack(packet)
            if (can_id & CAN_SFF_MASK) != self.response_id:
                continue
            chunks.append(data[:min(dlc, 8)])
            last_frame_time = time.monotonic()

        return b"".join(chunks).decode("utf-8", errors="replace")

    def transact(self, command):
        self.send_command(command)
        return self.read_response()

    def change_ids(self, new_command_id, new_response_id):
        response = self.transact(
            "CANID 0x{:03X} 0x{:03X}".format(new_command_id, new_response_id)
        )
        self.command_id = new_command_id
        self.response_id = new_response_id
        response_filter = CAN_FILTER.pack(self.response_id, CAN_SFF_MASK)
        self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, response_filter)
        time.sleep(0.02)
        return response


def print_response(response):
    if response:
        print(response, end="" if response.endswith("\n") else "\n")
    else:
        print("[no CAN response]", file=sys.stderr)


def run_file(client, path):
    with open(path, "r", encoding="utf-8") as command_file:
        for line_number, raw_line in enumerate(command_file, 1):
            command = raw_line.strip()
            if not command or command.startswith(";") or command.startswith("#"):
                continue
            print("> {}".format(command))
            try:
                print_response(client.transact(command))
            except (OSError, RuntimeError, ValueError) as error:
                raise RuntimeError("{}:{}: {}".format(path, line_number, error)) from error


def interactive(client):
    print("Dexter CAN console. Type HELP for firmware help; Ctrl-D or EXIT to quit.")
    while True:
        try:
            command = input("can> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not command:
            continue
        if command.upper() in ("EXIT", "QUIT"):
            return
        try:
            print_response(client.transact(command))
        except (OSError, RuntimeError, ValueError) as error:
            print("error: {}".format(error), file=sys.stderr)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Communicate with Dexter STM32L552 over Linux SocketCAN."
    )
    parser.add_argument("command", nargs="*", help="one firmware command")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface")
    parser.add_argument("--command-id", type=parse_can_id, default=0x600,
                        help="board receive ID (default: 0x600)")
    parser.add_argument("--response-id", type=parse_can_id, default=0x601,
                        help="board response ID (default: 0x601)")
    parser.add_argument("--set-ids", nargs=2, type=parse_can_id,
                        metavar=("NEW_RX", "NEW_TX"),
                        help="change L5 command/response IDs using the current IDs")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="seconds to wait for first response frame")
    parser.add_argument("--quiet-time", type=float, default=0.15,
                        help="response ends after this idle interval")
    parser.add_argument("-f", "--file", help="send commands from a text file")
    return parser


def main():
    args = build_parser().parse_args()
    if args.timeout <= 0 or args.quiet_time <= 0:
        print("error: timeout values must be positive", file=sys.stderr)
        return 2
    if args.set_ids and args.set_ids[0] == args.set_ids[1]:
        print("error: CAN receive and transmit IDs must differ", file=sys.stderr)
        return 2
    try:
        with DexterCanClient(
            args.interface,
            args.command_id,
            args.response_id,
            args.timeout,
            args.quiet_time,
        ) as client:
            if args.set_ids:
                print_response(client.change_ids(args.set_ids[0], args.set_ids[1]))
            if args.file:
                run_file(client, args.file)
            elif args.command:
                print_response(client.transact(" ".join(args.command)))
            elif not args.set_ids:
                interactive(client)
    except (OSError, RuntimeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
