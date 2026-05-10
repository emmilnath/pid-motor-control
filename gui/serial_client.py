"""Serial client for the PID controller binary protocol."""
from __future__ import annotations
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional
import serial


# Protocol constants
START_BYTE     = 0xAA
CMD_SET_GAINS  = 0x01
CMD_SET_SETPOINT = 0x02
CMD_GET_STATE  = 0x03
CMD_ENABLE     = 0x04
CMD_RESET      = 0x05
RESP_STATE     = 0x81
RESP_ACK       = 0x82
RESP_NACK      = 0x83


@dataclass
class ControllerState:
    timestamp_ms: int
    setpoint: float
    rpm: float
    error: float
    output: float
    kp: float
    ki: float
    kd: float


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    header = bytes([START_BYTE, cmd, len(payload)])
    crc_data = bytes([cmd, len(payload)]) + payload
    return header + payload + bytes([crc8(crc_data)])


class PIDSerialClient:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._on_state: Optional[Callable[[ControllerState], None]] = None

        # Parser state
        self._parse_state = 0  # 0=start, 1=cmd, 2=len, 3=payload, 4=crc
        self._frame_cmd = 0
        self._frame_len = 0
        self._frame_payload = bytearray()
        self._payload_idx = 0

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def on_state(self, callback: Callable[[ControllerState], None]):
        self._on_state = callback

    def set_gains(self, kp: float, ki: float, kd: float):
        payload = struct.pack("<fff", kp, ki, kd)
        self._send(CMD_SET_GAINS, payload)

    def set_setpoint(self, rpm: float):
        payload = struct.pack("<f", rpm)
        self._send(CMD_SET_SETPOINT, payload)

    def enable(self, on: bool):
        self._send(CMD_ENABLE, bytes([1 if on else 0]))

    def reset(self):
        self._send(CMD_RESET)

    def _send(self, cmd: int, payload: bytes = b""):
        if self.ser and self.ser.is_open:
            self.ser.write(build_frame(cmd, payload))

    def _read_loop(self):
        while self._running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.01)
                continue
            data = self.ser.read(64)
            for b in data:
                self._feed_byte(b)

    def _feed_byte(self, byte: int):
        if self._parse_state == 0:
            if byte == START_BYTE:
                self._parse_state = 1
                self._frame_payload = bytearray()
                self._payload_idx = 0
        elif self._parse_state == 1:
            self._frame_cmd = byte
            self._parse_state = 2
        elif self._parse_state == 2:
            self._frame_len = byte
            if byte > 32:
                self._parse_state = 0
            elif byte == 0:
                self._parse_state = 4
            else:
                self._parse_state = 3
        elif self._parse_state == 3:
            self._frame_payload.append(byte)
            self._payload_idx += 1
            if self._payload_idx >= self._frame_len:
                self._parse_state = 4
        elif self._parse_state == 4:
            self._parse_state = 0
            # Verify CRC
            crc_data = bytes([self._frame_cmd, self._frame_len]) + bytes(self._frame_payload)
            if crc8(crc_data) == byte:
                self._handle_frame(self._frame_cmd, bytes(self._frame_payload))

    def _handle_frame(self, cmd: int, payload: bytes):
        if cmd == RESP_STATE and len(payload) >= 32:
            ts, sp, rpm, err, out, kp, ki, kd = struct.unpack_from("<Ifffffff", payload)
            state = ControllerState(ts, sp, rpm, err, out, kp, ki, kd)
            if self._on_state:
                self._on_state(state)
