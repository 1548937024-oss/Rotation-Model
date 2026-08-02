"""In-process motor module simulator used for UI and protocol testing."""

from __future__ import annotations

from collections import deque
import struct
import threading
import time

from motor_protocol import (
    HEADER,
    MotorControl,
    build_reply_tail,
    parse_full_frame,
)


class SimulatedTransport:
    """A small serial-like transport implementing one module at a time."""

    def __init__(self, node_id: int = 1, byteorder: str = "little") -> None:
        self.node_id = node_id
        self.byteorder = byteorder
        self.baudrate = 921600
        self.is_open = True
        self._rx: deque[int] = deque()
        self._lock = threading.Lock()
        self._last_update = time.monotonic()
        self._pending_service = bytes(8)
        self.enabled = False
        self.mode = 0
        self.target_position = 0
        self.position = 0.0
        self.speed = 0
        self.iq = 0
        self.voltage_mv = 24000
        self.temperature_c = 31
        self.error_code = 0
        self.uid = 0x31A2B4C5
        self.parameters: dict[tuple[int, int], int] = {
            (0x1018, 4): self.uid,
            (0x3001, 1): self.baudrate,
            (0x6073, 1): 1200,
            (0x6073, 3): 500,
        }

    def _advance(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_update, 0.2)
        self._last_update = now
        if not self.enabled:
            self.speed = 0
            self.iq = 0
            return
        if self.mode == 0:
            delta = self.target_position - self.position
            max_step = max(abs(self.speed), 8) * 35.0 * dt
            step = max(-max_step, min(max_step, delta))
            self.position += step
            if abs(delta) < 1:
                self.speed = 0
        else:
            self.position = (self.position + self.speed * 16.0 * dt) % 65536

    def _motion_data(self) -> bytes:
        self._advance()
        flags = (0x80 if self.enabled else 0) | (0x40 if self.error_code else 0)
        flags |= self.error_code if self.error_code else self.mode
        return bytes((flags,)) + struct.pack(
            "<Hbb", int(self.position) & 0xFFFF, int(self.speed), int(self.iq)
        )

    def _device_data(self) -> bytes:
        self._advance()
        temperature = self.temperature_c + (2 if self.enabled else 0)
        return struct.pack("<Hb", self.voltage_mv, temperature)

    def _handle_service(self, data: bytes) -> None:
        sid = data[0]
        if sid in (0x23, 0x40):
            did = data[1] | (data[2] << 8)
            sub_id = data[3]
            key = (did, sub_id)
            if sid == 0x40:
                if key not in self.parameters:
                    self._pending_service = bytes((0x80, data[1], data[2], sub_id)) + (
                        0x06020000
                    ).to_bytes(4, "little")
                else:
                    self._pending_service = bytes((0x43, data[1], data[2], sub_id)) + (
                        self.parameters[key].to_bytes(4, "little")
                    )
                return
            if key == (0x1018, 4):
                code = 0x06010002
                self._pending_service = bytes((0x80, data[1], data[2], sub_id)) + code.to_bytes(
                    4, "little"
                )
                return
            value = int.from_bytes(data[4:8], "little")
            if key not in self.parameters and key != (0x6040, 0):
                code = 0x06020000
                self._pending_service = bytes((0x80, data[1], data[2], sub_id)) + code.to_bytes(
                    4, "little"
                )
                return
            if key == (0x6040, 0) and value == 0x80:
                self.error_code = 0
            else:
                self.parameters[key] = value
                if key == (0x3001, 1):
                    self.baudrate = value
            self._pending_service = bytes((0x60, data[1], data[2], sub_id)) + data[4:8]
            return
        if sid == 0xF1:
            sub_function = data[1]
            return_code = 0
            if sub_function == 0x10:
                if data[2] not in (0, 1):
                    return_code = 0xE2
                else:
                    self.enabled = bool(data[2])
            elif sub_function == 0x11:
                if self.enabled:
                    return_code = 0xE5
                elif data[2] not in (0, 1):
                    return_code = 0xE2
                else:
                    self.mode = data[2]
            elif sub_function == 0xA2:
                self.position = 0
                self.target_position = 0
            else:
                return_code = 0xE2
            self._pending_service = bytes(
                (0xF1, sub_function, data[2], return_code, 0, 0, 0, 0)
            )
            return
        if sid == 0xF2:
            new_id = data[3]
            uid = int.from_bytes(data[4:8], "little")
            return_code = 0
            if self.enabled:
                return_code = 0xE0
            elif uid != self.uid:
                return_code = 0xE2
            elif not 1 <= new_id <= 31:
                return_code = 0xE4
            if return_code == 0:
                self.node_id = new_id
            self._pending_service = bytes((0xF2, return_code, 0, new_id, 0, 0, 0, 0))

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("模拟串口已关闭")
        with self._lock:
            if len(data) == 3 and data[0] == HEADER:
                mid = int.from_bytes(data[1:3], self.byteorder) & 0x7FF
                if mid == 0x180 + self.node_id:
                    reply = self._motion_data()
                elif mid == 0x280 + self.node_id:
                    reply = self._device_data()
                elif 0x581 <= mid <= 0x59F:
                    reply = self._pending_service
                else:
                    return len(data)
                self._rx.extend(build_reply_tail(mid, reply, self.byteorder))
                return len(data)

            mid, payload = parse_full_frame(data, self.byteorder)
            if mid == 0x200 + self.node_id and len(payload) == 5:
                control = MotorControl.unpack(payload)
                self.enabled = control.enable
                self.mode = control.mode
                self.target_position = control.target_position
                self.speed = control.target_speed
                self.iq = control.target_iq
            elif mid == 0x200 and len(payload) >= 5 and len(payload) % 5 == 0:
                mcp_index = self.node_id - 1
                if (mcp_index + 1) * 5 <= len(payload):
                    control = MotorControl.unpack(
                        payload[mcp_index * 5 : (mcp_index + 1) * 5]
                    )
                    self.enabled = control.enable
                    self.mode = control.mode
                    self.target_position = control.target_position
                    self.speed = control.target_speed
                    self.iq = control.target_iq
            elif mid in (0x600, 0x600 + self.node_id) and len(payload) == 8:
                self._handle_service(payload)
            return len(data)

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + 0.12
        result = bytearray()
        while len(result) < size and time.monotonic() < deadline:
            with self._lock:
                while self._rx and len(result) < size:
                    result.append(self._rx.popleft())
            if len(result) < size:
                time.sleep(0.001)
        return bytes(result)

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def close(self) -> None:
        self.is_open = False
