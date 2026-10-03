"""MicroDriver V2.10 协议级模拟器（无需硬件即可验证界面与流程）。

模拟口径按 `docs/外置驱动器通讯协议_V1.4_20261003.md`：

* 默认 Node ID 1、460800 8N1、MID 小端、应答 CRC 用 Response MID。
* 首次使能先做约 1200 ms 对齐，对齐期间响应状态查询与服务请求。
* 0x6040 读取返回 Abort；设置零位 0xA2、设置 Node ID 0xF2 返回 Abort。
* 只读 Max Iq=1400 mA、Max Speed=18000 rpm；写入只读 DID 返回 Abort。
"""

from __future__ import annotations

from collections import deque
import struct
import threading
import time

from motor_protocol import (
    CONTROL_TIMEOUT_DEFAULT_MS,
    CONTROL_TIMEOUT_MAX_MS,
    CONTROL_TIMEOUT_MIN_MS,
    DEFAULT_BAUDRATE,
    HEADER,
    MAX_IQ_MA,
    MAX_SPEED_RPM,
    ALIGNMENT_TIME_S,
    MotorControl,
    POSITION_SOFT_MAX_COUNTS,
    POSITION_SOFT_MIN_COUNTS,
    build_reply_tail,
    parse_full_frame,
)

ABORT_READ_NOT_ALLOWED = 0x06010001
ABORT_WRITE_NOT_ALLOWED = 0x06010002
ABORT_DID_NOT_EXIST = 0x06020000
ABORT_VALUE_OUT_OF_RANGE = 0x06090030

# 与 board_conf.h 的 BOARD_FW_VERSION_U32 / BOARD_PROTO_VERSION_U32 一致
BOARD_FW_VERSION = 0x00021003
BOARD_PROTO_VERSION = 0x00010400


class SimulatedTransport:
    """单节点串口模拟器，接口与 pyserial.Serial 兼容。"""

    def __init__(self, node_id: int = 1, baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.node_id = node_id
        self.baudrate = int(baudrate)
        self.is_open = True
        self._rx: deque[int] = deque()
        self._lock = threading.Lock()
        self._last_update = time.monotonic()

        self.enabled = False
        self.mode = 0
        self.target_position = 0
        self.target_speed = 0
        self.target_iq = 20
        self.position = 0.0
        self.speed = 0
        self.iq = 0
        self.voltage_mv = 24000
        self.temperature_c = 31
        self.error_code = 0
        self.uid = 0x31A2B4C5
        self.phase_currents = (0, 0, 0)
        self.encoder_total = 0
        self.encoder_z = 0

        self._zero_established = False
        self._align_until: float | None = None
        self._apply_after: float | None = None
        self._pending_service = bytes(8)
        self._last_control: MotorControl | None = None
        # 安全保护 / 停车 / 通讯看门狗状态
        self._safety_latched = False
        self._comm_lost = False
        self._stop_active = False
        self._stop_kind = 0
        self._stop_deadline = 0.0
        self._last_cmd_time = time.monotonic()
        self._start_time = time.monotonic()
        self._pos_inband = True
        self._snapshot: dict[str, int] = {
            "code": 0, "time_ms": 0, "vbus_mv": 0, "rpm": 0, "iq_ma": 0, "pos": 0, "mode": 0,
        }
        # 0x2020 安全配置
        self._safe_cfg: dict[str, int] = {
            "pos_min": POSITION_SOFT_MIN_COUNTS,
            "pos_max": POSITION_SOFT_MAX_COUNTS,
            "ctrl_timeout_ms": CONTROL_TIMEOUT_DEFAULT_MS,
        }

        self._writable: dict[tuple[int, int], int] = {
            (0x6073, 0x01): 1500,
            (0x6073, 0x03): 1000,
            (0x202D, 0x01): 90,
            (0x202D, 0x02): 500,
            (0x202D, 0x03): 520,
            (0x202D, 0x04): 500,
            (0x2016, 0x02): 90,
            (0x2016, 0x03): 1000,
            (0x2016, 0x04): 75,
        }
        self._ranges: dict[tuple[int, int], tuple[int, int]] = {
            (0x6073, 0x01): (0, 3000),
            (0x6073, 0x03): (0, 65535),
            (0x202D, 0x01): (40, 110),
            (0x202D, 0x02): (0, 10000),
            (0x202D, 0x03): (80, 150),
            (0x202D, 0x04): (0, 10000),
            (0x2016, 0x02): (55, 110),
            (0x2016, 0x03): (0, 60000),
            (0x2016, 0x04): (50, 110),
        }
        self._eeprom: dict[tuple[int, int], int] = {}

    # ------------------------------------------------------------- 调试注入接口

    def inject_fault(self, code: int = 3) -> None:
        self.error_code = int(code)
        self.enabled = False
        self._stop_active = False
        self.speed = 0
        self.iq = 0
        self._record_snapshot(code)

    # ------------------------------------------------------------- 内部状态推进

    def _mark_alive(self) -> None:
        """记录一次合法控制帧（Control1 或 0xF1），用于控制帧超时保护。"""

        self._last_cmd_time = time.monotonic()

    def _record_snapshot(self, code: int) -> None:
        self._snapshot = {
            "code": int(code),
            "time_ms": int((time.monotonic() - self._start_time) * 1000),
            "vbus_mv": self.voltage_mv,
            "rpm": int(self.speed * MAX_SPEED_RPM / 100),
            "iq_ma": int(self.iq * MAX_IQ_MA / 100),
            "pos": int(self.encoder_total),
            "mode": int(self.mode),
        }

    def _begin_stop(self, kind: int) -> None:
        """kind: 1 = 正常停车（先减速），2 = 快速/急停（立即断开）。"""

        self._stop_kind = int(kind)
        if kind == 1 and self.enabled and self._align_until is None:
            if self._stop_active:
                return
            self._stop_active = True
            self._stop_deadline = time.monotonic() + 1.5
            self.target_speed = 0
            self.target_position = int(self.position)
            return
        self.enabled = False
        self._stop_active = False
        self._align_until = None
        self._apply_after = None
        self.speed = 0
        self.iq = 0

    def _watchdog_tick(self, now: float) -> None:
        """控制帧超时：使能后超时未收到合法控制帧，则受控停车并下使能。"""

        if not self.enabled or self._stop_active or self._align_until is not None:
            return
        timeout = int(self._safe_cfg["ctrl_timeout_ms"])
        if timeout and (now - self._last_cmd_time) * 1000.0 >= timeout:
            self._comm_lost = True
            self._begin_stop(1)

    def _advance(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_update, 0.2)
        self._last_update = now

        if self._align_until is not None:
            if now >= self._align_until:
                self._align_until = None
                self._zero_established = True
                self._apply_after = now + 0.02
                self.position = 0.0
                self._mark_alive()
            else:
                self.speed = 0
                self.iq = min(18, int(250 * 100 / MAX_IQ_MA))  # 对齐电流约 250 mA
                return

        self._watchdog_tick(now)

        if self._stop_active:
            if (not self.enabled) or abs(self.speed) <= 1 or now >= self._stop_deadline:
                self.enabled = False
                self._stop_active = False
                self.speed = 0
                self.iq = 0
                return
            self.target_speed = 0
            self.speed = int(self.speed * 0.5)
            self.iq = 0
            return

        if self._apply_after is not None:
            if now < self._apply_after:
                self.speed = 0
                self.iq = 0
                return
            self._apply_after = None

        if self.error_code:
            self.speed = 0
            self.iq = 0
            return
        if not self.enabled:
            self.speed = 0
            self.iq = 0
            return

        if self.mode == 0:
            delta = self.target_position - self.position
            max_step = max(abs(self.speed), 8) * 35.0 * dt
            step = max(-max_step, min(max_step, delta))
            self.position += step
            self.speed = int(max(-100, min(100, step / max(dt, 1e-6) / 35.0)))
            if abs(delta) < 1:
                self.speed = 0
                self._pos_inband = True
            else:
                self._pos_inband = False
        else:
            self.speed = max(-100, min(100, self.target_speed))
            self.position = (self.position + self.speed * 16.0 * dt) % 65536

        limit_percent = abs(self.target_iq) if self.target_iq else 100
        self.iq = min(100, limit_percent)
        self.encoder_total = int(self.position) * 65
        self.encoder_z = abs(self.encoder_total) // 4096
        peak = int(MAX_IQ_MA * self.iq / 100)
        self.phase_currents = (peak, -peak // 2, -peak // 2)

    def _mcu_state(self) -> int:
        if self._stop_active:
            return 6
        if self.error_code:
            return 4
        if self._align_until is not None:
            return 2
        if self.enabled:
            return 3
        return 0

    def _query_reply(self, mid: int) -> bytes | None:
        if mid == 0x180 + self.node_id:
            return self._motion_data()
        if mid == 0x280 + self.node_id:
            return self._device_data()
        # 0x580+NID 只在收到服务请求后主动下发，不作为可查询 MID（与固件一致）。
        return None

    def _motion_data(self) -> bytes:
        self._advance()
        if self.error_code:
            flags = 0x40 | (self.error_code & 0x3F)
        else:
            flags = (0x80 if self.enabled else 0x00) | (self.mode & 0x3F)
        position = int(self.position)
        position = ((position + 32768) % 65536) - 32768
        return bytes((flags,)) + struct.pack("<hbb", position, int(self.speed), int(self.iq))

    def _device_data(self) -> bytes:
        self._advance()
        temperature = self.temperature_c + (2 if self.enabled else 0)
        return struct.pack("<Hb", self.voltage_mv, temperature)

    # ------------------------------------------------------------- DID 服务

    def _read_only_value(self, key: tuple[int, int]) -> int | None:
        did, sub = key
        if key == (0x1018, 0x04):
            return self.uid
        if key == (0x3001, 0x01):
            return self.baudrate
        if key == (0x6073, 0x00):
            return MAX_IQ_MA
        if key == (0x607F, 0x00):
            return MAX_SPEED_RPM
        if did == 0x2010:
            if sub == 0x04:
                return max(abs(value) for value in self.phase_currents)
            if 1 <= sub <= 3:
                return self.phase_currents[sub - 1] & 0xFFFF
        if key == (0x2015, 0x01):
            return self.encoder_total & 0xFFFFFFFF
        if key == (0x2015, 0x02):
            return self.encoder_z & 0xFFFFFFFF
        if did == 0x2020:
            if sub == 0x01:
                return self._safe_cfg["pos_min"] & 0xFFFFFFFF
            if sub == 0x02:
                return self._safe_cfg["pos_max"] & 0xFFFFFFFF
            if sub == 0x03:
                return self._safe_cfg["ctrl_timeout_ms"]
        if did == 0x2021:
            snapshot = self._snapshot
            if sub == 0x01:
                return self._mcu_state()
            if sub == 0x02:
                return self._safety_flags()
            if sub == 0x03:
                return BOARD_FW_VERSION
            if sub == 0x04:
                return self.error_code & 0xFFFF
            if sub == 0x05:
                return self._stop_kind & 0xFFFF
            if sub == 0x06:
                return BOARD_PROTO_VERSION
            if sub == 0x07:
                return snapshot["code"] & 0xFFFF
            if sub == 0x08:
                return snapshot["time_ms"] & 0xFFFFFFFF
            if sub == 0x09:
                return snapshot["vbus_mv"] & 0xFFFFFFFF
            if sub == 0x0A:
                return snapshot["rpm"] & 0xFFFFFFFF
            if sub == 0x0B:
                return snapshot["iq_ma"] & 0xFFFFFFFF
            if sub == 0x0C:
                return snapshot["pos"] & 0xFFFFFFFF
            if sub == 0x0D:
                return snapshot["mode"] & 0xFFFF
        return None

    def _safety_flags(self) -> int:
        flags = 0
        if self._zero_established:
            flags |= 0x0001
        if self._pos_inband:
            flags |= 0x0002
        if self.error_code or self._safety_latched:
            flags |= 0x0004
        if self.enabled:
            flags |= 0x0008
        if self._comm_lost:
            flags |= 0x0010
        if self._stop_active:
            flags |= 0x0020
        if self._safety_latched:
            flags |= 0x0040
        return flags

    def _reply_service(self, data: bytes) -> None:
        """按固件行为：服务应答在收完请求后立即主动下发，主机无需再查询。"""

        self._pending_service = bytes(data)
        self._rx.extend(build_reply_tail(0x580 + self.node_id, bytes(data)))

    def _abort(self, did: int, sub: int, code: int) -> None:
        self._reply_service(
            bytes((0x80, did & 0xFF, (did >> 8) & 0xFF, sub)) + code.to_bytes(4, "little")
        )

    def _handle_did(self, data: bytes) -> None:
        sid = data[0]
        did = data[1] | (data[2] << 8)
        sub = data[3]
        if sid == 0x40:
            if (did, sub) == (0x6040, 0x00):
                self._abort(did, sub, ABORT_READ_NOT_ALLOWED)
                return
            value = self._read_only_value((did, sub))
            if value is None and (did, sub) in self._writable:
                value = self._writable[(did, sub)]
            if value is None:
                self._abort(did, sub, ABORT_DID_NOT_EXIST)
                return
            self._reply_service(
                bytes((0x43, did & 0xFF, (did >> 8) & 0xFF, sub))
                + (value & 0xFFFFFFFF).to_bytes(4, "little")
            )
            return

        if sid == 0x23:
            value = int.from_bytes(data[4:8], "little")
            if (did, sub) == (0x1010, 0x00):
                self._advance()
                if self.enabled:
                    self._abort(did, sub, ABORT_WRITE_NOT_ALLOWED)
                    return
                self._eeprom = dict(self._writable)
                self._reply_service(
                    bytes((0x60, did & 0xFF, (did >> 8) & 0xFF, sub, 0, 0, 0, 0))
                )
                return
            if (did, sub) == (0x1011, 0x00):
                if self._eeprom:
                    self._writable.update(self._eeprom)
                self._reply_service(
                    bytes((0x60, did & 0xFF, (did >> 8) & 0xFF, sub, 0, 0, 0, 0))
                )
                return
            if did == 0x2020:
                # 安全配置：仅停机可写，且有交叉约束
                if self.enabled or self._align_until is not None or self._stop_active:
                    self._abort(did, sub, ABORT_WRITE_NOT_ALLOWED)
                    return
                signed_value = value - 0x100000000 if value & 0x80000000 else value
                if sub == 0x01:
                    if signed_value < POSITION_SOFT_MIN_COUNTS or signed_value >= self._safe_cfg["pos_max"]:
                        self._abort(did, sub, ABORT_VALUE_OUT_OF_RANGE)
                        return
                    self._safe_cfg["pos_min"] = signed_value
                elif sub == 0x02:
                    if signed_value > POSITION_SOFT_MAX_COUNTS or signed_value <= self._safe_cfg["pos_min"]:
                        self._abort(did, sub, ABORT_VALUE_OUT_OF_RANGE)
                        return
                    self._safe_cfg["pos_max"] = signed_value
                elif sub == 0x03:
                    if not CONTROL_TIMEOUT_MIN_MS <= value <= CONTROL_TIMEOUT_MAX_MS:
                        self._abort(did, sub, ABORT_VALUE_OUT_OF_RANGE)
                        return
                    self._safe_cfg["ctrl_timeout_ms"] = value
                else:
                    self._abort(did, sub, ABORT_DID_NOT_EXIST)
                    return
                self._reply_service(
                    bytes((0x60, did & 0xFF, (did >> 8) & 0xFF, sub)) + value.to_bytes(4, "little")
                )
                return
            if (did, sub) in ((0x6073, 0x00), (0x607F, 0x00), (0x1018, 0x04), (0x3001, 0x01)) or did in (0x2010, 0x2015, 0x2021):
                self._abort(did, sub, ABORT_WRITE_NOT_ALLOWED)
                return
            if (did, sub) not in self._writable:
                self._abort(did, sub, ABORT_DID_NOT_EXIST)
                return
            low, high = self._ranges.get((did, sub), (0, 0xFFFF))
            if not low <= value <= high:
                self._abort(did, sub, ABORT_VALUE_OUT_OF_RANGE)
                return
            self._writable[(did, sub)] = value
            self._reply_service(
                bytes((0x60, did & 0xFF, (did >> 8) & 0xFF, sub))
                + value.to_bytes(4, "little")
            )
            return

        self._abort(did, sub, ABORT_DID_NOT_EXIST)

    def _handle_service(self, data: bytes) -> None:
        sid = data[0]
        self._mark_alive()
        if sid in (0x23, 0x40):
            self._handle_did(data)
            return
        if sid == 0xF1:
            sub_function = data[1]
            value = data[2]
            return_code = 0
            if sub_function == 0x10:
                if value not in (0, 1):
                    return_code = 0xE5
                elif value == 1:
                    if self._safety_latched or self._stop_active or self.enabled:
                        return_code = 0xE5
                    else:
                        self.error_code = 0
                        self.enabled = True
                        self._stop_kind = 0
                        if not self._zero_established:
                            self._align_until = time.monotonic() + ALIGNMENT_TIME_S
                        else:
                            self._apply_after = time.monotonic() + 0.02
                else:
                    self._begin_stop(2)
            elif sub_function == 0x11:
                if self.enabled or self._safety_latched or self._stop_active:
                    return_code = 0xE5
                elif value not in (0, 1):
                    return_code = 0xE5
                else:
                    self.mode = value
            elif sub_function == 0x12:
                self._safety_latched = False
                self._comm_lost = False
                self._begin_stop(2)
                time.sleep(0.004)
                self.error_code = 0
            elif sub_function == 0x13:
                self._begin_stop(1)
            elif sub_function == 0x14:
                self._begin_stop(2)
            elif sub_function == 0x15:
                self._safety_latched = True
                self._begin_stop(2)
            elif sub_function == 0xA2:
                self._abort(0x0000, 0xA2, ABORT_DID_NOT_EXIST)
                return
            else:
                self._abort(0x0000, sub_function, ABORT_DID_NOT_EXIST)
                return
            self._reply_service(
                bytes((0xF1, sub_function, value, return_code, 0, 0, 0, 0))
            )
            return
        if sid == 0xF2:
            self._abort(0x0000, 0xF2, ABORT_WRITE_NOT_ALLOWED)
            return
        self._abort(0x0000, 0x00, ABORT_DID_NOT_EXIST)

    # ------------------------------------------------------------- 控制报文

    def _apply_control(self, control: MotorControl) -> None:
        self._mark_alive()
        now = time.monotonic()
        if not control.enable:
            self.enabled = False
            self._align_until = None
            self._apply_after = None
            self.speed = 0
            self.iq = 0
            self._last_control = control
            return
        if self._safety_latched or self._stop_active:
            # 急停锁存 / 停车过程中拒绝 Control1 使能
            self.enabled = False
            self._last_control = control
            return
        if not self.enabled:
            self.enabled = True
            self.mode = control.mode
            self._stop_kind = 0
            if not self._zero_established:
                self._align_until = now + ALIGNMENT_TIME_S
            else:
                self._apply_after = now + 0.02
        elif control.mode != self.mode:
            self.enabled = False
            self.mode = control.mode
            self.enabled = True
            self._apply_after = now + 0.02
        self.target_position = control.target_position
        self.target_speed = control.target_speed
        self.target_iq = control.target_iq
        self._last_control = control

    # ------------------------------------------------------------- 传输接口

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("模拟串口已关闭")
        payload = bytes(data)
        with self._lock:
            if len(payload) == 3 and payload[0] == HEADER:
                mid = int.from_bytes(payload[1:3], "little") & 0x7FF
                reply = self._query_reply(mid)
                if reply is not None:
                    self._rx.extend(build_reply_tail(mid, reply))
                return len(payload)

            mid, body = parse_full_frame(payload)
            if not body:
                reply = self._query_reply(mid)
                if reply is not None:
                    self._rx.extend(build_reply_tail(mid, reply))
                return len(payload)

            if mid == 0x200 + self.node_id and len(body) == 5:
                self._apply_control(MotorControl.unpack(body))
            elif mid == 0x200 and body and len(body) % 5 == 0:
                index = self.node_id - 1
                if (index + 1) * 5 <= len(body):
                    self._apply_control(MotorControl.unpack(body[index * 5 : (index + 1) * 5]))
            elif mid in (0x600, 0x600 + self.node_id) and len(body) == 8:
                self._handle_service(body)
            return len(payload)

    def read(self, size: int = 1) -> bytes:
        """非阻塞读取：返回当前已排队的字节，没有则立即返回空。"""

        with self._lock:
            count = min(int(size), len(self._rx))
            return bytes(self._rx.popleft() for _ in range(count))

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def close(self) -> None:
        self.is_open = False
