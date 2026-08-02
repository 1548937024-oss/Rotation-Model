"""Protocol codec for Motor Module Serial Communication Protocol V1.2.

The protocol uses two frame forms:

* Master command: Header + MID + DLEN + Data + CRC
* Master query:   Header + MID
  Slave reply:                 DLEN + Data + CRC

Data fields explicitly documented as multi-byte values are little-endian.
The document does not explicitly state the byte order of the two-byte MID, so
the codec makes it configurable and defaults to little-endian.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Final


HEADER: Final[int] = 0xAA
MAX_DATA_LENGTH: Final[int] = 128


class ProtocolError(ValueError):
    """Raised when a frame or signal violates the protocol definition."""


class RunMode(IntEnum):
    POSITION = 0x00
    SPEED = 0x01


ERROR_NAMES: Final[dict[int, str]] = {
    0: "无故障",
    1: "供电电压过高",
    2: "供电电压过低",
    3: "驱动芯片过流",
    4: "模组温度过高",
    5: "电机缺相",
    6: "电机堵转",
}

ABORT_NAMES: Final[dict[int, str]] = {
    0x06010001: "不允许读取",
    0x06010002: "不允许写入",
    0x06020000: "Data ID 不存在",
}

MOTOR_CONFIG_RETURN_NAMES: Final[dict[int, str]] = {
    0x00: "无错误",
    0xE0: "电机处于使能状态",
    0xE2: "参数错误或 UID 不匹配",
    0xE4: "Node ID 无效",
    0xE5: "电机正在运行，需先下使能",
}


def _require_range(name: str, value: int, low: int, high: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ProtocolError(f"{name} 必须是整数")
    if not low <= value <= high:
        raise ProtocolError(f"{name} 必须位于 [{low}, {high}]，实际为 {value}")
    return value


def crc8(data: bytes, initial: int = 0x00) -> int:
    """CRC-8, polynomial 0x07, init 0x00, no reflection, xorout 0x00."""

    crc = initial & 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_mid(mid: int, byteorder: str = "little") -> bytes:
    _require_range("MID", mid, 0, 0x7FF)
    if byteorder not in {"little", "big"}:
        raise ProtocolError("MID 字节序只能是 little 或 big")
    return mid.to_bytes(2, byteorder)


def decode_mid(data: bytes, byteorder: str = "little") -> int:
    if len(data) != 2:
        raise ProtocolError("MID 必须恰好占 2 字节")
    value = int.from_bytes(data, byteorder)
    if value > 0x7FF:
        raise ProtocolError(f"MID 高位保留位非零：0x{value:04X}")
    return value


def build_query(mid: int, byteorder: str = "little") -> bytes:
    """Build the three-byte master query prefix."""

    return bytes((HEADER,)) + encode_mid(mid, byteorder)


def build_frame(mid: int, data: bytes, byteorder: str = "little") -> bytes:
    if not 1 <= len(data) <= MAX_DATA_LENGTH:
        raise ProtocolError("Data 长度必须为 1~128 字节")
    body = build_query(mid, byteorder) + bytes((len(data),)) + data
    return body + bytes((crc8(body),))


def parse_full_frame(frame: bytes, byteorder: str = "little") -> tuple[int, bytes]:
    if len(frame) < 6:
        raise ProtocolError("完整帧长度至少为 6 字节")
    if frame[0] != HEADER:
        raise ProtocolError(f"Header 应为 0xAA，实际为 0x{frame[0]:02X}")
    mid = decode_mid(frame[1:3], byteorder)
    dlen = frame[3]
    if not 1 <= dlen <= MAX_DATA_LENGTH:
        raise ProtocolError(f"非法 DLEN：{dlen}")
    if len(frame) != dlen + 5:
        raise ProtocolError(f"帧长不匹配：DLEN={dlen}，总长={len(frame)}")
    expected = crc8(frame[:-1])
    if frame[-1] != expected:
        raise ProtocolError(f"CRC 错误：收到 0x{frame[-1]:02X}，期望 0x{expected:02X}")
    return mid, frame[4:-1]


def parse_reply_tail(
    mid: int, tail: bytes, byteorder: str = "little"
) -> bytes:
    """Validate a slave DLEN+Data+CRC tail against the queried Header+MID."""

    if len(tail) < 3:
        raise ProtocolError("从机响应至少应包含 DLEN、1 字节 Data 和 CRC")
    dlen = tail[0]
    if not 1 <= dlen <= MAX_DATA_LENGTH:
        raise ProtocolError(f"从机返回非法 DLEN：{dlen}")
    if len(tail) != dlen + 2:
        raise ProtocolError(f"响应长度不匹配：DLEN={dlen}，响应尾长度={len(tail)}")
    crc_input = build_query(mid, byteorder) + tail[:-1]
    expected = crc8(crc_input)
    if tail[-1] != expected:
        raise ProtocolError(f"CRC 错误：收到 0x{tail[-1]:02X}，期望 0x{expected:02X}")
    return tail[1:-1]


def build_reply_tail(mid: int, data: bytes, byteorder: str = "little") -> bytes:
    """Build a simulated slave reply tail."""

    if not 1 <= len(data) <= MAX_DATA_LENGTH:
        raise ProtocolError("Data 长度必须为 1~128 字节")
    prefix = build_query(mid, byteorder)
    tail_without_crc = bytes((len(data),)) + data
    return tail_without_crc + bytes((crc8(prefix + tail_without_crc),))


@dataclass(frozen=True)
class MotorControl:
    enable: bool
    mode: int
    target_position: int
    target_speed: int
    target_iq: int

    def pack(self) -> bytes:
        mode = _require_range("运行模式", int(self.mode), 0, 0x3F)
        position = _require_range("目标位置", self.target_position, 0, 65535)
        speed = _require_range("目标速度", self.target_speed, -100, 100)
        iq = _require_range("目标 Iq", self.target_iq, -100, 100)
        flags = (0x80 if self.enable else 0x00) | mode
        return bytes((flags,)) + struct.pack("<Hbb", position, speed, iq)

    @classmethod
    def unpack(cls, data: bytes) -> "MotorControl":
        if len(data) != 5:
            raise ProtocolError("Motor Control Pack 必须为 5 字节")
        position, speed, iq = struct.unpack("<Hbb", data[1:])
        return cls(bool(data[0] & 0x80), data[0] & 0x3F, position, speed, iq)


def build_single_control_frame(
    node_id: int, control: MotorControl, byteorder: str = "little"
) -> bytes:
    _require_range("Node ID", node_id, 1, 31)
    return build_frame(0x200 + node_id, control.pack(), byteorder)


MAX_GROUP_MCP_COUNT: Final[int] = 8
MAX_MULTI_NODE_COUNT: Final[int] = 5


def build_group_control_frame(
    nodes: tuple[int, ...],
    control: MotorControl,
    byteorder: str = "little",
) -> bytes:
    """Build MSG_GroupControl1 (0x200) with one MCP slot per node position."""

    if not nodes:
        raise ProtocolError("多控报文至少需要一个 Node ID")
    node_set = set(int(node) for node in nodes)
    if len(node_set) > MAX_GROUP_MCP_COUNT:
        raise ProtocolError(f"多控报文最多支持 {MAX_GROUP_MCP_COUNT} 个节点")
    if not node_set or any(not 1 <= node <= MAX_GROUP_MCP_COUNT for node in node_set):
        raise ProtocolError(f"多控节点 ID 必须位于 1~{MAX_GROUP_MCP_COUNT}")

    max_node = max(node_set)
    packed = control.pack()
    idle = MotorControl(False, 0, 0, 0, 0).pack()
    data = bytearray()
    for node in range(1, max_node + 1):
        data.extend(packed if node in node_set else idle)
    return build_frame(0x200, bytes(data), byteorder)


@dataclass(frozen=True)
class MotionStatus:
    enabled: bool
    error: bool
    mode_or_error_code: int
    actual_position: int
    actual_speed: int
    actual_iq: int

    @property
    def mode(self) -> int | None:
        return None if self.error else self.mode_or_error_code

    @property
    def error_code(self) -> int | None:
        return self.mode_or_error_code if self.error else None

    @property
    def error_text(self) -> str:
        if not self.error:
            return "无故障"
        code = self.mode_or_error_code
        return ERROR_NAMES.get(code, f"未知故障码 {code}")


def parse_motion_status(data: bytes) -> MotionStatus:
    if len(data) != 5:
        raise ProtocolError(f"状态报文 1 的 Data 应为 5 字节，实际为 {len(data)}")
    position, speed, iq = struct.unpack("<Hbb", data[1:])
    return MotionStatus(
        enabled=bool(data[0] & 0x80),
        error=bool(data[0] & 0x40),
        mode_or_error_code=data[0] & 0x3F,
        actual_position=position,
        actual_speed=speed,
        actual_iq=iq,
    )


@dataclass(frozen=True)
class DeviceStatus:
    voltage_mv: int
    temperature_c: int


def parse_device_status(data: bytes) -> DeviceStatus:
    if len(data) != 3:
        raise ProtocolError(f"状态报文 2 的 Data 应为 3 字节，实际为 {len(data)}")
    voltage, temperature = struct.unpack("<Hb", data)
    return DeviceStatus(voltage, temperature)


def _pad_service(data: bytes) -> bytes:
    if len(data) > 8:
        raise ProtocolError("服务数据区不能超过 8 字节")
    return data.ljust(8, b"\x00")


def build_read_did(did: int, sub_id: int) -> bytes:
    _require_range("Data ID", did, 0, 0xFFFF)
    _require_range("Sub ID", sub_id, 0, 0xFF)
    return bytes((0x40, did & 0xFF, did >> 8, sub_id, 0, 0, 0, 0))


def build_write_did(did: int, sub_id: int, value: int) -> bytes:
    _require_range("Data ID", did, 0, 0xFFFF)
    _require_range("Sub ID", sub_id, 0, 0xFF)
    _require_range("写入值", value, 0, 0xFFFFFFFF)
    return bytes((0x23, did & 0xFF, did >> 8, sub_id)) + struct.pack("<I", value)


def build_motor_enable_service(enable: bool) -> bytes:
    return _pad_service(bytes((0xF1, 0x10, int(enable))))


def build_run_mode_service(mode: int) -> bytes:
    _require_range("运行模式", mode, 0, 0x3F)
    return _pad_service(bytes((0xF1, 0x11, mode)))


def build_set_zero_service() -> bytes:
    return _pad_service(bytes((0xF1, 0xA2)))


def build_set_node_id_service(new_node_id: int, uid: int) -> bytes:
    _require_range("新 Node ID", new_node_id, 1, 31)
    _require_range("UID", uid, 0, 0xFFFFFFFF)
    return bytes((0xF2, 0x00, 0x00, new_node_id)) + struct.pack("<I", uid)


@dataclass(frozen=True)
class ServiceResponse:
    srid: int
    raw: bytes
    ok: bool
    summary: str
    did: int | None = None
    sub_id: int | None = None
    value: int | None = None
    return_code: int | None = None


def parse_service_response(data: bytes) -> ServiceResponse:
    if len(data) != 8:
        raise ProtocolError(f"服务响应 Data 应为 8 字节，实际为 {len(data)}")
    srid = data[0]
    if srid in (0x43, 0x60):
        did = data[1] | (data[2] << 8)
        sub_id = data[3]
        value = int.from_bytes(data[4:8], "little")
        action = "读取" if srid == 0x43 else "写入"
        return ServiceResponse(
            srid, data, True, f"{action}成功：DID 0x{did:04X}/{sub_id} = {value}",
            did=did, sub_id=sub_id, value=value
        )
    if srid == 0x80:
        did = data[1] | (data[2] << 8)
        sub_id = data[3]
        code = int.from_bytes(data[4:8], "little")
        name = ABORT_NAMES.get(code, "未知读写错误")
        return ServiceResponse(
            srid, data, False,
            f"读写失败：DID 0x{did:04X}/{sub_id}，0x{code:08X} {name}",
            did=did, sub_id=sub_id, value=code, return_code=code
        )
    if srid == 0xF1:
        sub_function = data[1]
        return_code = data[3]
        name = MOTOR_CONFIG_RETURN_NAMES.get(return_code, "未知返回码")
        return ServiceResponse(
            srid, data, return_code == 0,
            f"电机配置 0x{sub_function:02X}：0x{return_code:02X} {name}",
            sub_id=sub_function, value=data[2], return_code=return_code
        )
    if srid == 0xF2:
        return_code = data[1]
        name = MOTOR_CONFIG_RETURN_NAMES.get(return_code, "未知返回码")
        return ServiceResponse(
            srid, data, return_code == 0,
            f"设置 Node ID：0x{return_code:02X} {name}，Node ID={data[3]}",
            value=data[3], return_code=return_code
        )
    return ServiceResponse(srid, data, False, f"未知服务响应 SRID=0x{srid:02X}")


def parse_hex_bytes(text: str, *, allow_empty: bool = False) -> bytes:
    cleaned = text.replace("0x", "").replace("0X", "")
    for separator in (" ", "\t", "\r", "\n", ",", "-", ":"):
        cleaned = cleaned.replace(separator, "")
    if not cleaned:
        if allow_empty:
            return b""
        raise ProtocolError("十六进制数据不能为空")
    if len(cleaned) % 2:
        raise ProtocolError("十六进制字符数必须为偶数")
    try:
        return bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ProtocolError("包含非法十六进制字符") from exc


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)
