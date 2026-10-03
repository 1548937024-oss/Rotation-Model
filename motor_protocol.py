"""MicroDriver V2.10 RS485 通讯协议 V1.4 编解码。

实现依据：`docs/外置驱动器通讯协议_V1.4_20261003.md`。

帧格式：

* 主控完整帧：Header(0xAA) + MID(2, 小端) + DLEN(1) + Data + CRC8
* 主控短询问：Header(0xAA) + MID(2, 小端)
* 从机应答：  DLEN(1) + Data + CRC8

应答 CRC 必须按 `AA + Response_MID + DLEN + Data` 计算，不能用请求 MID。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Final


HEADER: Final[int] = 0xAA
MAX_DATA_LENGTH: Final[int] = 128

DEFAULT_BAUDRATE: Final[int] = 460800
DEFAULT_NODE_ID: Final[int] = 1

# 协议量程（只读值，不是已验收的稳定运行承诺）
MAX_IQ_MA: Final[int] = 1400
MAX_SPEED_RPM: Final[int] = 18000
POSITION_MAX_LSB: Final[int] = 32767
POSITION_MIN_LSB: Final[int] = -32768

# 首次使能若未建零点会先做对齐，上位机需预留的时间
ALIGNMENT_TIME_S: Final[float] = 1.2


class ProtocolError(ValueError):
    """帧或信号不符合协议定义时抛出。"""


class RunMode(IntEnum):
    POSITION = 0x00
    SPEED = 0x01


# Status1 Byte0 bit5:0 在有故障时表示错误码
ERROR_CODE_NAMES: Final[dict[int, str]] = {
    3: "过流、过温或 nFAULT 综合故障",
    4: "外环速度失控（项目扩展）",
}

ABORT_NAMES: Final[dict[int, str]] = {
    0x06010001: "不允许读取",
    0x06010002: "不允许写入",
    0x06020000: "DID 不存在",
    0x06090030: "写入值超出范围",
}

MOTOR_CONFIG_RETURN_NAMES: Final[dict[int, str]] = {
    0x00: "成功",
    0xE5: "运行中或拒绝",
}


def _require_range(name: str, value: int, low: int, high: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ProtocolError(f"{name} 必须是整数")
    if not low <= value <= high:
        raise ProtocolError(f"{name} 必须位于 [{low}, {high}]，实际为 {value}")
    return value


def crc8(data: bytes, initial: int = 0x00) -> int:
    """CRC-8：多项式 0x07，初值 0x00，不反转，最终异或 0x00。"""

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
    """3 字节短询问头 `AA + MID`（依赖 2 ms 总线静默切帧）。"""

    return bytes((HEADER,)) + encode_mid(mid, byteorder)


def build_query_frame(mid: int, byteorder: str = "little") -> bytes:
    """完整零长度询问帧 `AA + MID + 00 + CRC`，V1.4 推荐用法。"""

    return build_frame(mid, b"", byteorder)


def build_frame(mid: int, data: bytes, byteorder: str = "little") -> bytes:
    if len(data) > MAX_DATA_LENGTH:
        raise ProtocolError(f"Data 长度不能超过 {MAX_DATA_LENGTH} 字节")
    body = build_query(mid, byteorder) + bytes((len(data),)) + bytes(data)
    return body + bytes((crc8(body),))


def parse_full_frame(frame: bytes, byteorder: str = "little") -> tuple[int, bytes]:
    if len(frame) < 5:
        raise ProtocolError("完整帧长度至少为 5 字节")
    if frame[0] != HEADER:
        raise ProtocolError(f"Header 应为 0xAA，实际为 0x{frame[0]:02X}")
    mid = decode_mid(frame[1:3], byteorder)
    dlen = frame[3]
    if dlen > MAX_DATA_LENGTH:
        raise ProtocolError(f"非法 DLEN：{dlen}")
    if len(frame) != dlen + 5:
        raise ProtocolError(f"帧长不匹配：DLEN={dlen}，总长={len(frame)}")
    expected = crc8(frame[:-1])
    if frame[-1] != expected:
        raise ProtocolError(f"CRC 错误：收到 0x{frame[-1]:02X}，期望 0x{expected:02X}")
    return mid, bytes(frame[4:-1])


def parse_reply_tail(mid: int, tail: bytes, byteorder: str = "little") -> bytes:
    """校验从机 `DLEN + Data + CRC` 应答尾。

    `mid` 必须是对应请求的 Response MID（Status1=0x180+NID、Status2=0x280+NID、
    服务响应=0x580+NID），而不是请求 MID。
    """

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
    return bytes(tail[1:-1])


def build_reply_tail(mid: int, data: bytes, byteorder: str = "little") -> bytes:
    """构造模拟从机应答尾，CRC 前缀使用 Response MID。"""

    if not 1 <= len(data) <= MAX_DATA_LENGTH:
        raise ProtocolError("Data 长度必须为 1~128 字节")
    tail_without_crc = bytes((len(data),)) + bytes(data)
    return tail_without_crc + bytes((crc8(build_query(mid, byteorder) + tail_without_crc),))


@dataclass(frozen=True)
class MotorControl:
    """Control1 / GroupControl1 的 5 字节 MCP。"""

    enable: bool
    mode: int
    target_position: int
    target_speed: int
    target_iq: int

    def pack(self) -> bytes:
        mode = _require_range("运行模式", int(self.mode), 0, 0x3F)
        position = _require_range(
            "目标位置", self.target_position, POSITION_MIN_LSB, POSITION_MAX_LSB
        )
        speed = _require_range("目标速度", self.target_speed, -100, 100)
        iq = _require_range("目标 Iq 限幅", self.target_iq, -100, 100)
        flags = (0x80 if self.enable else 0x00) | mode
        return bytes((flags,)) + struct.pack("<hbb", position, speed, iq)

    @classmethod
    def unpack(cls, data: bytes) -> "MotorControl":
        if len(data) != 5:
            raise ProtocolError("Motor Control Pack 必须为 5 字节")
        position, speed, iq = struct.unpack("<hbb", data[1:])
        return cls(bool(data[0] & 0x80), data[0] & 0x3F, position, speed, iq)


def build_single_control_frame(
    node_id: int, control: MotorControl, byteorder: str = "little"
) -> bytes:
    _require_range("Node ID", node_id, 1, 31)
    return build_frame(0x200 + node_id, control.pack(), byteorder)


MAX_GROUP_MCP_COUNT: Final[int] = 8


def build_multi_control_frame(
    controls: dict[int, MotorControl],
    byteorder: str = "little",
) -> bytes:
    """构造 GroupControl1 (MID 0x200)，每个节点一个 5 字节 MCP 槽位。

    当前固件 Node ID 编译期固定为 1，本函数保留用于协议广播能力与报文调试。
    """

    if not controls:
        raise ProtocolError("GroupControl1 报文至少需要一个节点")
    node_set = {int(node) for node in controls}
    if len(node_set) > MAX_GROUP_MCP_COUNT:
        raise ProtocolError(f"GroupControl1 最多支持 {MAX_GROUP_MCP_COUNT} 个节点")
    if any(not 1 <= node <= MAX_GROUP_MCP_COUNT for node in node_set):
        raise ProtocolError(f"节点 ID 必须位于 1~{MAX_GROUP_MCP_COUNT}")

    idle = MotorControl(False, 0, 0, 0, 0).pack()
    data = bytearray()
    for node in range(1, max(node_set) + 1):
        control = controls.get(node)
        data.extend(control.pack() if control is not None else idle)
    return build_frame(0x200, bytes(data), byteorder)


def build_group_control_frame(
    nodes: tuple[int, ...],
    control: MotorControl,
    byteorder: str = "little",
) -> bytes:
    """对多个节点下发同一个 MCP。"""

    if not nodes:
        raise ProtocolError("GroupControl1 报文至少需要一个 Node ID")
    return build_multi_control_frame({int(node): control for node in nodes}, byteorder)


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
        return ERROR_CODE_NAMES.get(code, f"未知故障码 {code}")

    @property
    def speed_rpm(self) -> int:
        """ActualSpeed 百分比换算成 rpm（18000 rpm 为协议量程）。"""

        return round(self.actual_speed * MAX_SPEED_RPM / 100)

    @property
    def iq_ma_estimate(self) -> int:
        """ActualIq 百分比换算成相电流峰值 mA（占 Max Iq 百分比，非 dq 电流）。"""

        return round(abs(self.actual_iq) * MAX_IQ_MA / 100)


def parse_motion_status(data: bytes) -> MotionStatus:
    if len(data) != 5:
        raise ProtocolError(f"Status1 的 Data 应为 5 字节，实际为 {len(data)}")
    position, speed, iq = struct.unpack("<hbb", data[1:])
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
        raise ProtocolError(f"Status2 的 Data 应为 3 字节，实际为 {len(data)}")
    voltage, temperature = struct.unpack("<Hb", data)
    return DeviceStatus(voltage, temperature)


def _pad_service(data: bytes) -> bytes:
    if len(data) > 8:
        raise ProtocolError("服务数据区不能超过 8 字节")
    return bytes(data).ljust(8, b"\x00")


def build_read_did(did: int, sub_id: int) -> bytes:
    _require_range("DID", did, 0, 0xFFFF)
    _require_range("Sub ID", sub_id, 0, 0xFF)
    return bytes((0x40, did & 0xFF, did >> 8, sub_id, 0, 0, 0, 0))


def build_write_did(did: int, sub_id: int, value: int) -> bytes:
    _require_range("DID", did, 0, 0xFFFF)
    _require_range("Sub ID", sub_id, 0, 0xFF)
    _require_range("写入值", value, 0, 0xFFFFFFFF)
    return bytes((0x23, did & 0xFF, did >> 8, sub_id)) + struct.pack("<I", value)


def build_motor_enable_service(enable: bool) -> bytes:
    """0xF1/0x10 使能服务。首次使能若未建零点会立即启动约 1200 ms 对齐。"""

    return _pad_service(bytes((0xF1, 0x10, int(bool(enable)))))


def build_run_mode_service(mode: int) -> bytes:
    """0xF1/0x11 运行模式服务：0 位置，1 速度。"""

    _require_range("运行模式", mode, 0, 0x3F)
    return _pad_service(bytes((0xF1, 0x11, mode)))


def build_clear_fault_service() -> bytes:
    """0xF1/0x12 清错服务（V1.4 推荐入口，替代旧的 0x6040 控制字）。"""

    return _pad_service(bytes((0xF1, 0x12, 0x01)))


def build_save_parameters() -> bytes:
    """0x1010 保存参数到 EEPROM（要求电机未运行）。"""

    return build_write_did(0x1010, 0x00, 0x00000001)


def build_reload_parameters() -> bytes:
    """0x1011 从 EEPROM 重新加载参数（不是恢复出厂默认值）。"""

    return build_write_did(0x1011, 0x00, 0x00000001)


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
            srid,
            bytes(data),
            True,
            f"{action}成功：DID 0x{did:04X}/{sub_id} = {value}",
            did=did,
            sub_id=sub_id,
            value=value,
        )
    if srid == 0x80:
        did = data[1] | (data[2] << 8)
        sub_id = data[3]
        code = int.from_bytes(data[4:8], "little")
        name = ABORT_NAMES.get(code, "未知读写错误")
        return ServiceResponse(
            srid,
            bytes(data),
            False,
            f"读写失败：DID 0x{did:04X}/{sub_id}，0x{code:08X} {name}",
            did=did,
            sub_id=sub_id,
            value=code,
            return_code=code,
        )
    if srid == 0xF1:
        sub_function = data[1]
        return_code = data[3]
        name = MOTOR_CONFIG_RETURN_NAMES.get(return_code, "未知返回码")
        return ServiceResponse(
            srid,
            bytes(data),
            return_code == 0,
            f"0xF1 服务 0x{sub_function:02X}：0x{return_code:02X} {name}",
            sub_id=sub_function,
            value=data[2],
            return_code=return_code,
        )
    return ServiceResponse(srid, bytes(data), False, f"未知服务响应 SRID=0x{srid:02X}")


@dataclass(frozen=True)
class DidSpec:
    """DID 目录条目，供上位机生成参数界面与做范围校验。"""

    did: int
    sub: int
    name: str
    unit: str = ""
    writable: bool = False
    minimum: int = 0
    maximum: int = 0xFFFFFFFF
    default: int | None = None
    signed: bool = False
    note: str = ""

    @property
    def key(self) -> tuple[int, int]:
        return (self.did, self.sub)

    @property
    def label(self) -> str:
        suffix = f" {self.unit}" if self.unit else ""
        return f"{self.name}{suffix}"


DID_CATALOG: Final[tuple[DidSpec, ...]] = (
    DidSpec(0x1018, 0x04, "Device UID", "", writable=False, maximum=0xFFFFFFFF),
    DidSpec(0x3001, 0x01, "当前波特率", "bit/s", writable=False, maximum=0xFFFFFFFF),
    DidSpec(
        0x6073, 0x00, "Max Iq Current", "mA", writable=False,
        minimum=0, maximum=65535, note="只读，当前 1400 mA",
    ),
    DidSpec(
        0x607F, 0x00, "Max Speed", "rpm", writable=False,
        minimum=0, maximum=65535, note="只读，当前 18000 rpm",
    ),
    DidSpec(
        0x6073, 0x01, "堵转电流", "mA", writable=True,
        minimum=0, maximum=3000, default=1500,
        note="写入 RAM；当前过流保护入口仍是硬件 OCP/nFAULT",
    ),
    DidSpec(
        0x6073, 0x03, "堵转时间", "ms", writable=True,
        minimum=0, maximum=65535, default=1000,
    ),
    DidSpec(
        0x202D, 0x01, "欠压阈值", "0.1V", writable=True,
        minimum=40, maximum=110, default=90,
    ),
    DidSpec(
        0x202D, 0x02, "欠压时间", "ms", writable=True,
        minimum=0, maximum=10000, default=500,
    ),
    DidSpec(
        0x202D, 0x03, "过压阈值", "0.1V", writable=True,
        minimum=80, maximum=150, default=520,
    ),
    DidSpec(
        0x202D, 0x04, "过压时间", "ms", writable=True,
        minimum=0, maximum=10000, default=500,
    ),
    DidSpec(
        0x2016, 0x02, "过温阈值", "°C", writable=True,
        minimum=55, maximum=110, default=90,
    ),
    DidSpec(
        0x2016, 0x03, "过温时间", "ms", writable=True,
        minimum=0, maximum=60000, default=1000,
    ),
    DidSpec(
        0x2016, 0x04, "过温恢复", "°C", writable=True,
        minimum=50, maximum=110, default=75,
    ),
    DidSpec(0x2010, 0x01, "A 相电流", "mA", writable=False, signed=True, minimum=-32768, maximum=32767),
    DidSpec(0x2010, 0x02, "B 相电流", "mA", writable=False, signed=True, minimum=-32768, maximum=32767),
    DidSpec(0x2010, 0x03, "C 相电流", "mA", writable=False, signed=True, minimum=-32768, maximum=32767),
    DidSpec(0x2010, 0x04, "相电流峰值", "mA", writable=False, minimum=0, maximum=65535),
    DidSpec(0x2015, 0x01, "编码器累计", "count", writable=False, signed=True, minimum=-2147483648, maximum=2147483647),
    DidSpec(0x2015, 0x02, "编码器 Z 计数", "count", writable=False, minimum=0, maximum=0xFFFFFFFF),
)

DID_BY_KEY: Final[dict[tuple[int, int], DidSpec]] = {spec.key: spec for spec in DID_CATALOG}


def decode_did_value(did: int, sub_id: int, raw_u32: int) -> int:
    """按 DID 类型解释响应中的 32 位值（S16/S32 需要符号扩展）。"""

    spec = DID_BY_KEY.get((did, sub_id))
    value = raw_u32 & 0xFFFFFFFF
    if spec is None:
        return value
    if spec.signed and spec.did == 0x2015:
        return value - 0x100000000 if value & 0x80000000 else value
    if spec.signed and spec.did == 0x2010:
        value &= 0xFFFF
        return value - 0x10000 if value & 0x8000 else value
    return value


def format_did_value(did: int, sub_id: int, raw_u32: int) -> str:
    spec = DID_BY_KEY.get((did, sub_id))
    value = decode_did_value(did, sub_id, raw_u32)
    if spec and spec.did == 0x1018:
        return f"0x{value & 0xFFFFFFFF:08X}"
    if spec and spec.unit:
        return f"{value} {spec.unit}"
    return str(value)


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
