"""MicroDriver V2.10 单线程 RS485 通信工作线程。

所有串口收发都在本线程串行执行，界面线程只通过命令队列交互，避免读写竞争。
协议口径见 `motor_protocol.py` 与 `docs/外置驱动器通讯协议_V1.4_20261003.md`。
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, Final

from motor_protocol import (
    DEFAULT_NODE_ID,
    MAX_DATA_LENGTH,
    MotorControl,
    ProtocolError,
    build_clear_fault_service,
    build_frame,
    build_motor_enable_service,
    build_query,
    build_query_frame,
    build_read_did,
    build_reload_parameters,
    build_run_mode_service,
    build_save_parameters,
    build_single_control_frame,
    build_write_did,
    decode_did_value,
    format_hex,
    parse_device_status,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)
from simulator import SimulatedTransport

SIMULATOR_PORT = "__SIMULATOR__"

# 应答尾的预期 DLEN，用于在接收流错位时重新同步
STATUS1_DLEN: Final[int] = 5
STATUS2_DLEN: Final[int] = 3
SERVICE_DLEN: Final[int] = 8

# 连接启动窗口：先做握手，拿到连续两个完整状态周期再进入周期轮询
STARTUP_WINDOW_S: Final[float] = 3.0
STARTUP_REQUIRED_OK: Final[int] = 2
STARTUP_FIRST_PROBE_S: Final[float] = 0.2
STARTUP_PROBE_INTERVAL_S: Final[float] = 0.2

# 单次询问最多发 1 次 + 自动重试 2 次
QUERY_ATTEMPTS: Final[int] = 3
REPLY_TIMEOUT_S: Final[float] = 0.06

# 固件在收完 SvcRequest 后【主动下发】SvcResponse（见 protocol.c
# proto_handle_svc -> proto_send(MSGID_SVCRSP_BASE + NID)）。
# MID 0x580+NID 不是可查询报文，所以这里只等主动应答，不发 0x581 查询。
# 超时要覆盖 USB-RS485 常见的 16 ms 延迟定时器与固件主循环抖动。
SERVICE_TIMEOUT_S: Final[float] = 0.2
# 兼容极少数"查询式"固件实现时才置 True；本项目固件为主动下发，保持 False。
SERVICE_QUERY_FALLBACK: Final[bool] = False

# 扩展遥测 DID：三相电流、峰值电流、编码器累计、Z 计数
TELEMETRY_DIDS: tuple[tuple[int, int], ...] = (
    (0x2010, 0x01),
    (0x2010, 0x02),
    (0x2010, 0x03),
    (0x2010, 0x04),
    (0x2015, 0x01),
    (0x2015, 0x02),
)


@dataclass
class WorkerCommand:
    name: str
    args: tuple[Any, ...] = ()


class DeviceWorker:
    """持有 transport 的通信线程。"""

    def __init__(self) -> None:
        self.commands: queue.Queue[WorkerCommand] = queue.Queue()
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="microdriver-serial", daemon=True)
        self.transport: Any | None = None
        self.node_id = DEFAULT_NODE_ID
        self.poll_enabled = False
        self.poll_interval = 0.02
        self._next_poll = float("inf")
        self.cyclic_enabled = False
        self.cyclic_interval = 0.02
        self._next_control = float("inf")
        self.latest_control = MotorControl(False, 0, 0, 0, 20)
        self.service_timeout_s = SERVICE_TIMEOUT_S
        self._script_steps: list[tuple] | None = None
        self._script_index = 0
        self._script_next_time = float("inf")
        # 连接启动窗口握手状态
        self.startup_pending = False
        self.startup_ok_count = 0
        self.startup_deadline = 0.0
        self._next_startup_probe = float("inf")
        self._thread.start()

    # ---------------------------------------------------------------- 公共接口

    def submit(self, name: str, *args: Any) -> None:
        self.commands.put(WorkerCommand(name, args))

    def shutdown(self) -> None:
        self.submit("shutdown")
        self._thread.join(timeout=1.5)

    # ---------------------------------------------------------------- 基础收发

    def _emit(self, kind: str, payload: Any) -> None:
        self.events.put((kind, payload))

    def _log(self, direction: str, data: bytes, note: str = "") -> None:
        timestamp = time.strftime("%H:%M:%S")
        suffix = f"  {note}" if note else ""
        self._emit("log", f"{timestamp} {direction:<2}  {format_hex(data)}{suffix}")

    def _expected_dlen(self, mid: int) -> int | None:
        """已知应答报文的预期 DLEN；未知 MID 返回 None（按 CRC 扫描任意长度）。"""

        if mid == 0x180 + self.node_id:
            return STATUS1_DLEN
        if mid == 0x280 + self.node_id:
            return STATUS2_DLEN
        if mid == 0x580 + self.node_id:
            return SERVICE_DLEN
        return None

    def _collect_reply(
        self, mid: int, expected_dlen: int | None, deadline: float
    ) -> tuple[bytes, int]:
        """在字节流中按预期 DLEN 重新同步，并用 CRC 确认候选应答。

        从机应答尾没有 Header/MID，一旦错位只能靠 CRC 恢复。这里维护一个滚动窗口：

        * 只在字节等于预期 DLEN 的位置尝试解析（未知 MID 时接受合法 DLEN 范围）；
        * 长度足够后先做 CRC，失败就右移一个字节继续找；
        * 返回 Data 与被丢弃的字节数，供上层记录错位统计。

        这样启动窗口里读到的 0x84 之类字节会被当作噪声丢弃，而不是当成非法 DLEN 直接报错。
        """

        buffer = bytearray()
        position = 0
        discarded = 0
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"等待 MID 0x{mid:03X} 应答超时，已丢弃 {discarded} 字节"
                )
            chunk = self.transport.read(1)
            if not chunk:
                time.sleep(0.001)
                continue
            buffer.extend(chunk)
            while position < len(buffer):
                dlen = buffer[position]
                if expected_dlen is not None:
                    if dlen != expected_dlen:
                        position += 1
                        discarded += 1
                        continue
                elif not 1 <= dlen <= MAX_DATA_LENGTH:
                    position += 1
                    discarded += 1
                    continue
                total = position + dlen + 2
                if len(buffer) < total:
                    break
                candidate = bytes(buffer[position:total])
                try:
                    data = parse_reply_tail(mid, candidate)
                except ProtocolError:
                    position += 1
                    discarded += 1
                    continue
                del buffer[:total]
                return data, discarded
            # 已确认的噪声字节及时释放，避免窗口无限增长
            if position > 256:
                del buffer[:position]
                position = 0

    def _write(self, data: bytes, note: str = "") -> None:
        if self.transport is None:
            raise RuntimeError("串口未连接")
        written = self.transport.write(data)
        if written != len(data):
            raise OSError(f"串口仅写入 {written}/{len(data)} 字节")
        self._log("TX", data, note)

    def _drain_input(self) -> None:
        """发请求前丢弃接收缓冲里的残留字节，避免旧数据污染本次应答。"""

        if self.transport is None:
            return
        try:
            self.transport.reset_input_buffer()
        except Exception:
            pass

    def _query(
        self,
        mid: int,
        note: str = "",
        *,
        short: bool = False,
        expect_dlen: int | None = None,
        attempts: int = QUERY_ATTEMPTS,
    ) -> bytes:
        """询问从机并解析应答尾。

        V1.4 推荐使用完整零长度帧 `AA MID 00 CRC`；`short=True` 时才发 3 字节短询问。
        每次发请求前先清输入缓冲，读取时按预期 DLEN 重新同步，最多发 1 次 + 重试 2 次。
        """

        if expect_dlen is None:
            expect_dlen = self._expected_dlen(mid)
        last_error: Exception | None = None
        for attempt in range(1, max(1, attempts) + 1):
            self._drain_input()
            query = build_query(mid) if short else build_query_frame(mid)
            self._write(query, note or f"查询 MID 0x{mid:03X}")
            try:
                data, discarded = self._collect_reply(
                    mid, expect_dlen, time.monotonic() + REPLY_TIMEOUT_S
                )
            except TimeoutError as exc:
                last_error = exc
                continue
            if discarded:
                self._emit(
                    "resync",
                    (mid, discarded, attempt),
                )
            return data
        raise last_error or TimeoutError(f"MID 0x{mid:03X} 无应答")

    # ---------------------------------------------------------------- 控制报文

    def _send_control(self, control: MotorControl, note: str = "Control1") -> None:
        frame = build_single_control_frame(self.node_id, control)
        self._write(frame, f"{note}（Node {self.node_id}）")

    def _flush_due_controls(self, now: float) -> None:
        if not self.cyclic_enabled:
            return
        burst = 0
        while self.cyclic_enabled and now >= self._next_control and burst < 10:
            self._send_control(self.latest_control, "周期 Control1")
            self._next_control += self.cyclic_interval
            burst += 1
        if now >= self._next_control:
            self._next_control = now + self.cyclic_interval

    # ---------------------------------------------------------------- 服务请求

    def _service(self, payload: bytes, note: str = "服务请求") -> Any:
        """发送 SvcRequest 并接收固件主动下发的 SvcResponse。

        固件 `proto_handle_svc()` 解析完请求后立即 `proto_send(0x580+NID, …)`，
        应答不带 Header/MID，只有 DLEN(8)+Data+CRC。MID 0x580+NID 不是可查询报文
        （固件 MID 分发里没有它），所以这里只等主动应答，不再发 0x581 查询 ——
        旧 V1.2 上位机的"查询式"流程会先清掉刚到达的正确应答，导致 read did 失败。

        等待窗口覆盖 USB-RS485 的 16 ms 延迟定时器；超时才重发请求，最多 1+2 次。
        """

        request_mid = 0x600 + self.node_id
        response_mid = 0x580 + self.node_id
        last_error: Exception | None = None
        for attempt in range(1, QUERY_ATTEMPTS + 1):
            self._drain_input()
            self._write(build_frame(request_mid, payload), note)
            try:
                data, discarded = self._collect_reply(
                    response_mid,
                    SERVICE_DLEN,
                    time.monotonic() + self.service_timeout_s,
                )
            except TimeoutError as exc:
                last_error = exc
                continue
            if discarded:
                self._emit("resync", (response_mid, discarded, attempt))
            response = parse_service_response(data)
            self._emit("service", response)
            return response

        if SERVICE_QUERY_FALLBACK:
            data = self._query(
                response_mid, "查询服务响应", expect_dlen=SERVICE_DLEN, attempts=2
            )
            response = parse_service_response(data)
            self._emit("service", response)
            return response
        raise last_error or TimeoutError("未收到服务响应")

    def _read_did(self, did: int, sub_id: int, note: str = "") -> Any:
        label = note or f"读取 DID 0x{did:04X}/{sub_id}"
        return self._service(build_read_did(did, sub_id), label)

    def _read_telemetry(self) -> None:
        """读取 0x2010 / 0x2015 扩展遥测，返回原始值供界面换算与滤波。"""

        values: dict[tuple[int, int], int] = {}
        for did, sub_id in TELEMETRY_DIDS:
            response = self._read_did(did, sub_id)
            if response.ok and response.value is not None:
                values[(did, sub_id)] = decode_did_value(did, sub_id, response.value)
        self._emit("telemetry", values)

    # ---------------------------------------------------------------- 状态轮询

    def _poll_once(self) -> None:
        motion_data = self._query(
            0x180 + self.node_id, "查询 Status1", expect_dlen=STATUS1_DLEN
        )
        self._emit("motion_status", parse_motion_status(motion_data))
        device_data = self._query(
            0x280 + self.node_id, "查询 Status2", expect_dlen=STATUS2_DLEN
        )
        self._emit("device_status", parse_device_status(device_data))

    def _startup_probe(self) -> tuple[bool, str]:
        """启动窗口内做一轮完整 Status1+Status2 握手。"""

        try:
            motion_data = self._query(
                0x180 + self.node_id, "启动握手 Status1", expect_dlen=STATUS1_DLEN
            )
            motion = parse_motion_status(motion_data)
            device_data = self._query(
                0x280 + self.node_id, "启动握手 Status2", expect_dlen=STATUS2_DLEN
            )
            device = parse_device_status(device_data)
        except (TimeoutError, ProtocolError) as exc:
            self.startup_ok_count = 0
            return False, str(exc)
        self._emit("motion_status", motion)
        self._emit("device_status", device)
        self.startup_ok_count += 1
        return True, ""

    def _finish_startup(self, reason: str) -> None:
        self.startup_pending = False
        self._next_poll = time.monotonic()
        elapsed = max(0.0, STARTUP_WINDOW_S - (self.startup_deadline - time.monotonic()))
        self._emit(
            "startup",
            {
                "phase": "done",
                "ok_count": self.startup_ok_count,
                "required": STARTUP_REQUIRED_OK,
                "reason": reason,
                "elapsed_s": round(elapsed, 2),
            },
        )
        if reason == "timeout":
            self._emit(
                "warning",
                f"启动窗口 {STARTUP_WINDOW_S:.0f} s 内未连续收到 {STARTUP_REQUIRED_OK} 轮合法状态，"
                "仍开始周期轮询；请留意接收错位统计。",
            )

    # ---------------------------------------------------------------- 连接管理

    def _connect(self, port: str, baudrate: int, node_id: int) -> None:
        if self.transport is not None:
            self._disconnect(False)
        self.node_id = int(node_id)
        self.latest_control = MotorControl(False, 0, 0, 0, 20)
        if port == SIMULATOR_PORT:
            self.transport = SimulatedTransport(node_id=self.node_id, baudrate=baudrate)
            description = "模拟设备（MicroDriver V2.10）"
        else:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("缺少 pyserial，请先运行 setup_and_run.bat") from exc
            self.transport = serial.Serial(
                port=port,
                baudrate=int(baudrate),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.2,
            )
            self.transport.reset_input_buffer()
            description = f"{port} @ {int(baudrate)} 8N1"
        # 启动窗口：先握手确认链路干净，再让周期轮询接管。
        now = time.monotonic()
        self.startup_pending = True
        self.startup_ok_count = 0
        self.startup_deadline = now + STARTUP_WINDOW_S
        self._next_startup_probe = now + STARTUP_FIRST_PROBE_S
        self._emit("connected", description)
        self._emit(
            "startup",
            {
                "phase": "start",
                "ok_count": 0,
                "required": STARTUP_REQUIRED_OK,
                "window_s": STARTUP_WINDOW_S,
            },
        )
        self._log("--", b"", f"已连接 {description}，Node {self.node_id}，MID 小端")

    def _disconnect(self, disable_first: bool) -> None:
        if self.transport is None:
            return
        self.poll_enabled = False
        self.cyclic_enabled = False
        self._script_steps = None
        self.startup_pending = False
        self.startup_ok_count = 0
        if disable_first:
            safe = MotorControl(False, self.latest_control.mode, self.latest_control.target_position, 0, 0)
            for attempt in range(2):
                try:
                    self._send_control(safe, "断开前下使能")
                except Exception as exc:
                    self._emit("warning", f"断开前下使能发送失败：{exc}")
                    break
                if attempt == 0:
                    time.sleep(0.003)
        try:
            self.transport.close()
        finally:
            self.transport = None
            self._emit("disconnected", None)

    def _handle_transport_error(self, exc: Exception) -> None:
        if isinstance(exc, TimeoutError):
            return
        if isinstance(exc, OSError) or exc.__class__.__name__ == "SerialException":
            try:
                self._disconnect(False)
            except Exception:
                self.transport = None
                self.poll_enabled = False
                self.cyclic_enabled = False
                self._emit("disconnected", None)
            self._emit("warning", f"串口连接异常，已断开，请手动重连：{exc}")

    # ---------------------------------------------------------------- 调试脚本

    def _start_script(self, steps: list[tuple]) -> None:
        self._script_steps = list(steps)
        self._script_index = 0
        self._script_next_time = time.monotonic()

    def _advance_script(self, now: float) -> None:
        if self._script_steps is None:
            return
        while now >= self._script_next_time:
            if self._script_index >= len(self._script_steps):
                self._emit("notice", "脚本执行完成")
                self._script_steps = None
                return
            step = self._script_steps[self._script_index]
            self._script_index += 1
            kind = step[0]
            if kind == "delay":
                self._script_next_time = now + float(step[1])
                return
            if kind == "frame":
                _, mid, data = step
                self._write(build_frame(mid, data), f"脚本 [{self._script_index}] 完整帧 MID 0x{mid:03X}")
            elif kind == "query":
                _, mid, short = step
                data = self._query(mid, f"脚本 [{self._script_index}] 查询 MID 0x{mid:03X}", short=short)
                self._emit("raw_reply", (mid, data))
            else:
                raise RuntimeError(f"未知脚本步骤：{kind}")
            self._script_next_time = now

    # ---------------------------------------------------------------- 命令分发

    def _handle(self, command: WorkerCommand) -> None:
        name, args = command.name, command.args
        if name == "connect":
            self._connect(*args)
        elif name == "disconnect":
            self._disconnect(*args)
        elif name == "send_control":
            self.latest_control = args[0]
            self._send_control(self.latest_control)
        elif name == "emergency_disable":
            safe = MotorControl(False, self.latest_control.mode, self.latest_control.target_position, 0, 0)
            self.latest_control = safe
            self.cyclic_enabled = False
            self._send_control(safe, "立即下使能")
            time.sleep(0.003)
            self._send_control(safe, "立即下使能（重复）")
            self._emit("disabled", None)
        elif name == "configure_cyclic":
            was_enabled = self.cyclic_enabled
            self.latest_control = args[0]
            self.cyclic_enabled = bool(args[1])
            self.cyclic_interval = max(0.001, float(args[2]))
            if not was_enabled and self.cyclic_enabled:
                self._next_control = time.monotonic()
        elif name == "configure_poll":
            self.poll_enabled = bool(args[0])
            self.poll_interval = max(0.005, float(args[1]))
            self._next_poll = time.monotonic()
        elif name == "query_status":
            self._poll_once()
        elif name == "service":
            self._service(args[0], args[1] if len(args) > 1 else "服务请求")
        elif name == "motor_enable":
            self._service(build_motor_enable_service(bool(args[0])), "0xF1 使能服务")
        elif name == "run_mode":
            self._service(build_run_mode_service(int(args[0])), "0xF1 运行模式")
        elif name == "clear_fault":
            self._service(build_clear_fault_service(), "0xF1 清错")
        elif name == "save_parameters":
            self._service(build_save_parameters(), "保存参数到 EEPROM (0x1010)")
        elif name == "reload_parameters":
            self._service(build_reload_parameters(), "从 EEPROM 重新加载参数 (0x1011)")
        elif name == "read_did":
            self._read_did(int(args[0]), int(args[1]), args[2] if len(args) > 2 else "")
        elif name == "read_telemetry":
            self._read_telemetry()
        elif name == "raw_script":
            self._start_script(args[0])
        elif name == "raw_frame":
            mid, data = args
            self._write(build_frame(mid, data), f"原始完整帧 MID 0x{mid:03X}")
        elif name == "raw_query":
            mid, short = args
            data = self._query(mid, f"原始查询 MID 0x{mid:03X}", short=bool(short))
            self._emit("raw_reply", (mid, data))
        elif name == "write_did":
            did, sub_id, value, note = args
            self._service(build_write_did(int(did), int(sub_id), int(value)), note)
        elif name == "shutdown":
            self._disconnect(False)
            self._stop.set()
        else:
            raise RuntimeError(f"未知工作线程命令：{name}")

    # ---------------------------------------------------------------- 主循环

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            deadlines = []
            if self.transport is not None and self.startup_pending:
                deadlines.append(self._next_startup_probe)
            elif self.transport is not None and self.poll_enabled:
                deadlines.append(self._next_poll)
            if self.transport is not None and self.cyclic_enabled:
                deadlines.append(self._next_control)
            if self.transport is not None and self._script_steps is not None:
                deadlines.append(self._script_next_time)
            timeout = 0.05 if not deadlines else max(0.0, min(0.05, min(deadlines) - now))
            try:
                command = self.commands.get(timeout=timeout)
            except queue.Empty:
                command = None
            if command is not None:
                try:
                    self._handle(command)
                except Exception as exc:
                    self._emit("error", f"{command.name} 失败：{exc}")
                    self._handle_transport_error(exc)
            if self.transport is None:
                continue
            now = time.monotonic()
            if self._script_steps is not None:
                try:
                    self._advance_script(now)
                except Exception as exc:
                    self._script_steps = None
                    self._emit("error", f"脚本执行失败：{exc}")
                    self._handle_transport_error(exc)
            if self.cyclic_enabled:
                try:
                    self._flush_due_controls(now)
                except Exception as exc:
                    self.cyclic_enabled = False
                    self._emit("error", f"周期控制停止：{exc}")
                    self._handle_transport_error(exc)
            if self.startup_pending:
                if now >= self._next_startup_probe:
                    try:
                        ok, detail = self._startup_probe()
                    except Exception as exc:
                        ok, detail = False, str(exc)
                        self._emit("warning", f"启动握手失败：{exc}")
                        self._handle_transport_error(exc)
                        if self.transport is None:
                            continue
                    now = time.monotonic()
                    self._emit(
                        "startup",
                        {
                            "phase": "probe",
                            "ok_count": self.startup_ok_count,
                            "required": STARTUP_REQUIRED_OK,
                            "ok": ok,
                            "detail": detail,
                        },
                    )
                    if self.startup_ok_count >= STARTUP_REQUIRED_OK:
                        self._finish_startup("consecutive")
                    elif now >= self.startup_deadline:
                        self._finish_startup("timeout")
                    else:
                        self._next_startup_probe = now + STARTUP_PROBE_INTERVAL_S
            elif self.poll_enabled and now >= self._next_poll:
                try:
                    self._poll_once()
                except TimeoutError as exc:
                    self._emit("warning", f"状态查询超时：{exc}")
                except Exception as exc:
                    self._emit("warning", f"状态查询失败：{exc}")
                    self._handle_transport_error(exc)
                self._next_poll = time.monotonic() + self.poll_interval
