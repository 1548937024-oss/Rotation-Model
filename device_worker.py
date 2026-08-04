"""Single-threaded serial worker for the motor module host application."""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, Callable

from motor_protocol import (
    MAX_DATA_LENGTH,
    MotorControl,
    ProtocolError,
    build_frame,
    build_group_control_frame,
    build_multi_control_frame,
    build_query,
    build_single_control_frame,
    format_hex,
    parse_device_status,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)
from simulator import SimulatedTransport


@dataclass
class WorkerCommand:
    name: str
    args: tuple[Any, ...] = ()


class DeviceWorker:
    """Owns the transport so all UART traffic is serialized in one thread."""

    def __init__(self) -> None:
        self.commands: queue.Queue[WorkerCommand] = queue.Queue()
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="motor-serial", daemon=True)
        self.transport: Any | None = None
        self.byteorder = "little"
        self.node_id = 1
        self.control_nodes: tuple[int, ...] = (1,)
        self.poll_enabled = False
        self.poll_interval = 0.1
        self._next_poll = float("inf")
        self._poll_count = 0
        self.cyclic_enabled = False
        self.cyclic_interval = 0.02
        self._next_control = float("inf")
        self.latest_control = MotorControl(False, 0, 0, 0, 0)
        self.latest_controls: dict[int, MotorControl] = {}
        self._thread.start()

    def submit(self, name: str, *args: Any) -> None:
        self.commands.put(WorkerCommand(name, args))

    def shutdown(self) -> None:
        self.submit("shutdown")
        self._thread.join(timeout=1.5)

    def _emit(self, kind: str, payload: Any) -> None:
        self.events.put((kind, payload))

    def _log(self, direction: str, data: bytes, note: str = "") -> None:
        timestamp = time.strftime("%H:%M:%S")
        suffix = f"  {note}" if note else ""
        self._emit("log", f"{timestamp} {direction:<2}  {format_hex(data)}{suffix}")

    def _read_exact(self, size: int) -> bytes:
        result = bytearray()
        deadline = time.monotonic() + 0.05
        while len(result) < size and time.monotonic() < deadline:
            chunk = self.transport.read(size - len(result))
            if chunk:
                result.extend(chunk)
            else:
                time.sleep(0.001)
        if len(result) != size:
            raise TimeoutError(f"等待 {size} 字节，实际收到 {len(result)} 字节")
        return bytes(result)

    def _write(self, data: bytes, note: str = "") -> None:
        if self.transport is None:
            raise RuntimeError("串口未连接")
        written = self.transport.write(data)
        if written != len(data):
            raise OSError(f"串口仅写入 {written}/{len(data)} 字节")
        self._log("TX", data, note)

    def _query(self, mid: int, note: str = "") -> bytes:
        query = build_query(mid, self.byteorder)
        self._write(query, note or f"查询 MID 0x{mid:03X}")
        try:
            first = self._read_exact(1)
            dlen = first[0]
            if not 1 <= dlen <= MAX_DATA_LENGTH:
                raise ProtocolError(f"从机返回非法 DLEN：{dlen}")
            rest = self._read_exact(dlen + 1)
            tail = first + rest
            self._log("RX", tail, f"MID 0x{mid:03X} 的响应尾")
            return parse_reply_tail(mid, tail, self.byteorder)
        except Exception:
            # A partial response cannot be resynchronized because slave reply
            # tails have no Header/MID. Drop stale bytes before the next query.
            try:
                self.transport.reset_input_buffer()
            except Exception:
                pass
            raise

    def _send_control(
        self,
        control: MotorControl,
        note: str = "控制报文",
        nodes: tuple[int, ...] | None = None,
    ) -> None:
        selected = self.control_nodes if nodes is None else tuple(int(n) for n in nodes)
        if len(selected) == 1:
            frame = build_single_control_frame(selected[0], control, self.byteorder)
            self._write(frame, note)
        else:
            frame = build_group_control_frame(selected, control, self.byteorder)
            self._write(frame, f"{note}（多控 {len(selected)} 节点）")

    def _send_controls(
        self, controls: dict[int, MotorControl], note: str = "控制报文"
    ) -> None:
        if len(controls) == 1:
            node, control = next(iter(controls.items()))
            self._write(
                build_single_control_frame(int(node), control, self.byteorder), note
            )
        else:
            frame = build_multi_control_frame(controls, self.byteorder)
            self._write(frame, f"{note}（多控 {len(controls)} 节点）")

    def _flush_due_controls(self, now: float) -> None:
        """Emit one control per elapsed cycle so polling cannot starve it."""

        if not self.cyclic_enabled:
            return
        burst = 0
        while self.cyclic_enabled and now >= self._next_control and burst < 10:
            if self.latest_controls:
                self._send_controls(self.latest_controls, "周期多控")
            else:
                self._send_control(self.latest_control, "周期单控")
            self._next_control += self.cyclic_interval
            burst += 1
        if now >= self._next_control:
            self._next_control = now + self.cyclic_interval

    def _service(self, payload: bytes, note: str = "服务请求") -> Any:
        request_mid = 0x600 + self.node_id
        response_mid = 0x580 + self.node_id
        self._write(build_frame(request_mid, payload, self.byteorder), note)
        time.sleep(0.006)
        data = self._query(response_mid, "查询服务响应")
        response = parse_service_response(data)
        self._emit("service", response)
        return response

    def _poll_once(self) -> None:
        self._poll_count += 1
        for node in self.control_nodes:
            try:
                motion_data = self._query(0x180 + node, f"查询节点 {node} 运动状态")
                motion = parse_motion_status(motion_data)
                self._emit("multi_motion", (node, motion))
                if node == self.node_id:
                    self._emit("motion_status", motion)
                if self._poll_count % 5 == 0:
                    device_data = self._query(0x280 + node, f"查询节点 {node} 设备状态")
                    device = parse_device_status(device_data)
                    self._emit("multi_device", (node, device))
                    if node == self.node_id:
                        self._emit("device_status", device)
            except Exception as exc:
                if isinstance(exc, OSError) and not isinstance(exc, TimeoutError):
                    raise
                self._emit("warning", f"节点 {node} 查询失败：{exc}")

    def _connect(self, port: str, baudrate: int, byteorder: str, node_id: int) -> None:
        if self.transport is not None:
            self._disconnect(False)
        self.byteorder = byteorder
        self.node_id = node_id
        self.control_nodes = (node_id,)
        self.latest_controls.clear()
        if port == "__SIMULATOR__":
            self.transport = SimulatedTransport(node_id=node_id, byteorder=byteorder)
            description = "模拟设备"
        else:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("缺少 pyserial，请先运行 setup_and_run.bat") from exc
            self.transport = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.03,
                write_timeout=0.2,
            )
            self.transport.reset_input_buffer()
            description = f"{port} @ {baudrate}"
        self._emit("connected", description)
        self._log("--", b"", f"已连接 {description}，MID {byteorder}")

    def _disconnect(self, disable_first: bool) -> None:
        if self.transport is None:
            return
        self.poll_enabled = False
        self.cyclic_enabled = False
        if disable_first:
            try:
                safe = MotorControl(False, self.latest_control.mode, self.latest_control.target_position, 0, 0)
                if self.latest_controls:
                    self._send_controls(
                        {node: safe for node in self.control_nodes}, "断开前下使能"
                    )
                else:
                    self._send_control(safe, "断开前下使能")
            except Exception as exc:
                self._emit("warning", f"断开前下使能发送失败：{exc}")
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

    def _handle(self, command: WorkerCommand) -> None:
        name, args = command.name, command.args
        if name == "connect":
            self._connect(*args)
        elif name == "disconnect":
            self._disconnect(*args)
        elif name == "set_node":
            self.node_id = int(args[0])
            self.control_nodes = (self.node_id,)
            self.latest_controls.clear()
            self._emit("notice", f"当前通信 Node ID 已切换为 {self.node_id}")
        elif name == "send_control":
            self.latest_control = args[0]
            self.latest_controls.clear()
            if len(args) > 1 and args[1]:
                self.control_nodes = tuple(int(n) for n in args[1])
            self._send_control(self.latest_control)
        elif name == "send_controls":
            controls = dict(args[0])
            self.latest_controls = controls
            self.control_nodes = tuple(sorted(controls))
            if controls:
                self.latest_control = controls[self.control_nodes[0]]
            self._send_controls(self.latest_controls)
        elif name == "emergency_disable":
            safe = MotorControl(False, self.latest_control.mode, self.latest_control.target_position, 0, 0)
            self.latest_control = safe
            self.latest_controls = {node: safe for node in self.control_nodes}
            self.cyclic_enabled = False
            self._send_controls(self.latest_controls, "立即下使能")
            time.sleep(0.003)
            self._send_controls(self.latest_controls, "立即下使能（重复）")
            self._emit("disabled", None)
        elif name == "configure_poll":
            self.poll_enabled = bool(args[0])
            self.poll_interval = max(0.02, float(args[1]))
            if len(args) > 2 and args[2]:
                self.control_nodes = tuple(int(n) for n in args[2])
            self._next_poll = time.monotonic()
        elif name == "query_status":
            self._poll_once()
        elif name == "configure_cyclic":
            was_enabled = self.cyclic_enabled
            self.latest_control = args[0]
            self.latest_controls.clear()
            self.cyclic_enabled = bool(args[1])
            self.cyclic_interval = max(0.001, float(args[2]))
            if len(args) > 3 and args[3]:
                self.control_nodes = tuple(int(n) for n in args[3])
            if not was_enabled and self.cyclic_enabled:
                self._next_control = time.monotonic()
        elif name == "configure_cyclic_group":
            was_enabled = self.cyclic_enabled
            controls = dict(args[0])
            self.latest_controls = controls
            if controls:
                self.control_nodes = tuple(sorted(controls))
                self.latest_control = controls[self.control_nodes[0]]
            self.cyclic_enabled = bool(args[1])
            self.cyclic_interval = max(0.001, float(args[2]))
            if not was_enabled and self.cyclic_enabled:
                self._next_control = time.monotonic()
        elif name == "service":
            self._service(args[0], args[1] if len(args) > 1 else "服务请求")
        elif name == "set_node_by_uid":
            payload, new_node = args
            old_node = self.node_id
            self._write(
                build_frame(0x600 + old_node, payload, self.byteorder),
                f"设置 Node ID {old_node} -> {new_node}",
            )
            time.sleep(0.01)
            response = None
            last_error: Exception | None = None
            # The document says the new ID takes effect immediately, but does
            # not explicitly state which response MID is used. Try the new ID
            # first, then the old ID for firmware compatibility.
            for candidate in (int(new_node), old_node):
                try:
                    data = self._query(0x580 + candidate, "查询设置 Node ID 响应")
                    response = parse_service_response(data)
                    break
                except (TimeoutError, ProtocolError) as exc:
                    last_error = exc
            if response is None:
                raise last_error or TimeoutError("未收到设置 Node ID 响应")
            self._emit("service", response)
            if response.ok:
                self.node_id = int(new_node)
                self.control_nodes = (self.node_id,)
                self.latest_controls.clear()
                self._emit("node_changed", self.node_id)
        elif name == "change_baud":
            payload, new_baud = args
            request_mid = 0x600 + self.node_id
            response_mid = 0x580 + self.node_id
            self._write(build_frame(request_mid, payload, self.byteorder), "写入新波特率")
            time.sleep(0.025)
            self.transport.baudrate = int(new_baud)
            self._emit("baud_changed", int(new_baud))
            try:
                response = parse_service_response(self._query(response_mid, "以新波特率查询响应"))
                self._emit("service", response)
            except TimeoutError:
                self._emit(
                    "warning",
                    "设备未返回波特率写入响应；上位机已切换至新波特率，请立即查询状态验证通信。",
                )
        elif name == "raw_frame":
            mid, data = args
            self._write(build_frame(mid, data, self.byteorder), f"原始完整帧 MID 0x{mid:03X}")
        elif name == "raw_query":
            mid = int(args[0])
            data = self._query(mid, f"原始查询 MID 0x{mid:03X}")
            self._emit("raw_reply", (mid, data))
        elif name == "shutdown":
            self._disconnect(False)
            self._stop.set()
        else:
            raise RuntimeError(f"未知工作线程命令：{name}")

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            deadlines = []
            if self.transport is not None and self.poll_enabled:
                deadlines.append(self._next_poll)
            if self.transport is not None and self.cyclic_enabled:
                deadlines.append(self._next_control)
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
            if self.cyclic_enabled:
                try:
                    self._flush_due_controls(now)
                except Exception as exc:
                    self.cyclic_enabled = False
                    self._emit("error", f"周期控制停止：{exc}")
                    self._handle_transport_error(exc)
            if self.poll_enabled and now >= self._next_poll:
                try:
                    self._poll_once()
                except Exception as exc:
                    self._emit("warning", f"状态查询失败：{exc}")
                    self._handle_transport_error(exc)
                self._next_poll = time.monotonic() + self.poll_interval
