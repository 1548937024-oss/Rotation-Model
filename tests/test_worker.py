from __future__ import annotations

import queue
import time
import unittest

from device_worker import DeviceWorker
from motor_protocol import (
    MotorControl,
    build_frame,
    build_motor_enable_service,
    build_query_frame,
    build_read_did,
    build_reply_tail,
    parse_full_frame,
)
from simulator import SimulatedTransport


class ScriptedTransport:
    """测试用串口替身：按写入次数注入预置字节流。"""

    def __init__(self, responses: list[bytes]) -> None:
        self.responses = list(responses)
        self.writes: list[bytes] = []
        self.is_open = True
        self._rx = bytearray()

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        index = len(self.writes) - 1
        if index < len(self.responses):
            self._rx.extend(self.responses[index])
        return len(data)

    def read(self, size: int = 1) -> bytes:
        count = min(int(size), len(self._rx))
        out = bytes(self._rx[:count])
        del self._rx[:count]
        return out

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        self.is_open = False


class WorkerHarness(unittest.TestCase):
    def setUp(self) -> None:
        self.worker = DeviceWorker()
        self.worker.submit("connect", "__SIMULATOR__", 460800, 1)
        self._wait_for("connected")

    def tearDown(self) -> None:
        self.worker.shutdown()

    def _wait_for(self, wanted: str, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                kind, payload = self.worker.events.get(timeout=0.05)
            except queue.Empty:
                continue
            if kind == "error":
                self.fail(payload)
            if kind == wanted:
                return payload
        self.fail(f"等待事件 {wanted} 超时")


class WorkerEndToEndTests(WorkerHarness):
    def test_control_and_status(self) -> None:
        self.worker.transport._zero_established = True
        self.worker.submit("send_control", MotorControl(True, 1, 0, 30, 15))
        # 使能后固件延迟约 20 ms 才加载当帧目标，等待后再查询。
        time.sleep(0.06)
        self.worker.submit("query_status")
        motion = self._wait_for("motion_status")
        self.assertTrue(motion.enabled)
        self.assertEqual(motion.actual_speed, 30)
        device = self._wait_for("device_status")
        self.assertEqual(device.voltage_mv, 24000)

    def test_read_max_iq_and_speed(self) -> None:
        self.worker.submit("read_did", 0x6073, 0x00, "读 Max Iq")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.value, 1400)
        self.worker.submit("read_did", 0x607F, 0x00, "读 Max Speed")
        response = self._wait_for("service")
        self.assertEqual(response.value, 18000)

    def test_clear_fault_then_status_recovers(self) -> None:
        self.worker.transport.inject_fault(3)
        self.worker.submit("query_status")
        motion = self._wait_for("motion_status")
        self.assertTrue(motion.error)
        self.assertEqual(motion.error_code, 3)
        self.worker.submit("clear_fault")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.worker.submit("query_status")
        motion = self._wait_for("motion_status")
        self.assertFalse(motion.error)

    def test_enable_service_returns_response(self) -> None:
        self.worker.submit("motor_enable", True)
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.sub_id, 0x10)

    def test_save_parameters(self) -> None:
        self.worker.submit("write_did", 0x6073, 0x01, 1000, "写堵转电流")
        self._wait_for("service")
        self.worker.submit("save_parameters")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.did, 0x1010)

    def test_reload_parameters(self) -> None:
        self.worker.submit("reload_parameters")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.did, 0x1011)

    def test_stop_services_return_ok(self) -> None:
        self.worker.submit("stop")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.sub_id, 0x13)
        self.worker.submit("quick_stop")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.sub_id, 0x14)

    def test_emergency_stop_latches_until_clear_fault(self) -> None:
        self.worker.submit("emergency_stop")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.sub_id, 0x15)

        self.worker.submit("motor_enable", True)
        refused = self._wait_for("service")
        self.assertFalse(refused.ok)
        self.assertEqual(refused.return_code, 0xE5)

        self.worker.submit("clear_fault")
        self.assertTrue(self._wait_for("service").ok)
        self.worker.submit("motor_enable", True)
        self.assertTrue(self._wait_for("service").ok)

    def test_read_safety_emits_values(self) -> None:
        self.worker.submit("read_safety")
        values = self._wait_for("safety", timeout=5.0)
        self.assertEqual(values[(0x2021, 0x03)], 0x00021003)
        self.assertEqual(values[(0x2021, 0x06)], 0x00010400)
        self.assertIn((0x2021, 0x02), values)

    def test_read_telemetry(self) -> None:
        self.worker.transport._zero_established = True
        self.worker.submit("send_control", MotorControl(True, 1, 0, 50, 100))
        self.worker.transport._apply_after = time.monotonic() - 0.001
        self.worker.transport._advance()
        self.worker.submit("read_telemetry")
        values = self._wait_for("telemetry", timeout=3.0)
        self.assertIn((0x2010, 0x04), values)
        self.assertIn((0x2015, 0x01), values)

    def test_emergency_disable(self) -> None:
        self.worker.submit("emergency_disable")
        self._wait_for("disabled")
        self.assertFalse(self.worker.latest_control.enable)

    def test_cyclic_repeats_latest_control(self) -> None:
        self.worker.submit("configure_cyclic", MotorControl(False, 0, 0, 0, 20), True, 0.01)
        deadline = time.monotonic() + 1.0
        writes = 0
        while time.monotonic() < deadline and writes < 3:
            time.sleep(0.02)
            writes += 1
        self.assertTrue(self.worker.cyclic_enabled)

    def test_raw_script_query(self) -> None:
        steps = (
            ("query", 0x181, False),
            ("delay", 0.005),
            ("query", 0x281, True),
        )
        self.worker.submit("raw_script", steps)
        self._wait_for("raw_reply")
        payload = self._wait_for("notice")
        self.assertIn("脚本执行完成", payload)


class WorkerFramingTests(unittest.TestCase):
    def test_queries_use_full_zero_length_frame_by_default(self) -> None:
        class RecordingTransport:
            def __init__(self) -> None:
                self.writes: list[bytes] = []
                self.is_open = True

            def write(self, data: bytes) -> int:
                self.writes.append(bytes(data))
                return len(data)

            def read(self, size: int = 1) -> bytes:
                return b""

            def reset_input_buffer(self) -> None:
                return None

            def close(self) -> None:
                self.is_open = False

        worker = DeviceWorker()
        try:
            transport = RecordingTransport()
            worker.transport = transport
            with self.assertRaises(TimeoutError):
                worker._query(0x181, "test")
            self.assertEqual(transport.writes[0], build_query_frame(0x181))
        finally:
            worker.shutdown()

    def test_short_query_option(self) -> None:
        class RecordingTransport:
            def __init__(self) -> None:
                self.writes: list[bytes] = []
                self.is_open = True

            def write(self, data: bytes) -> int:
                self.writes.append(bytes(data))
                return len(data)

            def read(self, size: int = 1) -> bytes:
                return b""

            def reset_input_buffer(self) -> None:
                return None

            def close(self) -> None:
                self.is_open = False

        worker = DeviceWorker()
        try:
            transport = RecordingTransport()
            worker.transport = transport
            with self.assertRaises(TimeoutError):
                worker._query(0x181, "test", short=True)
            self.assertEqual(transport.writes[0], bytes.fromhex("AA 81 01"))
        finally:
            worker.shutdown()

    def test_query_discards_noise_and_resyncs(self) -> None:
        payload = bytes.fromhex("80 0A 00 00 14")
        noise = bytes.fromhex("84 00 55")
        worker = DeviceWorker()
        events: list[tuple] = []
        try:
            worker.transport = ScriptedTransport([noise + build_reply_tail(0x181, payload)])
            worker.node_id = 1
            worker._emit = lambda kind, value: events.append((kind, value))  # type: ignore[assignment]
            data = worker._query(0x181, "test")
            self.assertEqual(data, payload)
            resyncs = [value for kind, value in events if kind == "resync"]
            self.assertEqual(resyncs, [(0x181, 3, 1)])
        finally:
            worker.shutdown()

    def test_query_skips_wrong_dlen_frame_then_matches(self) -> None:
        status2_tail = build_reply_tail(0x281, bytes.fromhex("C0 5D 1F"))
        status1_payload = bytes.fromhex("80 0A 00 00 14")
        status1_tail = build_reply_tail(0x181, status1_payload)
        worker = DeviceWorker()
        try:
            worker.transport = ScriptedTransport([status2_tail + status1_tail])
            worker.node_id = 1
            data = worker._query(0x181, "test")
            self.assertEqual(data, status1_payload)
        finally:
            worker.shutdown()

    def test_query_retries_when_first_reply_missing(self) -> None:
        payload = bytes.fromhex("80 00 00 00 00")
        worker = DeviceWorker()
        try:
            transport = ScriptedTransport([b"", build_reply_tail(0x181, payload)])
            worker.transport = transport
            worker.node_id = 1
            data = worker._query(0x181, "test")
            self.assertEqual(data, payload)
            self.assertEqual(len(transport.writes), 2)
        finally:
            worker.shutdown()

    def test_query_gives_up_after_three_attempts(self) -> None:
        worker = DeviceWorker()
        try:
            transport = ScriptedTransport([b"", b"", b""])
            worker.transport = transport
            worker.node_id = 1
            with self.assertRaises(TimeoutError):
                worker._query(0x181, "test")
            self.assertEqual(len(transport.writes), 3)
        finally:
            worker.shutdown()

    def test_startup_probe_needs_two_consecutive_cycles(self) -> None:
        worker = DeviceWorker()
        try:
            worker.transport = SimulatedTransport(node_id=1)
            worker.node_id = 1
            worker.startup_pending = True
            worker.startup_ok_count = 0
            worker.startup_deadline = time.monotonic() + 3.0
            self.assertTrue(worker._startup_probe()[0])
            self.assertEqual(worker.startup_ok_count, 1)
            self.assertTrue(worker.startup_pending)
            self.assertTrue(worker._startup_probe()[0])
            self.assertEqual(worker.startup_ok_count, 2)
            worker._finish_startup("consecutive")
            self.assertFalse(worker.startup_pending)
            events = []
            while True:
                try:
                    events.append(worker.events.get_nowait())
                except queue.Empty:
                    break
            done = [
                payload
                for kind, payload in events
                if kind == "startup" and isinstance(payload, dict) and payload.get("phase") == "done"
            ]
            self.assertEqual(len(done), 1)
            self.assertEqual(done[0]["reason"], "consecutive")
        finally:
            worker.shutdown()

    def test_service_waits_for_delayed_pushed_reply(self) -> None:
        """固件主动下发应答；即使延迟超过旧 6 ms 窗口也必须成功，且不发 0x581 查询。"""

        class DelayedPushTransport:
            def __init__(self, delay_s: float, tail: bytes) -> None:
                self.delay_s = delay_s
                self.tail = tail
                self.writes: list[bytes] = []
                self.is_open = True
                self._buffer = bytearray()
                self._release_at: float | None = None

            def write(self, data: bytes) -> int:
                self.writes.append(bytes(data))
                mid = int.from_bytes(bytes(data[1:3]), "little") & 0x7FF
                if mid == 0x601:
                    self._release_at = time.monotonic() + self.delay_s
                return len(data)

            def read(self, size: int = 1) -> bytes:
                if self._release_at is not None and time.monotonic() >= self._release_at:
                    self._buffer.extend(self.tail)
                    self.tail = b""
                    self._release_at = None
                count = min(int(size), len(self._buffer))
                out = bytes(self._buffer[:count])
                del self._buffer[:count]
                return out

            def reset_input_buffer(self) -> None:
                self._buffer.clear()

            def close(self) -> None:
                self.is_open = False

        tail = build_reply_tail(0x581, bytes.fromhex("F1 10 01 00 00 00 00 00"))
        worker = DeviceWorker()
        try:
            transport = DelayedPushTransport(0.03, tail)
            worker.transport = transport
            worker.node_id = 1
            response = worker._service(build_motor_enable_service(True))
            self.assertTrue(response.ok)
            self.assertEqual(response.sub_id, 0x10)
            self.assertEqual(len(transport.writes), 1)
            self.assertEqual(
                int.from_bytes(transport.writes[0][1:3], "little") & 0x7FF, 0x601
            )
        finally:
            worker.shutdown()

    def test_transport_error_marks_disconnected(self) -> None:
        class BrokenTransport:
            def __init__(self) -> None:
                self.is_open = True

            def write(self, data: bytes) -> int:
                raise OSError("device removed")

            def read(self, size: int = 1) -> bytes:
                return b""

            def reset_input_buffer(self) -> None:
                return None

            def close(self) -> None:
                self.is_open = False

        worker = DeviceWorker()
        try:
            worker.transport = BrokenTransport()
            worker.submit("send_control", MotorControl(False, 0, 0, 0, 20))
            deadline = time.monotonic() + 1.5
            disconnected = False
            while time.monotonic() < deadline:
                try:
                    kind, _ = worker.events.get(timeout=0.05)
                except queue.Empty:
                    continue
                if kind == "disconnected":
                    disconnected = True
                    break
            self.assertTrue(disconnected)
            self.assertIsNone(worker.transport)
        finally:
            worker.shutdown()


if __name__ == "__main__":
    unittest.main()
