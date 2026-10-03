from __future__ import annotations

import time
import unittest

from motor_protocol import (
    MAX_IQ_MA,
    MAX_SPEED_RPM,
    POSITION_SOFT_MAX_COUNTS,
    MotorControl,
    build_clear_fault_service,
    build_emergency_stop_service,
    build_frame,
    build_motor_enable_service,
    build_normal_stop_service,
    build_query_frame,
    build_quick_stop_service,
    build_read_did,
    build_reply_tail,
    build_reload_parameters,
    build_save_parameters,
    build_single_control_frame,
    build_write_did,
    decode_did_value,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)
from simulator import SimulatedTransport


def query(transport: SimulatedTransport, mid: int) -> bytes:
    transport.write(build_query_frame(mid))
    first = transport.read(1)
    if not first:
        raise AssertionError("模拟器未返回 DLEN")
    tail = first + transport.read(first[0] + 1)
    return parse_reply_tail(mid, tail)


def service(transport: SimulatedTransport, payload: bytes):
    """发送服务请求后直接接收固件主动下发的 0x581 应答尾。"""

    transport.write(build_frame(0x601, payload))
    first = transport.read(1)
    if not first:
        raise AssertionError("模拟器未主动下发服务应答")
    tail = first + transport.read(first[0] + 1)
    return parse_service_response(parse_reply_tail(0x581, tail))


class SimulatorIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = SimulatedTransport()

    def tearDown(self) -> None:
        self.transport.close()

    def _finish_alignment(self) -> None:
        self.transport._align_until = time.monotonic() - 0.001
        self.transport._apply_after = time.monotonic() - 0.001
        self.transport._advance()

    def test_first_enable_starts_alignment_then_runs(self) -> None:
        response = service(self.transport, build_motor_enable_service(True))
        self.assertTrue(response.ok)
        self.assertIsNotNone(self.transport._align_until)
        status = parse_motion_status(query(self.transport, 0x181))
        self.assertTrue(status.enabled)

        self._finish_alignment()
        self.transport.write(build_single_control_frame(1, MotorControl(True, 1, 0, 40, 20)))
        # 运行中切换模式会先停机再启动，协议约定约 20 ms 后才加载目标。
        self.transport._apply_after = time.monotonic() - 0.001
        status = parse_motion_status(query(self.transport, 0x181))
        self.assertEqual(status.mode, 1)
        self.assertEqual(status.actual_speed, 40)

    def test_control_then_status(self) -> None:
        self.transport._zero_established = True
        self.transport.write(build_single_control_frame(1, MotorControl(True, 1, 0, 25, 10)))
        self.transport._apply_after = time.monotonic() - 0.001
        status = parse_motion_status(query(self.transport, 0x181))
        self.assertTrue(status.enabled)
        self.assertEqual(status.mode, 1)
        self.assertEqual(status.actual_speed, 25)
        self.assertEqual(status.actual_iq, 10)

    def test_read_only_max_values(self) -> None:
        self.assertEqual(service(self.transport, build_read_did(0x6073, 0x00)).value, MAX_IQ_MA)
        self.assertEqual(service(self.transport, build_read_did(0x607F, 0x00)).value, MAX_SPEED_RPM)
        self.assertEqual(service(self.transport, build_read_did(0x1018, 0x04)).value, 0x31A2B4C5)
        self.assertEqual(service(self.transport, build_read_did(0x3001, 0x01)).value, 460800)

    def test_read_0x6040_is_abort(self) -> None:
        response = service(self.transport, build_read_did(0x6040, 0x00))
        self.assertFalse(response.ok)
        self.assertEqual(response.return_code, 0x06010001)

    def test_write_read_only_did_is_abort(self) -> None:
        response = service(self.transport, build_write_did(0x6073, 0x00, 1000))
        self.assertFalse(response.ok)
        self.assertEqual(response.return_code, 0x06010002)

    def test_write_out_of_range_is_abort(self) -> None:
        response = service(self.transport, build_write_did(0x6073, 0x01, 4000))
        self.assertFalse(response.ok)
        self.assertEqual(response.return_code, 0x06090030)

    def test_write_then_read_parameter(self) -> None:
        self.assertTrue(service(self.transport, build_write_did(0x6073, 0x01, 1000)).ok)
        self.assertEqual(service(self.transport, build_read_did(0x6073, 0x01)).value, 1000)
        self.assertTrue(service(self.transport, build_write_did(0x202D, 0x01, 90)).ok)
        self.assertEqual(service(self.transport, build_read_did(0x202D, 0x01)).value, 90)

    def test_save_and_reload(self) -> None:
        service(self.transport, build_write_did(0x2016, 0x02, 80))
        self.assertTrue(service(self.transport, build_save_parameters()).ok)
        service(self.transport, build_write_did(0x2016, 0x02, 60))
        self.assertEqual(service(self.transport, build_read_did(0x2016, 0x02)).value, 60)
        self.assertTrue(service(self.transport, build_reload_parameters()).ok)
        self.assertEqual(service(self.transport, build_read_did(0x2016, 0x02)).value, 80)

    def test_clear_fault(self) -> None:
        self.transport.inject_fault(3)
        status = parse_motion_status(query(self.transport, 0x181))
        self.assertTrue(status.error)
        self.assertEqual(status.error_code, 3)
        self.assertTrue(service(self.transport, build_clear_fault_service()).ok)
        status = parse_motion_status(query(self.transport, 0x181))
        self.assertFalse(status.error)

    def test_set_zero_and_node_id_are_abort(self) -> None:
        zero = service(self.transport, bytes((0xF1, 0xA2, 0, 0, 0, 0, 0, 0)))
        self.assertFalse(zero.ok)
        node = service(self.transport, bytes((0xF2, 0, 0, 2, 0, 0, 0, 0)))
        self.assertFalse(node.ok)
        self.assertEqual(self.transport.node_id, 1)

    def test_run_mode_refused_while_enabled(self) -> None:
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        response = service(self.transport, bytes((0xF1, 0x11, 1, 0, 0, 0, 0, 0)))
        self.assertFalse(response.ok)
        self.assertEqual(response.return_code, 0xE5)

    def test_telemetry_dids(self) -> None:
        self.transport._zero_established = True
        self.transport.write(build_single_control_frame(1, MotorControl(True, 1, 0, 50, 100)))
        self.transport._apply_after = time.monotonic() - 0.001
        self.transport._advance()
        peak = service(self.transport, build_read_did(0x2010, 0x04)).value
        self.assertGreater(peak, 0)
        self.assertLessEqual(peak, MAX_IQ_MA)
        total = service(self.transport, build_read_did(0x2015, 0x01)).value
        self.assertIsInstance(total, int)

    def test_short_and_full_queries_both_work(self) -> None:
        short = parse_reply_tail(0x181, self._short_query(0x181))
        self.assertEqual(len(short), 5)

    def test_normal_stop_decelerates_then_disables(self) -> None:
        self.transport._zero_established = True
        self.assertTrue(service(self.transport, build_motor_enable_service(True)).ok)
        self.transport._apply_after = time.monotonic() - 0.001
        self.transport._advance()
        self.transport.write(build_single_control_frame(1, MotorControl(True, 1, 0, 60, 20)))
        self.transport._apply_after = time.monotonic() - 0.001
        self.transport._advance()
        self.assertTrue(self.transport.enabled)

        self.assertTrue(service(self.transport, build_normal_stop_service()).ok)
        self.assertEqual(self.transport._stop_kind, 1)
        self.assertTrue(self.transport._stop_active)
        for _ in range(12):
            self.transport._advance()
        self.assertFalse(self.transport.enabled)

    def test_quick_stop_disables_immediately(self) -> None:
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        self.assertTrue(self.transport.enabled)
        self.assertTrue(service(self.transport, build_quick_stop_service()).ok)
        self.assertFalse(self.transport.enabled)
        self.assertEqual(self.transport._stop_kind, 2)

    def test_emergency_stop_latches_until_clear_fault(self) -> None:
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        self.assertTrue(service(self.transport, build_emergency_stop_service()).ok)
        self.assertTrue(self.transport._safety_latched)
        flags = service(self.transport, build_read_did(0x2021, 0x02)).value
        self.assertTrue(flags & 0x0040)

        refused = service(self.transport, build_motor_enable_service(True))
        self.assertFalse(refused.ok)
        self.assertEqual(refused.return_code, 0xE5)

        self.assertTrue(service(self.transport, build_clear_fault_service()).ok)
        self.assertFalse(self.transport._safety_latched)
        self.assertTrue(service(self.transport, build_motor_enable_service(True)).ok)

    def test_control_frame_timeout_stops_and_disables(self) -> None:
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        self.transport._apply_after = time.monotonic() - 0.001
        self.transport._advance()
        self.assertTrue(self.transport.enabled)

        # 模拟超过 200 ms 没有收到任何 Control1 / 0xF1 帧
        self.transport._last_cmd_time = time.monotonic() - 0.5
        for _ in range(12):
            self.transport._advance()
        self.assertFalse(self.transport.enabled)
        self.assertTrue(self.transport._comm_lost)
        flags = service(self.transport, build_read_did(0x2021, 0x02)).value
        self.assertTrue(flags & 0x0010)

    def test_control_frames_keep_watchdog_alive(self) -> None:
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        self.transport._last_cmd_time = time.monotonic() - 0.5
        self.transport.write(build_single_control_frame(1, MotorControl(True, 1, 0, 10, 20)))
        self.transport._advance()
        self.assertTrue(self.transport.enabled)
        self.assertFalse(self.transport._comm_lost)

    def test_safety_config_read_write(self) -> None:
        self.assertEqual(service(self.transport, build_read_did(0x2020, 0x03)).value, 200)
        self.assertTrue(service(self.transport, build_write_did(0x2020, 0x03, 300)).ok)
        self.assertEqual(service(self.transport, build_read_did(0x2020, 0x03)).value, 300)
        out_of_range = service(self.transport, build_write_did(0x2020, 0x03, 10))
        self.assertFalse(out_of_range.ok)
        self.assertEqual(out_of_range.return_code, 0x06090030)

    def test_soft_limit_write_requires_stopped_and_ordered(self) -> None:
        raw_min = (-1000) & 0xFFFFFFFF
        self.assertTrue(service(self.transport, build_write_did(0x2020, 0x01, raw_min)).ok)
        read_back = service(self.transport, build_read_did(0x2020, 0x01)).value
        self.assertEqual(decode_did_value(0x2020, 0x01, read_back), -1000)

        # 软上限必须大于软下限
        bad_max = service(self.transport, build_write_did(0x2020, 0x02, (-2000) & 0xFFFFFFFF))
        self.assertFalse(bad_max.ok)
        self.assertEqual(bad_max.return_code, 0x06090030)
        self.assertEqual(
            service(self.transport, build_read_did(0x2020, 0x02)).value,
            POSITION_SOFT_MAX_COUNTS,
        )

        # 运行中拒绝写安全配置
        self.transport._zero_established = True
        service(self.transport, build_motor_enable_service(True))
        refused = service(self.transport, build_write_did(0x2020, 0x03, 250))
        self.assertFalse(refused.ok)
        self.assertEqual(refused.return_code, 0x06010002)

    def test_safety_status_did(self) -> None:
        self.assertEqual(service(self.transport, build_read_did(0x2021, 0x03)).value, 0x00021003)
        self.assertEqual(service(self.transport, build_read_did(0x2021, 0x06)).value, 0x00010400)
        self.assertEqual(service(self.transport, build_read_did(0x2021, 0x01)).value, 0)
        refused = service(self.transport, build_write_did(0x2021, 0x01, 1))
        self.assertFalse(refused.ok)
        self.assertEqual(refused.return_code, 0x06010002)

    def _short_query(self, mid: int) -> bytes:
        self.transport.write(bytes((0xAA, mid & 0xFF, (mid >> 8) & 0xFF)))
        first = self.transport.read(1)
        return first + self.transport.read(first[0] + 1)


if __name__ == "__main__":
    unittest.main()
