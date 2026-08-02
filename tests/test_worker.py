from __future__ import annotations

import queue
import time
import unittest

from device_worker import DeviceWorker
from motor_protocol import MotorControl, build_read_did, build_set_node_id_service


class WorkerEndToEndTests(unittest.TestCase):
    def setUp(self) -> None:
        self.worker = DeviceWorker()
        self.worker.submit("connect", "__SIMULATOR__", 921600, "little", 1)
        self._wait_for("connected")

    def tearDown(self) -> None:
        self.worker.shutdown()

    def _wait_for(self, wanted: str, timeout: float = 1.0):
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

    def test_control_status_and_service(self) -> None:
        self.worker.submit("send_control", MotorControl(True, 0, 1200, 30, 15))
        self.worker.submit("query_status")
        motion = self._wait_for("motion_status")
        self.assertTrue(motion.enabled)
        self.assertEqual(motion.actual_speed, 30)
        self.worker.submit("service", build_read_did(0x1018, 4), "读取 UID")
        response = self._wait_for("service")
        self.assertTrue(response.ok)
        self.assertEqual(response.value, 0x31A2B4C5)

    def test_set_node_id_command(self) -> None:
        payload = build_set_node_id_service(3, 0x31A2B4C5)
        self.worker.submit("set_node_by_uid", payload, 3)
        self.assertEqual(self._wait_for("node_changed"), 3)
        self.worker.submit("query_status")
        self.assertFalse(self._wait_for("motion_status").enabled)

    def test_multi_node_control_and_poll(self) -> None:
        self.worker.submit(
            "send_control", MotorControl(True, 0, 1200, 30, 15), (1, 2)
        )
        self.worker.submit("query_status")
        node_id, motion = self._wait_for("multi_motion")
        self.assertEqual(node_id, 1)
        self.assertTrue(motion.enabled)
        self.assertEqual(motion.actual_speed, 30)


if __name__ == "__main__":
    unittest.main()
