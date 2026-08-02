from __future__ import annotations

import unittest

from motor_protocol import (
    MotorControl,
    build_group_control_frame,
    build_frame,
    build_query,
    build_read_did,
    build_set_node_id_service,
    build_single_control_frame,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)
from simulator import SimulatedTransport


def read_reply(transport: SimulatedTransport, mid: int) -> bytes:
    transport.write(build_query(mid))
    first = transport.read(1)
    if not first:
        raise AssertionError("模拟器未返回 DLEN")
    tail = first + transport.read(first[0] + 1)
    return parse_reply_tail(mid, tail)


class SimulatorIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = SimulatedTransport(node_id=1)

    def tearDown(self) -> None:
        self.transport.close()

    def test_control_then_status(self) -> None:
        control = MotorControl(True, 0, 2000, 50, 25)
        self.transport.write(build_single_control_frame(1, control))
        status = parse_motion_status(read_reply(self.transport, 0x181))
        self.assertTrue(status.enabled)
        self.assertEqual(status.mode, 0)
        self.assertEqual(status.actual_speed, 50)
        self.assertEqual(status.actual_iq, 25)

    def test_group_control_applies_own_mcp_slot(self) -> None:
        control = MotorControl(True, 1, 300, 40, 20)
        self.transport.write(build_group_control_frame((1, 2), control))
        status = parse_motion_status(read_reply(self.transport, 0x181))
        self.assertTrue(status.enabled)
        self.assertEqual(status.mode, 1)
        self.assertEqual(status.actual_speed, 40)
        self.assertEqual(status.actual_iq, 20)

    def test_group_control_node_two_uses_second_slot(self) -> None:
        transport = SimulatedTransport(node_id=2)
        try:
            control = MotorControl(True, 0, 500, 25, 10)
            transport.write(build_group_control_frame((1, 2), control))
            status = parse_motion_status(read_reply(transport, 0x182))
            self.assertTrue(status.enabled)
            self.assertEqual(status.mode, 0)
            self.assertEqual(status.actual_iq, 10)
        finally:
            transport.close()

    def test_read_uid_service_sequence(self) -> None:
        request = build_frame(0x601, build_read_did(0x1018, 4))
        self.transport.write(request)
        response = parse_service_response(read_reply(self.transport, 0x581))
        self.assertTrue(response.ok)
        self.assertEqual(response.value, 0x31A2B4C5)

    def test_set_node_id_takes_effect(self) -> None:
        payload = build_set_node_id_service(2, 0x31A2B4C5)
        self.transport.write(build_frame(0x601, payload))
        response = parse_service_response(read_reply(self.transport, 0x582))
        self.assertTrue(response.ok)
        self.assertEqual(self.transport.node_id, 2)
        status = parse_motion_status(read_reply(self.transport, 0x182))
        self.assertFalse(status.enabled)

    def test_big_endian_mid_option(self) -> None:
        transport = SimulatedTransport(node_id=1, byteorder="big")
        try:
            transport.write(
                build_single_control_frame(
                    1, MotorControl(True, 1, 0, -20, 10), byteorder="big"
                )
            )
            mid = 0x181
            transport.write(build_query(mid, byteorder="big"))
            first = transport.read(1)
            tail = first + transport.read(first[0] + 1)
            status = parse_motion_status(parse_reply_tail(mid, tail, byteorder="big"))
            self.assertTrue(status.enabled)
            self.assertEqual(status.mode, 1)
            self.assertEqual(status.actual_speed, -20)
        finally:
            transport.close()


if __name__ == "__main__":
    unittest.main()
