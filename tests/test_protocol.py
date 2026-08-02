from __future__ import annotations

import unittest

from motor_protocol import (
    MotorControl,
    ProtocolError,
    build_group_control_frame,
    build_frame,
    build_query,
    build_read_did,
    build_reply_tail,
    build_set_node_id_service,
    build_single_control_frame,
    build_write_did,
    crc8,
    parse_device_status,
    parse_full_frame,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)


class CrcAndFrameTests(unittest.TestCase):
    def test_crc_fixed_vector(self) -> None:
        # Fixed vector calculated for CRC-8/poly 0x07/init 0/refin false/xorout 0.
        self.assertEqual(crc8(bytes.fromhex("AA 01 02 05 80 00 00 00 00")), 0xBA)

    def test_query_is_header_plus_little_endian_mid(self) -> None:
        self.assertEqual(build_query(0x201), bytes.fromhex("AA 01 02"))
        self.assertEqual(build_query(0x201, "big"), bytes.fromhex("AA 02 01"))

    def test_known_control_frame(self) -> None:
        control = MotorControl(True, 0, 0x1234, -10, 20)
        self.assertEqual(
            build_single_control_frame(1, control),
            bytes.fromhex("AA 01 02 05 80 34 12 F6 14 39"),
        )

    def test_group_control_frame_layout(self) -> None:
        control = MotorControl(True, 1, 100, 30, 15)
        frame = build_group_control_frame((1, 3), control)
        mid, payload = parse_full_frame(frame)
        self.assertEqual(mid, 0x200)
        self.assertEqual(len(payload), 15)
        self.assertEqual(payload[0:5], control.pack())
        self.assertEqual(payload[5:10], MotorControl(False, 0, 0, 0, 0).pack())
        self.assertEqual(payload[10:15], control.pack())

    def test_group_control_frame_validation(self) -> None:
        control = MotorControl(False, 0, 0, 0, 0)
        with self.assertRaises(ProtocolError):
            build_group_control_frame((), control)
        with self.assertRaises(ProtocolError):
            build_group_control_frame((0,), control)
        with self.assertRaises(ProtocolError):
            build_group_control_frame((9,), control)
        with self.assertRaises(ProtocolError):
            build_group_control_frame(tuple(range(1, 10)), control)

    def test_full_frame_round_trip(self) -> None:
        payload = build_read_did(0x1018, 4)
        frame = build_frame(0x601, payload)
        mid, decoded = parse_full_frame(frame)
        self.assertEqual(mid, 0x601)
        self.assertEqual(decoded, payload)

    def test_crc_corruption_is_rejected(self) -> None:
        frame = bytearray(build_frame(0x201, bytes.fromhex("80 00 00 00 00")))
        frame[-1] ^= 1
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            parse_full_frame(bytes(frame))

    def test_reply_tail_uses_queried_header_and_mid_for_crc(self) -> None:
        data = bytes.fromhex("80 34 12 F6 14")
        tail = build_reply_tail(0x181, data)
        self.assertEqual(parse_reply_tail(0x181, tail), data)
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            parse_reply_tail(0x182, tail)


class SignalCodecTests(unittest.TestCase):
    def test_control_signed_values_and_position(self) -> None:
        original = MotorControl(True, 1, 65535, -100, 100)
        self.assertEqual(MotorControl.unpack(original.pack()), original)
        self.assertEqual(original.pack(), bytes.fromhex("81 FF FF 9C 64"))

    def test_control_range_validation(self) -> None:
        with self.assertRaises(ProtocolError):
            MotorControl(True, 0, 0, 101, 0).pack()
        with self.assertRaises(ProtocolError):
            MotorControl(True, 0, -1, 0, 0).pack()

    def test_motion_status(self) -> None:
        status = parse_motion_status(bytes.fromhex("81 34 12 F6 14"))
        self.assertTrue(status.enabled)
        self.assertFalse(status.error)
        self.assertEqual(status.mode, 1)
        self.assertEqual(status.actual_position, 0x1234)
        self.assertEqual(status.actual_speed, -10)
        self.assertEqual(status.actual_iq, 20)

    def test_fault_status(self) -> None:
        status = parse_motion_status(bytes.fromhex("C6 00 00 00 00"))
        self.assertTrue(status.error)
        self.assertEqual(status.error_code, 6)
        self.assertEqual(status.error_text, "电机堵转")

    def test_device_status(self) -> None:
        status = parse_device_status(bytes.fromhex("C0 5D E2"))
        self.assertEqual(status.voltage_mv, 24000)
        self.assertEqual(status.temperature_c, -30)


class ServiceCodecTests(unittest.TestCase):
    def test_read_and_write_did_layout(self) -> None:
        self.assertEqual(
            build_read_did(0x1018, 4), bytes.fromhex("40 18 10 04 00 00 00 00")
        )
        self.assertEqual(
            build_write_did(0x3001, 1, 921600),
            bytes.fromhex("23 01 30 01 00 10 0E 00"),
        )

    def test_set_node_id_layout(self) -> None:
        self.assertEqual(
            build_set_node_id_service(2, 0x31A2B4C5),
            bytes.fromhex("F2 00 00 02 C5 B4 A2 31"),
        )

    def test_read_response(self) -> None:
        response = parse_service_response(bytes.fromhex("43 18 10 04 C5 B4 A2 31"))
        self.assertTrue(response.ok)
        self.assertEqual(response.did, 0x1018)
        self.assertEqual(response.value, 0x31A2B4C5)

    def test_abort_response(self) -> None:
        response = parse_service_response(bytes.fromhex("80 18 10 04 02 00 01 06"))
        self.assertFalse(response.ok)
        self.assertEqual(response.return_code, 0x06010002)


if __name__ == "__main__":
    unittest.main()
