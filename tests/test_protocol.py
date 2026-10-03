from __future__ import annotations

import unittest

from motor_protocol import (
    DEFAULT_BAUDRATE,
    DID_BY_KEY,
    MAX_IQ_MA,
    MAX_SPEED_RPM,
    MotorControl,
    ProtocolError,
    build_clear_fault_service,
    build_frame,
    build_group_control_frame,
    build_motor_enable_service,
    build_multi_control_frame,
    build_query_frame,
    build_read_did,
    build_reload_parameters,
    build_reply_tail,
    build_save_parameters,
    build_single_control_frame,
    build_write_did,
    crc8,
    decode_did_value,
    parse_device_status,
    parse_full_frame,
    parse_motion_status,
    parse_reply_tail,
    parse_service_response,
)


class ProtocolExampleTests(unittest.TestCase):
    """《外置驱动器通讯协议 V1.4》正文示例帧必须逐字节复现。"""

    def test_position_control_example(self) -> None:
        control = MotorControl(True, 0, 10, 0, 20)
        self.assertEqual(
            build_single_control_frame(1, control),
            bytes.fromhex("AA 01 02 05 80 0A 00 00 14 4A"),
        )

    def test_speed_control_examples(self) -> None:
        self.assertEqual(
            build_single_control_frame(1, MotorControl(True, 1, 0, 1, 20)),
            bytes.fromhex("AA 01 02 05 81 00 00 01 14 A1"),
        )
        self.assertEqual(
            build_single_control_frame(1, MotorControl(True, 1, 0, 100, 20)),
            bytes.fromhex("AA 01 02 05 81 00 00 64 14 15"),
        )

    def test_disable_control_example(self) -> None:
        self.assertEqual(
            build_single_control_frame(1, MotorControl(False, 0, 0, 0, 0)),
            bytes.fromhex("AA 01 02 05 00 00 00 00 00 2D"),
        )

    def test_service_examples(self) -> None:
        self.assertEqual(
            build_frame(0x601, build_motor_enable_service(True)),
            bytes.fromhex("AA 01 06 08 F1 10 01 00 00 00 00 00 FD"),
        )
        self.assertEqual(
            build_frame(0x601, build_motor_enable_service(False)),
            bytes.fromhex("AA 01 06 08 F1 10 00 00 00 00 00 00 D4"),
        )
        self.assertEqual(
            build_frame(0x601, build_clear_fault_service()),
            bytes.fromhex("AA 01 06 08 F1 12 01 00 00 00 00 00 44"),
        )

    def test_did_read_examples(self) -> None:
        self.assertEqual(
            build_frame(0x601, build_read_did(0x6073, 0x00)),
            bytes.fromhex("AA 01 06 08 40 73 60 00 00 00 00 00 EE"),
        )
        self.assertEqual(
            build_frame(0x601, build_read_did(0x607F, 0x00)),
            bytes.fromhex("AA 01 06 08 40 7F 60 00 00 00 00 00 71"),
        )

    def test_did_write_examples(self) -> None:
        self.assertEqual(
            build_frame(0x601, build_write_did(0x6073, 0x01, 1000)),
            bytes.fromhex("AA 01 06 08 23 73 60 01 E8 03 00 00 62"),
        )
        self.assertEqual(
            build_frame(0x601, build_write_did(0x202D, 0x01, 90)),
            bytes.fromhex("AA 01 06 08 23 2D 20 01 5A 00 00 00 AD"),
        )

    def test_save_and_reload_examples(self) -> None:
        self.assertEqual(
            build_frame(0x601, build_save_parameters()),
            bytes.fromhex("AA 01 06 08 23 10 10 00 01 00 00 00 2E"),
        )
        self.assertEqual(
            build_frame(0x601, build_reload_parameters()),
            bytes.fromhex("AA 01 06 08 23 11 10 00 01 00 00 00 F1"),
        )

    def test_full_zero_length_query_examples(self) -> None:
        self.assertEqual(build_query_frame(0x181), bytes.fromhex("AA 81 01 00 16"))
        self.assertEqual(build_query_frame(0x281), bytes.fromhex("AA 81 02 00 29"))

    def test_status_reply_examples(self) -> None:
        self.assertEqual(
            parse_reply_tail(0x181, bytes.fromhex("05 80 00 00 00 00 63")),
            bytes.fromhex("80 00 00 00 00"),
        )
        self.assertEqual(
            parse_reply_tail(0x181, bytes.fromhex("05 80 0A 00 00 14 93")),
            bytes.fromhex("80 0A 00 00 14"),
        )
        self.assertEqual(
            parse_reply_tail(0x281, bytes.fromhex("03 E0 2E 24 09")),
            bytes.fromhex("E0 2E 24"),
        )

    def test_service_reply_examples(self) -> None:
        enable = parse_reply_tail(0x581, bytes.fromhex("08 F1 10 01 00 00 00 00 00 E8"))
        self.assertEqual(enable, bytes.fromhex("F1 10 01 00 00 00 00 00"))
        clear = parse_reply_tail(0x581, bytes.fromhex("08 F1 12 01 00 00 00 00 00 51"))
        self.assertEqual(clear, bytes.fromhex("F1 12 01 00 00 00 00 00"))
        read_iq = parse_reply_tail(0x581, bytes.fromhex("08 43 73 60 00 78 05 00 00 8C"))
        self.assertEqual(read_iq, bytes.fromhex("43 73 60 00 78 05 00 00"))
        read_speed = parse_reply_tail(0x581, bytes.fromhex("08 43 7F 60 00 50 46 00 00 56"))
        self.assertEqual(read_speed, bytes.fromhex("43 7F 60 00 50 46 00 00"))
        save = parse_reply_tail(0x581, bytes.fromhex("08 60 10 10 00 01 00 00 00 D2"))
        self.assertEqual(save, bytes.fromhex("60 10 10 00 01 00 00 00"))
        reload_ = parse_reply_tail(0x581, bytes.fromhex("08 60 11 10 00 01 00 00 00 0D"))
        self.assertEqual(reload_, bytes.fromhex("60 11 10 00 01 00 00 00"))


class FramingTests(unittest.TestCase):
    def test_crc_uses_response_mid_not_request_mid(self) -> None:
        tail = bytes.fromhex("05 80 00 00 00 00 63")
        self.assertEqual(parse_reply_tail(0x181, tail), bytes.fromhex("80 00 00 00 00"))
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            parse_reply_tail(0x201, tail)

    def test_round_trip(self) -> None:
        payload = build_read_did(0x1018, 0x04)
        mid, decoded = parse_full_frame(build_frame(0x601, payload))
        self.assertEqual(mid, 0x601)
        self.assertEqual(decoded, payload)

    def test_crc_corruption_rejected(self) -> None:
        frame = bytearray(build_frame(0x201, bytes.fromhex("80 00 00 00 00")))
        frame[-1] ^= 1
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            parse_full_frame(bytes(frame))

    def test_empty_data_frame_allowed(self) -> None:
        frame = build_frame(0x181, b"")
        mid, data = parse_full_frame(frame)
        self.assertEqual(mid, 0x181)
        self.assertEqual(data, b"")

    def test_crc8_polynomial_vector(self) -> None:
        self.assertEqual(crc8(b"123456789"), 0xF4)


class SignalTests(unittest.TestCase):
    def test_signed_position_and_speed(self) -> None:
        control = MotorControl(True, 1, -32768, -100, 100)
        self.assertEqual(MotorControl.unpack(control.pack()), control)

    def test_iq_zero_is_allowed_by_codec(self) -> None:
        # 协议允许 0，但语义是回落到 1400 mA 默认限幅；界面下限另行为 1。
        self.assertEqual(MotorControl(True, 0, 0, 0, 0).pack(), bytes.fromhex("80 00 00 00 00"))

    def test_range_validation(self) -> None:
        with self.assertRaises(ProtocolError):
            MotorControl(True, 0, 32768, 0, 0).pack()
        with self.assertRaises(ProtocolError):
            MotorControl(True, 0, 0, 101, 0).pack()

    def test_motion_status_flags(self) -> None:
        status = parse_motion_status(bytes.fromhex("81 34 12 F6 14"))
        self.assertTrue(status.enabled)
        self.assertFalse(status.error)
        self.assertEqual(status.mode, 1)
        self.assertEqual(status.actual_position, 0x1234)
        self.assertEqual(status.actual_speed, -10)
        self.assertEqual(status.actual_iq, 20)
        self.assertEqual(status.speed_rpm, round(-10 * MAX_SPEED_RPM / 100))

    def test_error_codes_match_v14(self) -> None:
        self.assertEqual(
            parse_motion_status(bytes.fromhex("C3 00 00 00 00")).error_text,
            "过流、过温或 nFAULT 综合故障",
        )
        self.assertEqual(
            parse_motion_status(bytes.fromhex("C4 00 00 00 00")).error_text,
            "外环速度失控（项目扩展）",
        )

    def test_device_status(self) -> None:
        status = parse_device_status(bytes.fromhex("C0 5D E2"))
        self.assertEqual(status.voltage_mv, 24000)
        self.assertEqual(status.temperature_c, -30)


class ServiceResponseTests(unittest.TestCase):
    def test_read_response(self) -> None:
        response = parse_service_response(bytes.fromhex("43 73 60 00 78 05 00 00"))
        self.assertTrue(response.ok)
        self.assertEqual(response.did, 0x6073)
        self.assertEqual(response.sub_id, 0x00)
        self.assertEqual(response.value, MAX_IQ_MA)

    def test_motor_config_response(self) -> None:
        response = parse_service_response(bytes.fromhex("F1 10 01 00 00 00 00 00"))
        self.assertTrue(response.ok)
        self.assertEqual(response.sub_id, 0x10)
        self.assertEqual(response.return_code, 0x00)
        refused = parse_service_response(bytes.fromhex("F1 11 00 E5 00 00 00 00"))
        self.assertFalse(refused.ok)
        self.assertEqual(refused.return_code, 0xE5)

    def test_abort_codes(self) -> None:
        for raw, code in (
            ("80 73 60 00 02 00 01 06", 0x06010002),
            ("80 40 60 00 01 00 01 06", 0x06010001),
            ("80 FF FF 00 00 00 02 06", 0x06020000),
            ("80 73 60 01 30 00 09 06", 0x06090030),
        ):
            response = parse_service_response(bytes.fromhex(raw))
            self.assertFalse(response.ok)
            self.assertEqual(response.return_code, code)


class DidCatalogTests(unittest.TestCase):
    def test_max_values_are_read_only(self) -> None:
        self.assertFalse(DID_BY_KEY[(0x6073, 0x00)].writable)
        self.assertFalse(DID_BY_KEY[(0x607F, 0x00)].writable)
        self.assertFalse(DID_BY_KEY[(0x1018, 0x04)].writable)

    def test_protocol_mode_baud_rate(self) -> None:
        self.assertEqual(DEFAULT_BAUDRATE, 460800)

    def test_write_ranges(self) -> None:
        self.assertEqual((DID_BY_KEY[(0x6073, 0x01)].minimum, DID_BY_KEY[(0x6073, 0x01)].maximum), (0, 3000))
        self.assertEqual((DID_BY_KEY[(0x202D, 0x01)].minimum, DID_BY_KEY[(0x202D, 0x01)].maximum), (40, 110))
        self.assertEqual((DID_BY_KEY[(0x2016, 0x02)].minimum, DID_BY_KEY[(0x2016, 0x02)].maximum), (55, 110))

    def test_signed_extension_decode(self) -> None:
        self.assertEqual(decode_did_value(0x2010, 0x01, 0xFFE0), -32)
        self.assertEqual(decode_did_value(0x2010, 0x04, 1400), 1400)
        self.assertEqual(decode_did_value(0x2015, 0x01, 0xFFFFFFFF), -1)
        self.assertEqual(decode_did_value(0x2015, 0x02, 0xFFFFFFFF), 0xFFFFFFFF)


class GroupControlTests(unittest.TestCase):
    def test_group_control_layout(self) -> None:
        control = MotorControl(True, 1, 100, 30, 15)
        frame = build_group_control_frame((1, 3), control)
        mid, payload = parse_full_frame(frame)
        self.assertEqual(mid, 0x200)
        self.assertEqual(len(payload), 15)
        self.assertEqual(payload[0:5], control.pack())
        self.assertEqual(payload[5:10], MotorControl(False, 0, 0, 0, 0).pack())
        self.assertEqual(payload[10:15], control.pack())

    def test_multi_control_validation(self) -> None:
        with self.assertRaises(ProtocolError):
            build_multi_control_frame({})
        with self.assertRaises(ProtocolError):
            build_multi_control_frame({0: MotorControl(False, 0, 0, 0, 0)})


if __name__ == "__main__":
    unittest.main()
