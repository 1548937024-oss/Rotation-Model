from __future__ import annotations

import unittest

from app import (
    DEFAULT_MID_ORDER_LABEL,
    IQ_LIMITS,
    MID_BYTEORDER_BY_LABEL,
    POSITION_LIMITS,
    SPEED_LIMITS,
    MotorHostApp,
    clamp_control_value,
)
from motor_protocol import ProtocolError


class ControlUiTests(unittest.TestCase):
    def test_default_mid_order_is_big_endian(self) -> None:
        self.assertEqual(DEFAULT_MID_ORDER_LABEL, "大端")
        self.assertEqual(MID_BYTEORDER_BY_LABEL[DEFAULT_MID_ORDER_LABEL], "big")

    def test_control_values_are_clamped_to_protocol_limits(self) -> None:
        self.assertEqual(clamp_control_value(-1, *POSITION_LIMITS), 0)
        self.assertEqual(clamp_control_value(70000, *POSITION_LIMITS), 65535)
        self.assertEqual(clamp_control_value(-120, *SPEED_LIMITS), -100)
        self.assertEqual(clamp_control_value(120, *SPEED_LIMITS), 100)
        self.assertEqual(clamp_control_value(-101, *IQ_LIMITS), -100)
        self.assertEqual(clamp_control_value(101, *IQ_LIMITS), 100)

    def test_clamping_updates_the_bound_ui_variable(self) -> None:
        class VariableStub:
            def __init__(self, value: str) -> None:
                self.value = value

            def get(self) -> str:
                return self.value

            def set(self, value: object) -> None:
                self.value = str(value)

        variable = VariableStub("70000")
        value = MotorHostApp._clamp_control_variable(variable, *POSITION_LIMITS)
        self.assertEqual(value, 65535)
        self.assertEqual(variable.get(), "65535")

    def test_contiguous_node_selection_rule(self) -> None:
        self.assertEqual(MotorHostApp._check_contiguous_nodes((1,)), (1,))
        self.assertEqual(MotorHostApp._check_contiguous_nodes((1, 2, 3)), (1, 2, 3))
        with self.assertRaises(ProtocolError):
            MotorHostApp._check_contiguous_nodes((1, 3))
        with self.assertRaises(ProtocolError):
            MotorHostApp._check_contiguous_nodes((2, 3))

    def test_multi_active_nodes_are_contiguous_from_node_one(self) -> None:
        class VariableStub:
            def __init__(self, value: bool) -> None:
                self.value = value

            def get(self) -> bool:
                return self.value

        class MultiTabStub:
            mn_active_vars = [VariableStub(i == 0) for i in range(5)]

            _multi_nodes = MotorHostApp._multi_nodes
            _check_contiguous_nodes = staticmethod(MotorHostApp._check_contiguous_nodes)

        stub = MultiTabStub()
        self.assertEqual(stub._multi_nodes(), (1,))
        stub.mn_active_vars[1].value = True
        stub.mn_active_vars[2].value = True
        self.assertEqual(stub._multi_nodes(), (1, 2, 3))
        stub.mn_active_vars[1].value = False
        with self.assertRaises(ProtocolError):
            stub._multi_nodes()

    def test_multi_enable_all_checks_every_node(self) -> None:
        class VariableStub:
            def __init__(self, value: bool) -> None:
                self.value = value

            def set(self, value: object) -> None:
                self.value = bool(value)

        class MultiTabStub:
            mn_enable_vars = [VariableStub(False) for _ in range(5)]

            def _sync_multi_cyclic(self) -> None:
                pass

        stub = MultiTabStub()
        MotorHostApp._multi_enable_all(stub)
        self.assertTrue(all(var.value for var in stub.mn_enable_vars))

    def test_raw_script_parser_builds_ordered_steps(self) -> None:
        text = (
            "# 注释\n"
            "0x201 80 00 00 00 00\n"
            "\n"
            "0x181 ?\n"
            "delay 150\n"
            "0x280 C0 5D 1F\n"
        )
        steps = MotorHostApp._parse_raw_script(text, 20)
        self.assertEqual(
            steps,
            [
                ("frame", 0x201, bytes.fromhex("80 00 00 00 00")),
                ("delay", 0.02),
                ("query", 0x181),
                ("delay", 0.15),
                ("frame", 0x280, bytes.fromhex("C0 5D 1F")),
            ],
        )

    def test_raw_script_parser_handles_leading_delay_and_zero_interval(self) -> None:
        steps = MotorHostApp._parse_raw_script("delay 0\n0x201 80", 20)
        self.assertEqual(steps, [("frame", 0x201, b"\x80")])
        steps = MotorHostApp._parse_raw_script("delay 50\n0x201 80", 20)
        self.assertEqual(steps, [("delay", 0.05), ("frame", 0x201, b"\x80")])

    def test_raw_script_parser_reports_line_number_and_invalid_query(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "第 2 行"):
            MotorHostApp._parse_raw_script("0x201 80\nXYZ 00", 20)
        with self.assertRaisesRegex(ProtocolError, "第 1 行"):
            MotorHostApp._parse_raw_script("0x181 ? 00", 20)


if __name__ == "__main__":
    unittest.main()
