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


if __name__ == "__main__":
    unittest.main()
