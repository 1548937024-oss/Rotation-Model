from __future__ import annotations

import unittest

import app as app_module
from app import (
    COMMON_BAUDRATES,
    IQ_LIMITS,
    POSITION_LIMITS,
    SPEED_LIMITS,
    MotorHostApp,
    clamp_control_value,
)
from motor_protocol import DID_BY_KEY, ProtocolError
from signal_filter import DisplayFilters, ExponentialFilter


class ControlUiTests(unittest.TestCase):
    def test_protocol_defaults_are_v14(self) -> None:
        self.assertEqual(COMMON_BAUDRATES[0], "460800")
        self.assertEqual(POSITION_LIMITS, (-32768, 32767))
        self.assertEqual(SPEED_LIMITS, (-100, 100))
        # TargetIq=0 表示固件默认 1400 mA，界面不允许误发 0。
        self.assertEqual(IQ_LIMITS, (1, 100))

    def test_control_values_are_clamped(self) -> None:
        self.assertEqual(clamp_control_value(-80000, *POSITION_LIMITS), -32768)
        self.assertEqual(clamp_control_value(70000, *POSITION_LIMITS), 32767)
        self.assertEqual(clamp_control_value(-120, *SPEED_LIMITS), -100)
        self.assertEqual(clamp_control_value(0, *IQ_LIMITS), 1)
        self.assertEqual(clamp_control_value(120, *IQ_LIMITS), 100)

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
        self.assertEqual(value, 32767)
        self.assertEqual(variable.get(), "32767")

    def test_read_only_max_values_are_in_did_catalog(self) -> None:
        self.assertFalse(DID_BY_KEY[(0x6073, 0x00)].writable)
        self.assertFalse(DID_BY_KEY[(0x607F, 0x00)].writable)
        self.assertIn((0x6073, 0x01), DID_BY_KEY)
        self.assertIn((0x2016, 0x04), DID_BY_KEY)

    def test_safety_config_entries_are_present(self) -> None:
        self.assertTrue(DID_BY_KEY[(0x2020, 0x01)].writable)
        self.assertTrue(DID_BY_KEY[(0x2020, 0x02)].writable)
        self.assertTrue(DID_BY_KEY[(0x2020, 0x03)].writable)
        self.assertFalse(DID_BY_KEY[(0x2021, 0x02)].writable)

    def test_safe_config_cross_check(self) -> None:
        class VariableStub:
            def __init__(self, value: str) -> None:
                self.value = value

            def get(self) -> str:
                return self.value

        class Stub:
            param_vars = {
                (0x2020, 0x01): VariableStub("-1000"),
                (0x2020, 0x02): VariableStub("2000"),
            }
            _check_safe_config = MotorHostApp._check_safe_config

        original = app_module.messagebox.showerror
        app_module.messagebox.showerror = lambda *args, **kwargs: None
        try:
            stub = Stub()
            self.assertTrue(stub._check_safe_config((0x2020, 0x01), -2000))
            self.assertFalse(stub._check_safe_config((0x2020, 0x01), 3000))
            self.assertFalse(stub._check_safe_config((0x2020, 0x02), -5000))
            self.assertTrue(stub._check_safe_config((0x2020, 0x02), 3000))
            self.assertTrue(stub._check_safe_config((0x2020, 0x03), 200))
        finally:
            app_module.messagebox.showerror = original


class RawScriptParserTests(unittest.TestCase):
    def test_full_and_short_query_syntax(self) -> None:
        text = (
            "# 注释\n"
            "0x201 80 0A 00 00 14\n"
            "0x181 ?\n"
            "0x281 ??\n"
            "delay 100\n"
            "0x601 40 73 60 00 00 00 00 00\n"
            "0x581 ?\n"
        )
        steps = MotorHostApp._parse_raw_script(text, 20)
        self.assertEqual(
            steps,
            [
                ("frame", 0x201, bytes.fromhex("80 0A 00 00 14")),
                ("delay", 0.02),
                ("query", 0x181, False),
                ("delay", 0.02),
                ("query", 0x281, True),
                ("delay", 0.1),
                ("frame", 0x601, bytes.fromhex("40 73 60 00 00 00 00 00")),
                ("delay", 0.02),
                ("query", 0x581, False),
            ],
        )

    def test_leading_delay_and_zero_interval(self) -> None:
        self.assertEqual(
            MotorHostApp._parse_raw_script("delay 0\n0x181 ?", 20),
            [("query", 0x181, False)],
        )
        self.assertEqual(
            MotorHostApp._parse_raw_script("delay 50\n0x181 ?", 20),
            [("delay", 0.05), ("query", 0x181, False)],
        )

    def test_invalid_lines_report_line_number(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "第 2 行"):
            MotorHostApp._parse_raw_script("0x201 80\nXYZ 00", 20)
        with self.assertRaisesRegex(ProtocolError, "第 1 行"):
            MotorHostApp._parse_raw_script("0x181 ? 00", 20)


class DisplayFilterTests(unittest.TestCase):
    def test_filter_disabled_passes_samples_through(self) -> None:
        filters = DisplayFilters(alpha=0.2)
        filters.enabled = False
        self.assertEqual(filters.apply_int("max_iq", 1400), 1400)
        self.assertEqual(filters.apply_int("max_iq", 1300), 1300)

    def test_filter_smooths_step(self) -> None:
        filters = DisplayFilters(alpha=0.25)
        filters.enabled = True
        self.assertEqual(filters.apply_int("max_iq", 1400), 1400)
        self.assertEqual(filters.apply_int("max_iq", 1000), 1300)
        self.assertEqual(filters.apply_int("max_iq", 1000), 1225)

    def test_exponential_filter_first_sample_initialises(self) -> None:
        item = ExponentialFilter(alpha=0.5)
        self.assertEqual(item.update(10), 10.0)
        self.assertEqual(item.update(20), 15.0)

    def test_alpha_is_clamped_to_valid_range(self) -> None:
        item = ExponentialFilter(alpha=0.0)
        self.assertAlmostEqual(item.alpha, 0.05)
        item.set_alpha(5.0)
        self.assertAlmostEqual(item.alpha, 1.0)


if __name__ == "__main__":
    unittest.main()
