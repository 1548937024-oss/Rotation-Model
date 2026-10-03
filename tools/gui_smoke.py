"""GUI 冒烟自检：用模拟设备启动界面，验证连接、状态、参数与遥测闭环。

需要能创建窗口的完整 Python 环境（含 Tcl/Tk）。用法：

    python tools/gui_smoke.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import app as app_module  # noqa: E402
from app import SIMULATOR_LABEL, MotorHostApp  # noqa: E402


def pump(app: MotorHostApp, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        app.update()
        time.sleep(0.02)


def main() -> int:
    app = MotorHostApp()
    app.update()
    app.port_var.set(SIMULATOR_LABEL)
    app.filter_var.set(True)
    app._toggle_connection()
    pump(app, 3.5)

    checks = {
        "connected": app.connected,
        "uid": app.uid_var.get(),
        "max_iq": app.max_iq_var.get(),
        "max_speed": app.max_speed_var.get(),
    }

    app.worker.submit("query_status")
    pump(app, 1.5)
    checks["enable"] = app.status_vars["enable"].get()
    checks["mode"] = app.status_vars["mode"].get()
    checks["voltage"] = app.status_vars["voltage"].get()
    checks["temperature"] = app.status_vars["temperature"].get()
    checks["error"] = app.status_vars["error"].get()

    app.worker.submit("read_telemetry")
    pump(app, 6.0)
    checks["phase_a"] = app.telemetry_vars["ia"].get()
    checks["peak"] = app.telemetry_vars["peak"].get()
    checks["enc_total"] = app.telemetry_vars["enc_total"].get()

    # 0x1011 重新加载后应自动回读只读量程。
    app_module.messagebox.askyesno = lambda *args, **kwargs: True
    app.max_iq_var.set("--")
    app.max_speed_var.set("--")
    app._reload_parameters()
    pump(app, 3.0)
    checks["max_iq_after_reload"] = app.max_iq_var.get()
    checks["max_speed_after_reload"] = app.max_speed_var.get()

    # 0x2020/03 控制帧超时 + 0x2021 安全状态
    app.worker.submit("read_safety")
    pump(app, 8.0)
    checks["control_timeout"] = app.control_timeout_var.get()
    checks["mcu_state"] = app.safety_vars["state"].get()
    checks["safety_flags"] = app.safety_vars["flags"].get()
    checks["safety_fw"] = app.safety_vars["fw"].get()
    checks["safety_proto"] = app.safety_vars["proto"].get()

    for key, value in checks.items():
        print(f"{key} = {value}")

    app.worker.shutdown()
    app.destroy()

    ok = (
        checks["connected"]
        and checks["max_iq"].startswith("1400")
        and checks["max_speed"].startswith("18000")
        and checks["error"] == "无故障"
        and checks["max_iq_after_reload"].startswith("1400")
        and checks["max_speed_after_reload"].startswith("18000")
        and checks["control_timeout"] == "200 ms"
        and checks["safety_fw"].startswith("V2.10")
        and checks["safety_proto"].startswith("V1.4")
    )
    print("UI SMOKE OK" if ok else "UI SMOKE FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
