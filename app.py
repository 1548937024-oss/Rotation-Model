"""外置驱动器 MicroDriver V2.10 上位机（协议 V1.4）。

面向 `docs/外置驱动器通讯协议_V1.4_20261003.md` 实现：

* 串口 460800 8N1，Node ID 1，MID 小端；
* Status1 / Status2 只读状态轮询，Control1 无应答；
* 0xF1 使能 / 运行模式 / 清错服务；
* DID 参数读写、0x1010 保存、0x1011 重新加载；
* 0x2010 / 0x2015 扩展遥测；
* 只读 Max Iq / Max Speed 从设备回读，并可做显示滤波。
"""

from __future__ import annotations

import queue
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Callable

from device_worker import SIMULATOR_PORT, DeviceWorker
from motor_protocol import (
    ALIGNMENT_TIME_S,
    DEFAULT_BAUDRATE,
    DEFAULT_NODE_ID,
    DID_CATALOG,
    DID_BY_KEY,
    MAX_IQ_MA,
    MAX_SPEED_RPM,
    DeviceStatus,
    MotionStatus,
    MotorControl,
    ProtocolError,
    RunMode,
    format_hex,
    parse_hex_bytes,
)
from signal_filter import DisplayFilters


APP_VERSION = "V2.10"
SIMULATOR_LABEL = "模拟设备（无需硬件）"
COMMON_BAUDRATES = ("460800", "921600", "230400", "115200", "57600", "38400", "19200", "9600")
POSITION_LIMITS = (-32768, 32767)
SPEED_LIMITS = (-100, 100)
# TargetIq=0 在固件中表示回落到 1400 mA 默认限幅，因此界面下限取 1。
IQ_LIMITS = (1, 100)
MODE_LABELS = {0: "位置模式", 1: "速度模式"}
STATUS_ROWS = (
    ("enable", "使能状态"),
    ("mode", "运行模式"),
    ("position", "实际位置"),
    ("speed", "实际速度"),
    ("rpm", "实测转速"),
    ("iq", "ActualIq"),
    ("iq_ma", "相电流峰值"),
    ("voltage", "母线电压"),
    ("temperature", "NTC 温度"),
    ("error", "故障状态"),
)
TELEMETRY_ROWS = (
    ("ia", "A 相电流"),
    ("ib", "B 相电流"),
    ("ic", "C 相电流"),
    ("peak", "相电流峰值"),
    ("enc_total", "编码器累计"),
    ("enc_z", "编码器 Z 计数"),
)


def clamp_control_value(value: int | str, low: int, high: int) -> int:
    """转换为整数并夹紧到闭区间。"""

    parsed = int(str(value).strip(), 10)
    return max(low, min(high, parsed))


class MotorHostApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(f"外置驱动器 MicroDriver 上位机 {APP_VERSION}")
        self.geometry("1360x860")
        self.minsize(1120, 720)
        self.worker = DeviceWorker()
        self.connected = False
        self._enable_confirmed = False
        self._align_notice_until = 0.0
        self._connected_text = ""
        self.motion: MotionStatus | None = None
        self.device: DeviceStatus | None = None
        self.filters = DisplayFilters(alpha=0.3)
        self.max_iq_ma = MAX_IQ_MA
        self.max_speed_rpm = MAX_SPEED_RPM
        self.param_vars: dict[tuple[int, int], tk.StringVar] = {}
        self.param_value_labels: dict[tuple[int, int], tk.StringVar] = {}
        self._build_style()
        self._build_variables()
        self._build_ui()
        self.refresh_ports()
        self.after(30, self._drain_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ 样式

    def _build_style(self) -> None:
        style = ttk.Style(self)
        available = style.theme_names()
        style.theme_use("vista" if "vista" in available else "clam")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("Value.TLabel", font=("Consolas", 12, "bold"))
        style.configure("Ok.TLabel", foreground="#157A36", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Bad.TLabel", foreground="#B3261E", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Warn.TLabel", foreground="#9A5A00", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Danger.TButton", foreground="#A00000", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Success.TButton", foreground="#157A36", font=("Microsoft YaHei UI", 10, "bold"))

    # ------------------------------------------------------------------ 变量

    def _build_variables(self) -> None:
        self.port_var = tk.StringVar(value=SIMULATOR_LABEL)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.node_var = tk.IntVar(value=DEFAULT_NODE_ID)
        self.connection_text = tk.StringVar(value="未连接")

        self.enable_var = tk.BooleanVar(value=False)
        self.mode_var = tk.StringVar(value="位置模式")
        self.position_var = tk.StringVar(value="0")
        self.speed_var = tk.StringVar(value="20")
        self.iq_var = tk.StringVar(value="20")
        self.cyclic_var = tk.BooleanVar(value=False)
        self.cyclic_ms_var = tk.IntVar(value=20)
        self.poll_var = tk.BooleanVar(value=True)
        self.poll_ms_var = tk.IntVar(value=20)
        self.filter_var = tk.BooleanVar(value=True)
        self.filter_alpha_var = tk.DoubleVar(value=0.3)
        self.disable_on_disconnect_var = tk.BooleanVar(value=True)

        self.status_vars = {key: tk.StringVar(value="--") for key, _ in STATUS_ROWS}
        self.telemetry_vars = {key: tk.StringVar(value="--") for key, _ in TELEMETRY_ROWS}
        self.error_style_var = tk.StringVar(value="尚未查询")

        self.uid_var = tk.StringVar(value="--")
        self.baud_read_var = tk.StringVar(value="--")
        self.max_iq_var = tk.StringVar(value="--")
        self.max_speed_var = tk.StringVar(value="--")
        self.service_result_var = tk.StringVar(value="尚无服务操作")

        for spec in DID_CATALOG:
            if spec.writable:
                initial = "" if spec.default is None else str(spec.default)
                self.param_vars[spec.key] = tk.StringVar(value=initial)
                self.param_value_labels[spec.key] = tk.StringVar(value="--")

        self.raw_mid_var = tk.StringVar(value="0x181")
        self.raw_data_var = tk.StringVar(value="00")
        self.raw_reply_var = tk.StringVar(value="--")
        self.raw_short_query_var = tk.BooleanVar(value=False)
        self.script_interval_var = tk.IntVar(value=20)

    # ------------------------------------------------------------------ 布局

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        title_row = ttk.Frame(root)
        title_row.pack(fill="x", pady=(0, 8))
        ttk.Label(
            title_row, text="外置驱动器 MicroDriver 上位机", style="Title.TLabel"
        ).pack(side="left")
        ttk.Label(
            title_row, text=f"{APP_VERSION}  协议 V1.4  |  460800 8N1  |  Node 1  |  MID 小端",
            foreground="#555555",
        ).pack(side="left", padx=12)
        self.connection_label = ttk.Label(
            title_row, textvariable=self.connection_text, style="Bad.TLabel"
        )
        self.connection_label.pack(side="right", padx=8)

        self._build_connection_bar(root)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, pady=(8, 0))
        control_tab = ttk.Frame(notebook, padding=10)
        service_tab = ttk.Frame(notebook, padding=10)
        raw_tab = ttk.Frame(notebook, padding=10)
        notebook.add(control_tab, text="控制与状态")
        notebook.add(service_tab, text="参数与服务")
        notebook.add(raw_tab, text="报文调试")
        self._build_control_tab(control_tab)
        self._build_service_tab(service_tab)
        self._build_raw_tab(raw_tab)

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="串口连接", padding=8)
        box.pack(fill="x")
        ttk.Label(box, text="端口").grid(row=0, column=0, padx=4, pady=3)
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(box, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Label(box, text="波特率").grid(row=0, column=3, padx=(14, 4))
        ttk.Combobox(box, textvariable=self.baud_var, values=COMMON_BAUDRATES, width=10).grid(
            row=0, column=4, padx=4
        )
        ttk.Label(box, text="Node ID").grid(row=0, column=5, padx=(14, 4))
        node_box = ttk.Spinbox(box, from_=1, to=31, textvariable=self.node_var, width=5, state="disabled")
        node_box.grid(row=0, column=6, padx=4)
        ttk.Label(box, text="（固件固定为 1）", foreground="#9A5A00").grid(
            row=0, column=7, padx=(0, 6)
        )
        self.connect_button = ttk.Button(box, text="连接", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=8, padx=(16, 4))
        ttk.Checkbutton(
            box, text="断开前下使能", variable=self.disable_on_disconnect_var
        ).grid(row=0, column=9, padx=4)

    def _labeled_slider(
        self,
        parent: ttk.Frame,
        row: int,
        label: str,
        variable: tk.Variable,
        low: int,
        high: int,
        unit: str,
    ) -> None:
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=5, pady=6)
        tk.Scale(
            parent,
            from_=low,
            to=high,
            orient="horizontal",
            resolution=1,
            showvalue=False,
            variable=variable,
            highlightthickness=0,
            borderwidth=0,
            length=240,
            command=self._sync_cyclic,
        ).grid(row=row, column=1, sticky="ew", padx=5)
        entry = ttk.Entry(parent, textvariable=variable, width=8, justify="right")
        entry.grid(row=row, column=2, sticky="w", padx=(4, 2))

        def clamp_entry(_event: tk.Event | None = None) -> None:
            self._clamp_control_variable(variable, low, high)
            self._sync_cyclic()

        entry.bind("<FocusOut>", clamp_entry)
        entry.bind("<Return>", clamp_entry)
        entry.bind("<KeyRelease>", lambda _event: self._sync_cyclic())
        ttk.Label(parent, text=f"{unit}  [{low}~{high}]").grid(
            row=row, column=3, sticky="w", padx=(2, 5)
        )

    # ------------------------------------------------------------------ 控制页

    def _build_control_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(2, weight=1)

        control = ttk.LabelFrame(tab, text="服务与运行控制", padding=10)
        control.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))

        service_row = ttk.Frame(control)
        service_row.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 6))
        ttk.Button(
            service_row, text="上使能", style="Success.TButton",
            command=lambda: self._motor_enable_service(True),
        ).pack(side="left", padx=3)
        ttk.Button(
            service_row, text="下使能", command=lambda: self._motor_enable_service(False)
        ).pack(side="left", padx=3)
        ttk.Button(
            service_row, text="清错 (0xF1 0x12)", style="Danger.TButton",
            command=self._clear_fault,
        ).pack(side="left", padx=3)
        ttk.Button(service_row, text="应用运行模式", command=self._apply_run_mode).pack(
            side="left", padx=3
        )

        ttk.Checkbutton(
            control, text="Control1 使能位", variable=self.enable_var, command=self._sync_cyclic
        ).grid(row=1, column=0, sticky="w", padx=5, pady=6)
        mode_combo = ttk.Combobox(
            control, textvariable=self.mode_var, values=("位置模式", "速度模式"),
            state="readonly", width=12,
        )
        ttk.Label(control, text="运行模式").grid(row=1, column=1, sticky="e", padx=4)
        mode_combo.grid(row=1, column=2, sticky="w", padx=5)
        mode_combo.bind("<<ComboboxSelected>>", lambda _event: self._sync_cyclic())

        self._labeled_slider(control, 2, "目标位置", self.position_var, *POSITION_LIMITS, "LSB（int16）")
        self._labeled_slider(control, 3, "目标速度", self.speed_var, *SPEED_LIMITS, "% 额定转速")
        self._labeled_slider(control, 4, "目标 Iq 限幅", self.iq_var, *IQ_LIMITS, "% Max Iq")

        buttons = ttk.Frame(control)
        buttons.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(8, 3))
        ttk.Button(buttons, text="发送一次", command=self._send_control).pack(side="left", padx=4)
        ttk.Button(
            buttons, text="立即下使能", style="Danger.TButton", command=self._emergency_disable
        ).pack(side="left", padx=4)
        ttk.Checkbutton(
            buttons, text="周期发送", variable=self.cyclic_var, command=self._configure_cyclic
        ).pack(side="left", padx=(18, 3))
        ttk.Spinbox(
            buttons, from_=1, to=1000, textvariable=self.cyclic_ms_var, width=7,
            command=self._sync_cyclic,
        ).pack(side="left", padx=3)
        ttk.Label(buttons, text="ms").pack(side="left")

        notes = ttk.LabelFrame(control, text="协议要点", padding=8)
        notes.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        ttk.Label(
            notes,
            justify="left",
            foreground="#555555",
            text=(
                "1. Control1 没有应答，请用 Status1 确认执行结果。\n"
                "2. 首次使能若未建零点，固件会先做约 1200 ms 对齐，对齐期间仍可查询状态。\n"
                "3. TargetIq=0 表示回落到固件默认 1400 mA 限幅，不是零电流；本界面下限为 1%。\n"
                "4. 清错后建议等待 ≥10 ms，再重新使能。"
            ),
        ).pack(anchor="w")

        status = ttk.LabelFrame(tab, text="实时状态（Status1 / Status2）", padding=10)
        status.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0), pady=(0, 8))
        status.columnconfigure(0, weight=1)

        self.status_table = ttk.Treeview(status, columns=("item", "value"), show="headings", height=10)
        self.status_table.heading("item", text="项目")
        self.status_table.heading("value", text="数值")
        self.status_table.column("item", width=140, anchor="w", stretch=False)
        self.status_table.column("value", width=260, anchor="w", stretch=True)
        self.status_table.grid(row=0, column=0, sticky="nsew")
        for key, label in STATUS_ROWS:
            self.status_table.insert("", "end", iid=key, values=(label, "--"))

        self.error_label = ttk.Label(status, textvariable=self.error_style_var, style="Ok.TLabel")
        self.error_label.grid(row=1, column=0, sticky="w", padx=5, pady=6)

        telemetry = ttk.LabelFrame(status, text="扩展遥测（0x2010 / 0x2015）", padding=8)
        telemetry.grid(row=2, column=0, sticky="ew", pady=(2, 6))
        for index, (key, label) in enumerate(TELEMETRY_ROWS):
            ttk.Label(telemetry, text=label).grid(row=index // 3, column=(index % 3) * 2, sticky="w", padx=4, pady=3)
            ttk.Label(telemetry, textvariable=self.telemetry_vars[key], style="Value.TLabel").grid(
                row=index // 3, column=(index % 3) * 2 + 1, sticky="w", padx=(0, 12)
            )
        ttk.Button(telemetry, text="读取扩展遥测", command=self._read_telemetry).grid(
            row=2, column=0, columnspan=6, sticky="e", padx=4, pady=(4, 0)
        )

        polling = ttk.Frame(status)
        polling.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(
            polling, text="自动查询", variable=self.poll_var, command=self._configure_poll
        ).pack(side="left", padx=4)
        ttk.Spinbox(polling, from_=5, to=500, textvariable=self.poll_ms_var, width=7).pack(
            side="left", padx=3
        )
        ttk.Label(polling, text="ms").pack(side="left")
        ttk.Button(polling, text="立即查询", command=self._query_status).pack(side="left", padx=12)

        filtering = ttk.Frame(status)
        filtering.grid(row=4, column=0, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(
            filtering, text="显示滤波", variable=self.filter_var, command=self._apply_filter_settings
        ).pack(side="left", padx=(4, 8))
        ttk.Label(filtering, text="平滑系数").pack(side="left", padx=(8, 3))
        ttk.Spinbox(
            filtering, from_=0.05, to=1.0, increment=0.05, textvariable=self.filter_alpha_var,
            width=6, command=self._apply_filter_settings,
        ).pack(side="left", padx=3)
        ttk.Label(filtering, text="（仅影响显示，Max Iq/Max Speed 也按滤波值显示）", foreground="#555555").pack(
            side="left", padx=8
        )

        log_box = ttk.LabelFrame(tab, text="通信日志", padding=6)
        log_box.grid(row=2, column=0, columnspan=2, sticky="nsew")
        log_box.rowconfigure(0, weight=1)
        log_box.columnconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(
            log_box, height=12, font=("Consolas", 9), wrap="none", state="disabled"
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        ttk.Button(log_box, text="清空日志", command=self._clear_log).grid(row=1, column=0, sticky="e", pady=(5, 0))

    # ------------------------------------------------------------------ 参数页

    def _service_row(
        self,
        parent: ttk.Frame,
        row: int,
        title: str,
        read_command: Callable[[], None] | None = None,
        variable: tk.Variable | None = None,
        write_command: Callable[[], None] | None = None,
        unit: str = "",
    ) -> None:
        ttk.Label(parent, text=title).grid(row=row, column=0, sticky="w", padx=5, pady=6)
        if read_command:
            ttk.Button(parent, text="读取", command=read_command).grid(row=row, column=1, padx=4)
        if variable is not None:
            ttk.Entry(parent, textvariable=variable, width=16).grid(row=row, column=2, padx=4)
        if unit:
            ttk.Label(parent, text=unit).grid(row=row, column=3, sticky="w", padx=3)
        if write_command:
            ttk.Button(parent, text="写入", command=write_command).grid(row=row, column=4, padx=4)

    def _build_service_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)

        info = ttk.LabelFrame(tab, text="设备信息（只读）", padding=10)
        info.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        self._service_row(info, 0, "Device UID (0x1018/04)", self._read_uid, self.uid_var)
        self._service_row(info, 1, "当前波特率 (0x3001/01)", self._read_baud, self.baud_read_var, unit="bit/s")
        self._service_row(
            info, 2, "Max Iq Current (0x6073/00)", lambda: self._read_did(0x6073, 0x00),
            self.max_iq_var, unit="mA（滤波显示）",
        )
        self._service_row(
            info, 3, "Max Speed (0x607F/00)", lambda: self._read_did(0x607F, 0x00),
            self.max_speed_var, unit="rpm（滤波显示）",
        )
        ttk.Label(
            info,
            text="Max Iq / Max Speed 为只读量程，界面显示回读值的滤波结果，并用于百分比换算。",
            wraplength=470, foreground="#555555",
        ).grid(row=4, column=0, columnspan=5, sticky="w", padx=5, pady=(8, 0))

        params = ttk.LabelFrame(tab, text="可写参数（写入 RAM，需保存才掉电保持）", padding=10)
        params.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 8))
        for index, spec in enumerate(spec for spec in DID_CATALOG if spec.writable):
            variable = self.param_vars[spec.key]
            self._service_row(
                params,
                index,
                f"{spec.name} (0x{spec.did:04X}/{spec.sub:02X})",
                lambda key=spec.key: self._read_did(*key),
                variable,
                lambda key=spec.key: self._write_param(key),
                spec.unit,
            )
            ttk.Label(
                params, textvariable=self.param_value_labels[spec.key], foreground="#555555"
            ).grid(row=index, column=5, sticky="w", padx=4)

        actions = ttk.Frame(tab)
        actions.grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Button(
            actions, text="保存参数到 EEPROM (0x1010)", command=self._save_parameters
        ).pack(side="left", padx=4)
        ttk.Button(
            actions, text="从 EEPROM 重新加载 (0x1011)", command=self._reload_parameters
        ).pack(side="left", padx=4)
        ttk.Button(actions, text="回读全部可写参数", command=self._read_all_params).pack(side="left", padx=4)

        result = ttk.LabelFrame(tab, text="最近一次服务结果", padding=10)
        result.grid(row=2, column=0, sticky="nsew", padx=(0, 5), pady=(8, 0))
        self.service_result_label = ttk.Label(
            result, textvariable=self.service_result_var, wraplength=470
        )
        self.service_result_label.pack(fill="both", expand=True)

        notes = ttk.LabelFrame(tab, text="协议注意事项", padding=10)
        notes.grid(row=2, column=1, sticky="nsew", padx=(5, 0), pady=(8, 0))
        ttk.Label(
            notes,
            justify="left",
            foreground="#555555",
            wraplength=470,
            text=(
                "1. 0x1011 是重新加载 EEPROM 中已保存的参数，不是恢复出厂默认值；\n"
                "   加载后请再次读取 0x6073/00 与 0x607F/00 确认量程。\n"
                "2. 保存参数要求电机未运行。\n"
                "3. 当前过流保护入口是硬件 OCP 与 nFAULT，堵转参数不构成唯一保护判据。\n"
                "4. 0x6040 明确返回 Read not allowed；清错请使用 0xF1 0x12。\n"
                "5. 相电流采样量程约 ±1650 mA，硬件 OCP 为 2500 mA，之间为不可观测区。\n"
                "6. 当前不支持总线设置零位与修改 Node ID。"
            ),
        ).pack(anchor="w")

    # ------------------------------------------------------------------ 调试页

    def _build_raw_tab(self, tab: ttk.Frame) -> None:
        frame = ttk.LabelFrame(tab, text="原始报文", padding=12)
        frame.pack(fill="x")
        ttk.Label(frame, text="MID").grid(row=0, column=0, sticky="w", padx=5, pady=6)
        ttk.Entry(frame, textvariable=self.raw_mid_var, width=15).grid(row=0, column=1, padx=5)
        ttk.Checkbutton(
            frame, text="查询使用 3 字节短询问（默认完整零长度帧）", variable=self.raw_short_query_var
        ).grid(row=0, column=2, columnspan=3, sticky="w", padx=12)
        ttk.Label(frame, text="Data（十六进制）").grid(row=1, column=0, sticky="w", padx=5, pady=6)
        ttk.Entry(frame, textvariable=self.raw_data_var, width=70).grid(
            row=1, column=1, columnspan=4, sticky="ew", padx=5
        )
        ttk.Button(frame, text="发送完整帧", command=self._send_raw_frame).grid(
            row=2, column=1, sticky="w", padx=5, pady=8
        )
        ttk.Button(frame, text="发送查询", command=self._send_raw_query).grid(
            row=2, column=2, sticky="w", padx=5, pady=8
        )
        ttk.Label(frame, text="响应 Data").grid(row=3, column=0, sticky="w", padx=5, pady=6)
        ttk.Entry(frame, textvariable=self.raw_reply_var, width=70, state="readonly").grid(
            row=3, column=1, columnspan=4, sticky="ew", padx=5
        )
        frame.columnconfigure(4, weight=1)

        regression = ttk.LabelFrame(tab, text="V1.4 快速回归", padding=8)
        regression.pack(fill="x", pady=10)
        ttk.Button(
            regression, text="生成快速回归脚本（连接检查 + 读 Max Iq / Max Speed / UID）",
            command=self._fill_regression_script,
        ).pack(side="left", padx=4)
        ttk.Label(
            regression,
            text="脚本会先查询 Status1，再依次读取 0x6073/00、0x607F/00、0x1018/04。",
            foreground="#555555",
        ).pack(side="left", padx=8)

        script = ttk.LabelFrame(tab, text="脚本调试（多条报文）", padding=10)
        script.pack(fill="both", expand=True)
        script.columnconfigure(0, weight=1)
        script.columnconfigure(1, weight=1)
        script.rowconfigure(0, weight=1)

        script_panel = ttk.Frame(script)
        script_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        script_panel.columnconfigure(0, weight=1)
        script_panel.rowconfigure(0, weight=1)
        self.script_text = scrolledtext.ScrolledText(
            script_panel, height=14, font=("Consolas", 9), wrap="none"
        )
        self.script_text.grid(row=0, column=0, sticky="nsew")
        script_actions = ttk.Frame(script_panel)
        script_actions.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(script_actions, text="执行脚本", command=self._run_raw_script).pack(side="left", padx=4)
        ttk.Button(script_actions, text="清空脚本", command=self._clear_raw_script).pack(side="left", padx=4)
        ttk.Label(script_actions, text="默认间隔").pack(side="left", padx=(16, 3))
        ttk.Spinbox(script_actions, from_=0, to=60000, textvariable=self.script_interval_var, width=7).pack(
            side="left", padx=3
        )
        ttk.Label(script_actions, text="ms").pack(side="left")
        ttk.Label(
            script_actions,
            text="语法：MID + Data；MID ? 完整零长度查询；MID ?? 短询问；delay 100；# 注释",
            foreground="#555555",
        ).pack(side="left", padx=16)

        response_panel = ttk.Frame(script)
        response_panel.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        response_panel.columnconfigure(0, weight=1)
        response_panel.rowconfigure(1, weight=1)
        ttk.Label(response_panel, text="多报文响应", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        self.raw_reply_text = scrolledtext.ScrolledText(
            response_panel, height=14, font=("Consolas", 9), wrap="none", state="disabled"
        )
        self.raw_reply_text.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        response_actions = ttk.Frame(response_panel)
        response_actions.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(response_actions, text="清空响应", command=self._clear_raw_replies).pack(side="left", padx=4)

    # ------------------------------------------------------------------ 通用

    def refresh_ports(self) -> None:
        values = [SIMULATOR_LABEL]
        try:
            from serial.tools import list_ports

            values.extend(f"{port.device} — {port.description}" for port in list_ports.comports())
        except ImportError:
            pass
        current = self.port_var.get()
        self.port_combo["values"] = values
        self.port_var.set(current if current in values else values[0])

    def _selected_port(self) -> str:
        text = self.port_var.get()
        return SIMULATOR_PORT if text == SIMULATOR_LABEL else text.split(" — ", 1)[0]

    @staticmethod
    def _clamp_control_variable(variable: tk.Variable, low: int, high: int) -> int:
        try:
            value = clamp_control_value(variable.get(), low, high)
        except (TypeError, ValueError, tk.TclError):
            value = low
        variable.set(value)
        return value

    def _require_connected(self) -> bool:
        if self.connected:
            return True
        messagebox.showwarning("未连接", "请先连接串口或模拟设备。")
        return False

    def _toggle_connection(self) -> None:
        if self.connected:
            self.worker.submit("disconnect", bool(self.disable_on_disconnect_var.get()))
            return
        try:
            baud = int(self.baud_var.get(), 0)
            if baud <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("参数错误", "波特率必须为正整数")
            return
        self.connection_text.set("连接中…")
        self.worker.submit("connect", self._selected_port(), baud, DEFAULT_NODE_ID)

    # ------------------------------------------------------------------ 控制

    def _control_value(self, *, force_disable: bool = False) -> MotorControl:
        mode = RunMode.POSITION if self.mode_var.get() == "位置模式" else RunMode.SPEED
        position = self._clamp_control_variable(self.position_var, *POSITION_LIMITS)
        speed = self._clamp_control_variable(self.speed_var, *SPEED_LIMITS)
        iq = self._clamp_control_variable(self.iq_var, *IQ_LIMITS)
        return MotorControl(
            enable=False if force_disable else bool(self.enable_var.get()),
            mode=int(mode),
            target_position=position,
            target_speed=speed,
            target_iq=iq,
        )

    def _confirm_enable(self, enabling: bool) -> bool:
        if not enabling or self._enable_confirmed:
            return True
        confirmed = messagebox.askyesno(
            "确认上使能",
            "发送后电机可能立即运动，首次使能还会先做约 1.2 s 对齐。\n"
            "请确认机构活动范围安全，目标位置/速度/Iq 已核对。",
            icon="warning",
        )
        if confirmed:
            self._enable_confirmed = True
        return confirmed

    def _send_control(self) -> None:
        if not self._require_connected():
            return
        try:
            control = self._control_value()
            control.pack()
        except (ValueError, tk.TclError, ProtocolError) as exc:
            messagebox.showerror("控制参数错误", str(exc))
            return
        if self._confirm_enable(control.enable):
            self.worker.submit("send_control", control)

    def _emergency_disable(self) -> None:
        if not self._require_connected():
            return
        self.enable_var.set(False)
        self.cyclic_var.set(False)
        self.worker.submit("emergency_disable")

    def _configure_cyclic(self) -> None:
        if not self._require_connected():
            self.cyclic_var.set(False)
            return
        try:
            control = self._control_value()
            interval = int(self.cyclic_ms_var.get())
            control.pack()
            if not 1 <= interval <= 1000:
                raise ProtocolError("周期必须为 1~1000 ms")
        except (ValueError, tk.TclError, ProtocolError) as exc:
            self.cyclic_var.set(False)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        if self.cyclic_var.get() and not self._confirm_enable(control.enable):
            self.cyclic_var.set(False)
            return
        self.worker.submit("configure_cyclic", control, self.cyclic_var.get(), interval / 1000)

    def _sync_cyclic(self, _value: object | None = None) -> None:
        if not (self.connected and self.cyclic_var.get()):
            return
        try:
            control = self._control_value()
            control.pack()
            interval = int(self.cyclic_ms_var.get())
            if not 1 <= interval <= 1000:
                raise ProtocolError("周期必须为 1~1000 ms")
        except (ValueError, tk.TclError):
            return
        except ProtocolError as exc:
            self.cyclic_var.set(False)
            self.worker.submit("configure_cyclic", control, False, 0.02)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        self.worker.submit("configure_cyclic", control, True, interval / 1000)

    def _motor_enable_service(self, enable: bool) -> None:
        if not self._require_connected():
            return
        if enable and not self._confirm_enable(True):
            return
        self.enable_var.set(enable)
        self.worker.submit("motor_enable", enable)
        if enable:
            self._align_notice_until = time.monotonic() + ALIGNMENT_TIME_S + 0.2
            self.error_style_var.set("已发送使能；若首次对齐约 1.2 s，请观察 Status1")
            self.error_label.configure(style="Warn.TLabel")

    def _apply_run_mode(self) -> None:
        if not self._require_connected():
            return
        mode = int(RunMode.POSITION if self.mode_var.get() == "位置模式" else RunMode.SPEED)
        self.worker.submit("run_mode", mode)

    def _clear_fault(self) -> None:
        if not self._require_connected():
            return
        if messagebox.askyesno(
            "确认清错",
            "将发送 0xF1 0x12 清错，等待 ≥10 ms 后再重新使能。是否继续？",
        ):
            self.worker.submit("clear_fault")

    # ------------------------------------------------------------------ 查询

    def _configure_poll(self) -> None:
        if not self._require_connected():
            self.poll_var.set(False)
            return
        try:
            interval = int(self.poll_ms_var.get())
            if not 5 <= interval <= 500:
                raise ProtocolError("查询周期必须为 5~500 ms")
        except (ValueError, tk.TclError, ProtocolError) as exc:
            self.poll_var.set(False)
            messagebox.showerror("查询周期错误", str(exc))
            return
        self.worker.submit("configure_poll", self.poll_var.get(), interval / 1000)

    def _query_status(self) -> None:
        if self._require_connected():
            self.worker.submit("query_status")

    def _read_telemetry(self) -> None:
        if self._require_connected():
            self.worker.submit("read_telemetry")

    def _apply_filter_settings(self) -> None:
        try:
            alpha = float(self.filter_alpha_var.get())
        except (TypeError, ValueError, tk.TclError):
            alpha = 0.3
        self.filters.enabled = bool(self.filter_var.get())
        self.filters.set_alpha(alpha)
        if not self.filters.enabled:
            self.filters.reset()
            self._render_status()

    # ------------------------------------------------------------------ 服务

    def _submit_service(self, payload: bytes, note: str) -> None:
        if self._require_connected():
            self.worker.submit("service", payload, note)

    def _read_did(self, did: int, sub_id: int) -> None:
        if not self._require_connected():
            return
        self.worker.submit("read_did", did, sub_id, f"读取 DID 0x{did:04X}/{sub_id}")

    def _read_uid(self) -> None:
        self._read_did(0x1018, 0x04)

    def _read_baud(self) -> None:
        self._read_did(0x3001, 0x01)

    def _read_all_params(self) -> None:
        if not self._require_connected():
            return
        for spec in DID_CATALOG:
            if spec.writable:
                self._read_did(spec.did, spec.sub)

    def _write_param(self, key: tuple[int, int]) -> None:
        spec = DID_BY_KEY[key]
        try:
            value = int(self.param_vars[key].get(), 0)
        except ValueError:
            messagebox.showerror("参数错误", f"{spec.name} 必须是整数")
            return
        if not spec.minimum <= value <= spec.maximum:
            messagebox.showerror(
                "参数错误", f"{spec.name} 必须位于 [{spec.minimum}, {spec.maximum}]"
            )
            return
        if messagebox.askyesno("确认写入", f"确认写入 {spec.name} = {value} {spec.unit}？"):
            self.worker.submit(
                "write_did", spec.did, spec.sub, value, f"写入 {spec.name} (0x{spec.did:04X}/{spec.sub:02X})"
            )

    def _save_parameters(self) -> None:
        if not self._require_connected():
            return
        if messagebox.askyesno("确认保存", "将把 RAM 参数写入 EEPROM，要求电机未运行。是否继续？"):
            self.worker.submit("save_parameters")

    def _reload_parameters(self) -> None:
        if not self._require_connected():
            return
        if messagebox.askyesno(
            "确认重新加载",
            "将从 EEPROM 重新加载参数（不是恢复出厂默认）。\n"
            "加载后请重新读取 Max Iq / Max Speed 确认量程。是否继续？",
        ):
            self.worker.submit("reload_parameters")
            # 0x1011 会把 EEPROM 中的旧量程带入 RAM，稍后自动回读确认。
            self.after(500, self._refresh_ratings)

    def _refresh_ratings(self) -> None:
        if not self.connected:
            return
        self._append_log("INFO 重新读取 Max Iq / Max Speed / 可写参数")
        self._read_did(0x6073, 0x00)
        self._read_did(0x607F, 0x00)
        self._read_all_params()

    # ------------------------------------------------------------------ 报文调试

    @staticmethod
    def _parse_mid(text: str) -> int:
        mid = int(text.strip(), 0)
        if not 0 <= mid <= 0x7FF:
            raise ProtocolError("MID 必须位于 0x000~0x7FF")
        return mid

    def _send_raw_frame(self) -> None:
        if not self._require_connected():
            return
        try:
            mid = self._parse_mid(self.raw_mid_var.get())
            data = parse_hex_bytes(self.raw_data_var.get(), allow_empty=True)
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("原始报文错误", str(exc))
            return
        self.worker.submit("raw_frame", mid, data)

    def _send_raw_query(self) -> None:
        if not self._require_connected():
            return
        try:
            mid = self._parse_mid(self.raw_mid_var.get())
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("MID 错误", str(exc))
            return
        self.worker.submit("raw_query", mid, bool(self.raw_short_query_var.get()))

    @staticmethod
    def _parse_raw_script(text: str, interval_ms: int) -> list[tuple]:
        default_delay = max(0, int(interval_ms)) / 1000
        steps: list[tuple] = []
        pending_delay: float | None = None
        seen_message = False
        for line_no, raw_line in enumerate(text.splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            lower = line.lower()
            if lower.startswith("delay ") or lower == "delay":
                try:
                    ms = int(line.split()[1], 0)
                except (IndexError, ValueError) as exc:
                    raise ProtocolError(f"第 {line_no} 行 delay 需要毫秒数") from exc
                if not 0 <= ms <= 60000:
                    raise ProtocolError(f"第 {line_no} 行 delay 必须在 0~60000 ms")
                pending_delay = ms / 1000
                continue
            parts = line.split()
            try:
                mid = int(parts[0], 0)
            except ValueError as exc:
                raise ProtocolError(f"第 {line_no} 行 MID 无效：{parts[0]}") from exc
            if not 0 <= mid <= 0x7FF:
                raise ProtocolError(f"第 {line_no} 行 MID 必须位于 0x000~0x7FF")
            if len(parts) >= 2 and parts[1] in ("?", "??"):
                if len(parts) != 2:
                    raise ProtocolError(f"第 {line_no} 行查询报文只能包含 MID 和 ?/??")
                step: tuple = ("query", mid, parts[1] == "??")
            else:
                try:
                    data = parse_hex_bytes(" ".join(parts[1:]))
                except ProtocolError as exc:
                    raise ProtocolError(f"第 {line_no} 行 Data 无效：{exc}") from exc
                step = ("frame", mid, data)
            delay = pending_delay if pending_delay is not None else default_delay
            if (seen_message or pending_delay is not None) and delay > 0:
                steps.append(("delay", delay))
            pending_delay = None
            steps.append(step)
            seen_message = True
        return steps

    def _fill_regression_script(self) -> None:
        script = "\n".join(
            [
                "# V1.4 快速回归：连接检查 + 读回只读量程",
                "0x181 ?",
                "delay 20",
                "0x601 40 73 60 00 00 00 00 00",
                "delay 20",
                "0x581 ?",
                "delay 20",
                "0x601 40 7F 60 00 00 00 00 00",
                "delay 20",
                "0x581 ?",
                "delay 20",
                "0x601 40 18 10 04 00 00 00 00",
                "delay 20",
                "0x581 ?",
            ]
        )
        self.script_text.delete("1.0", "end")
        self.script_text.insert("1.0", script)

    def _run_raw_script(self) -> None:
        if not self._require_connected():
            return
        try:
            interval = int(self.script_interval_var.get())
            steps = self._parse_raw_script(self.script_text.get("1.0", "end-1c"), interval)
        except (ValueError, tk.TclError, ProtocolError) as exc:
            messagebox.showerror("脚本解析错误", str(exc))
            return
        if not steps:
            messagebox.showwarning("脚本为空", "请先输入至少一条报文。")
            return
        self.worker.submit("raw_script", steps)
        self._append_log(f"INFO 脚本已提交，共 {len(steps)} 步")

    def _clear_raw_script(self) -> None:
        self.script_text.delete("1.0", "end")

    # ------------------------------------------------------------------ 日志与渲染

    def _append_raw_reply(self, mid: int, data: bytes) -> None:
        self.raw_reply_text.configure(state="normal")
        self.raw_reply_text.insert(
            "end", f"{time.strftime('%H:%M:%S')}  MID 0x{mid:03X}: {format_hex(data)}\n"
        )
        line_count = int(self.raw_reply_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.raw_reply_text.delete("1.0", "200.0")
        self.raw_reply_text.see("end")
        self.raw_reply_text.configure(state="disabled")

    def _clear_raw_replies(self) -> None:
        self.raw_reply_text.configure(state="normal")
        self.raw_reply_text.delete("1.0", "end")
        self.raw_reply_text.configure(state="disabled")

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.log_text.delete("1.0", "200.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _set_status(self, key: str, text: str) -> None:
        self.status_vars[key].set(text)
        self.status_table.item(key, values=(dict(STATUS_ROWS)[key], text))

    def _render_status(self) -> None:
        motion = self.motion
        device = self.device
        if motion is None:
            return
        f = self.filters
        self._set_status("enable", "已使能" if motion.enabled else "未使能")
        if motion.error:
            self._set_status("mode", "--")
            self._set_status("error", f"故障 {motion.error_code}：{motion.error_text}")
            self.error_style_var.set(f"故障 {motion.error_code}：{motion.error_text}（请清错后重新使能）")
            self.error_label.configure(style="Bad.TLabel")
        else:
            self._set_status("mode", MODE_LABELS.get(motion.mode, f"模式 {motion.mode}"))
            self._set_status("error", "无故障")
            if time.monotonic() >= self._align_notice_until:
                self.error_style_var.set("无故障")
                self.error_label.configure(style="Ok.TLabel")
        self._set_status("position", f"{motion.actual_position} LSB")
        self._set_status("speed", f"{motion.actual_speed} %")
        rpm = self.max_speed_rpm * motion.actual_speed / 100 if self.max_speed_rpm else 0
        self._set_status("rpm", f"{f.apply_int('rpm', round(rpm))} rpm")
        iq_ma = self.max_iq_ma * abs(motion.actual_iq) / 100 if self.max_iq_ma else 0
        self._set_status("iq", f"{motion.actual_iq} %")
        self._set_status("iq_ma", f"{f.apply_int('iq_ma', round(iq_ma))} mA（{MAX_IQ_MA} mA 量程估计）")
        if device is not None:
            voltage = f.apply_int("voltage", device.voltage_mv)
            self._set_status("voltage", f"{voltage / 1000:.3f} V")
            self._set_status("temperature", f"{f.apply_int('temperature', device.temperature_c)} °C")

    def _update_motion(self, status: MotionStatus) -> None:
        self.motion = status
        self._render_status()

    def _update_device(self, status: DeviceStatus) -> None:
        self.device = status
        self._render_status()

    def _update_telemetry(self, values: dict[tuple[int, int], int]) -> None:
        labels = {
            "ia": (0x2010, 0x01),
            "ib": (0x2010, 0x02),
            "ic": (0x2010, 0x03),
            "peak": (0x2010, 0x04),
            "enc_total": (0x2015, 0x01),
            "enc_z": (0x2015, 0x02),
        }
        for key, (did, sub) in labels.items():
            if (did, sub) not in values:
                continue
            value = self.filters.apply_int(f"tele_{key}", values[(did, sub)])
            unit = "count" if did == 0x2015 else "mA"
            self.telemetry_vars[key].set(f"{value} {unit}")

    def _handle_service_result(self, response: object) -> None:
        self.service_result_var.set(response.summary)
        self.service_result_label.configure(foreground="#157A36" if response.ok else "#B3261E")
        if not response.ok:
            return
        key = (response.did, response.sub_id) if response.did is not None else None
        if key == (0x1018, 0x04):
            self.uid_var.set(f"0x{response.value & 0xFFFFFFFF:08X}")
        elif key == (0x3001, 0x01):
            self.baud_read_var.set(str(response.value))
        elif key == (0x6073, 0x00):
            filtered = self.filters.apply_int("max_iq", int(response.value))
            self.max_iq_ma = filtered
            self.max_iq_var.set(f"{filtered} mA")
        elif key == (0x607F, 0x00):
            filtered = self.filters.apply_int("max_speed", int(response.value))
            self.max_speed_rpm = filtered
            self.max_speed_var.set(f"{filtered} rpm")
        elif key in self.param_vars:
            self.param_vars[key].set(str(response.value))
            spec = DID_BY_KEY[key]
            self.param_value_labels[key].set(f"回读 {response.value} {spec.unit}".strip())

    # ------------------------------------------------------------------ 事件泵

    def _handle_startup_event(self, info: object) -> None:
        """启动握手：先确认链路干净，再启动周期轮询与首次回读。"""

        data = info if isinstance(info, dict) else {}
        phase = data.get("phase")
        if phase == "start":
            self._append_log(
                f"INFO 启动握手：等待连续 {data.get('required', 2)} 轮合法 Status1/Status2"
                f"（窗口 {data.get('window_s', 3)} s）"
            )
        elif phase == "probe":
            if data.get("ok"):
                self._append_log(
                    f"INFO 启动握手成功 {data.get('ok_count')}/{data.get('required')}"
                )
            else:
                self._append_log(f"INFO 启动握手未通过：{data.get('detail', '')}")
        elif phase == "done":
            self._append_log(
                f"INFO 启动握手完成：连续 {data.get('ok_count')} 轮，"
                f"用时 {data.get('elapsed_s')} s（{data.get('reason')}）"
            )
            if self._connected_text:
                self.connection_text.set(self._connected_text)
            # 握手通过后再回读只读量程与 UID，避免与启动窗口抢总线。
            self._read_uid()
            self._read_did(0x6073, 0x00)
            self._read_did(0x607F, 0x00)

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.worker.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self._connected_text = f"已连接：{payload}"
                    self.connection_text.set(f"{self._connected_text}（同步中…）")
                    self.connection_label.configure(style="Ok.TLabel")
                    self.connect_button.configure(text="断开")
                    self.motion = None
                    self.device = None
                    self.filters.reset()
                    self._apply_filter_settings()
                    for key, label in STATUS_ROWS:
                        self._set_status(key, "--")
                    for key in self.telemetry_vars:
                        self.telemetry_vars[key].set("--")
                    self._configure_poll()
                elif kind == "startup":
                    self._handle_startup_event(payload)
                elif kind == "resync":
                    mid, discarded, attempt = payload
                    self._append_log(
                        f"INFO 接收流重同步：MID 0x{mid:03X} 丢弃 {discarded} 字节"
                        f"（第 {attempt} 次尝试）"
                    )
                elif kind == "disconnected":
                    self.connected = False
                    self.connection_text.set("未连接")
                    self.connection_label.configure(style="Bad.TLabel")
                    self.connect_button.configure(text="连接")
                    self.cyclic_var.set(False)
                elif kind == "motion_status":
                    self._update_motion(payload)
                elif kind == "device_status":
                    self._update_device(payload)
                elif kind == "telemetry":
                    self._update_telemetry(payload)
                elif kind == "service":
                    self._handle_service_result(payload)
                elif kind == "raw_reply":
                    mid, data = payload
                    self.raw_reply_var.set(f"MID 0x{mid:03X}: {format_hex(data)}")
                    self._append_raw_reply(mid, data)
                elif kind == "disabled":
                    self.enable_var.set(False)
                    self.cyclic_var.set(False)
                    self.error_style_var.set("已下使能")
                    self.error_label.configure(style="Warn.TLabel")
                elif kind == "log":
                    self._append_log(payload)
                elif kind == "notice":
                    self._append_log(f"INFO {payload}")
                elif kind == "warning":
                    self._append_log(f"WARN {payload}")
                elif kind == "error":
                    self._append_log(f"ERROR {payload}")
                    messagebox.showerror("通信错误", payload)
        except queue.Empty:
            pass
        self.after(30, self._drain_events)

    def _on_close(self) -> None:
        if self.connected and self.disable_on_disconnect_var.get():
            self.worker.submit("disconnect", True)
            self.after(80, self._finish_close)
        else:
            self._finish_close()

    def _finish_close(self) -> None:
        self.worker.shutdown()
        self.destroy()


def run() -> None:
    app = MotorHostApp()
    app.mainloop()
