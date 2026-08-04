"""Tkinter desktop UI for the motor module serial host."""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Callable

from device_worker import DeviceWorker
from motor_protocol import (
    DeviceStatus,
    MotionStatus,
    MotorControl,
    ProtocolError,
    RunMode,
    build_motor_enable_service,
    build_read_did,
    build_run_mode_service,
    build_set_node_id_service,
    build_set_zero_service,
    build_write_did,
    format_hex,
    parse_hex_bytes,
)


SIMULATOR_LABEL = "模拟设备（无需硬件）"
COMMON_BAUDRATES = ("2000000", "921600", "460800", "230400", "115200", "57600", "38400", "19200", "9600")
MID_BYTEORDER_BY_LABEL = {"大端": "big", "小端": "little"}
DEFAULT_MID_ORDER_LABEL = "大端"
POSITION_LIMITS = (0, 65535)
SPEED_LIMITS = (-100, 100)
IQ_LIMITS = (-100, 100)


def clamp_control_value(value: int | str, low: int, high: int) -> int:
    """Convert an integer-like value and clamp it to the inclusive limits."""

    parsed = int(str(value).strip(), 10)
    return max(low, min(high, parsed))


class MotorHostApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("电机模组串口上位机 V1.02")
        self.geometry("1180x790")
        self.minsize(1040, 700)
        self.worker = DeviceWorker()
        self.connected = False
        self._enable_confirmed = False
        self.node_motion: dict[int, MotionStatus] = {}
        self.node_device: dict[int, DeviceStatus] = {}
        self._build_style()
        self._build_variables()
        self._build_ui()
        self.refresh_ports()
        self.after(30, self._drain_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_style(self) -> None:
        style = ttk.Style(self)
        available = style.theme_names()
        style.theme_use("vista" if "vista" in available else "clam")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("Value.TLabel", font=("Consolas", 12, "bold"))
        style.configure("Ok.TLabel", foreground="#157A36", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Bad.TLabel", foreground="#B3261E", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Danger.TButton", foreground="#A00000", font=("Microsoft YaHei UI", 10, "bold"))

    def _build_variables(self) -> None:
        self.port_var = tk.StringVar(value=SIMULATOR_LABEL)
        self.baud_var = tk.StringVar(value="2000000")
        self.mid_order_var = tk.StringVar(value=DEFAULT_MID_ORDER_LABEL)
        self.node_var = tk.IntVar(value=1)
        self.node_sel_vars = [tk.BooleanVar(value=(i == 0)) for i in range(5)]
        self.connection_text = tk.StringVar(value="未连接")

        self.enable_var = tk.BooleanVar(value=False)
        self.mode_var = tk.StringVar(value="位置模式")
        self.position_var = tk.StringVar(value="0")
        self.speed_var = tk.StringVar(value="20")
        self.iq_var = tk.StringVar(value="20")
        self.cyclic_var = tk.BooleanVar(value=False)
        self.cyclic_ms_var = tk.IntVar(value=20)
        self.poll_var = tk.BooleanVar(value=True)
        self.poll_ms_var = tk.IntVar(value=100)
        self.disable_on_disconnect_var = tk.BooleanVar(value=True)

        self.mn_enable_vars = [tk.BooleanVar(value=False) for _ in range(5)]
        self.mn_mode_vars = [tk.StringVar(value="位置模式") for _ in range(5)]
        self.mn_position_vars = [tk.StringVar(value="0") for _ in range(5)]
        self.mn_speed_vars = [tk.StringVar(value="20") for _ in range(5)]
        self.mn_iq_vars = [tk.StringVar(value="20") for _ in range(5)]
        self.mn_active_vars = [tk.BooleanVar(value=(i == 0)) for i in range(5)]
        self.mn_cyclic_var = tk.BooleanVar(value=False)
        self.mn_cyclic_ms_var = tk.IntVar(value=20)
        self.multi_status_error = tk.StringVar(value="未查询")

        self.status_enable = tk.StringVar(value="--")
        self.status_mode = tk.StringVar(value="--")
        self.status_position = tk.StringVar(value="--")
        self.status_speed = tk.StringVar(value="--")
        self.status_iq = tk.StringVar(value="--")
        self.status_voltage = tk.StringVar(value="--")
        self.status_temperature = tk.StringVar(value="--")
        self.status_error = tk.StringVar(value="未查询")

        self.uid_var = tk.StringVar(value="--")
        self.baud_read_var = tk.StringVar(value="--")
        self.new_baud_var = tk.StringVar(value="2000000")
        self.stall_current_var = tk.StringVar(value="1200")
        self.stall_time_var = tk.StringVar(value="500")
        self.new_node_var = tk.IntVar(value=2)
        self.service_result_var = tk.StringVar(value="尚无服务操作")
        self.raw_mid_var = tk.StringVar(value="0x201")
        self.raw_data_var = tk.StringVar(value="80 00 00 00 00")
        self.raw_reply_var = tk.StringVar(value="--")

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        title_row = ttk.Frame(root)
        title_row.pack(fill="x", pady=(0, 8))
        ttk.Label(title_row, text="电机模组串口上位机", style="Title.TLabel").pack(side="left")
        self.connection_label = ttk.Label(
            title_row, textvariable=self.connection_text, style="Bad.TLabel"
        )
        self.connection_label.pack(side="right", padx=8)

        self._build_connection_bar(root)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, pady=(8, 0))
        control_tab = ttk.Frame(notebook, padding=10)
        multi_tab = ttk.Frame(notebook, padding=10)
        service_tab = ttk.Frame(notebook, padding=10)
        raw_tab = ttk.Frame(notebook, padding=10)
        notebook.add(control_tab, text="控制与状态")
        notebook.add(multi_tab, text="多节点控制")
        notebook.add(service_tab, text="参数与服务")
        notebook.add(raw_tab, text="报文调试")
        self._build_control_tab(control_tab)
        self._build_multi_tab(multi_tab)
        self._build_service_tab(service_tab)
        self._build_raw_tab(raw_tab)

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="串口连接", padding=8)
        box.pack(fill="x")
        ttk.Label(box, text="端口").grid(row=0, column=0, padx=4, pady=3)
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=27, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(box, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Label(box, text="波特率").grid(row=0, column=3, padx=(14, 4))
        ttk.Combobox(
            box, textvariable=self.baud_var, values=COMMON_BAUDRATES, width=10
        ).grid(row=0, column=4, padx=4)
        ttk.Label(box, text="MID 字节序").grid(row=0, column=5, padx=(14, 4))
        ttk.Combobox(
            box,
            textvariable=self.mid_order_var,
            values=tuple(MID_BYTEORDER_BY_LABEL),
            width=8,
            state="readonly",
        ).grid(row=0, column=6, padx=4)
        ttk.Label(box, text="Node ID").grid(row=0, column=7, padx=(14, 4))
        ttk.Spinbox(box, from_=1, to=31, textvariable=self.node_var, width=5).grid(
            row=0, column=8, padx=4
        )
        ttk.Button(box, text="应用 ID", command=self._apply_node).grid(row=0, column=9, padx=3)
        self.connect_button = ttk.Button(box, text="连接", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=10, padx=(16, 4))
        ttk.Checkbutton(
            box, text="断开前下使能", variable=self.disable_on_disconnect_var
        ).grid(row=0, column=11, padx=4)

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
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=5, pady=7)
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
            length=250,
            command=self._sync_cyclic,
        ).grid(row=row, column=1, sticky="ew", padx=5)
        entry = ttk.Entry(parent, textvariable=variable, width=8, justify="right")
        entry.grid(row=row, column=2, sticky="w", padx=(4, 2))

        def clamp_entry(_event: tk.Event | None = None) -> None:
            self._clamp_control_variable(variable, low, high)

        entry.bind("<FocusOut>", clamp_entry)
        entry.bind("<Return>", clamp_entry)
        entry.bind("<KeyRelease>", lambda _event: self._sync_cyclic())
        ttk.Label(parent, text=f"{unit}  [{low}~{high}]").grid(
            row=row, column=3, sticky="w", padx=(2, 5)
        )

    def _build_control_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)

        control = ttk.LabelFrame(tab, text="控制（MSG_Control1 / MSG_GroupControl1）", padding=10)
        control.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        ttk.Checkbutton(
            control, text="电机使能", variable=self.enable_var, command=self._sync_cyclic
        ).grid(
            row=0, column=0, sticky="w", padx=5, pady=7
        )
        ttk.Label(control, text="运行模式").grid(row=1, column=0, sticky="w", padx=5, pady=7)
        mode_combo = ttk.Combobox(
            control,
            textvariable=self.mode_var,
            values=("位置模式", "速度模式"),
            state="readonly",
            width=12,
        )
        mode_combo.grid(row=1, column=1, sticky="w", padx=5)
        mode_combo.bind("<<ComboboxSelected>>", lambda _event: self._sync_cyclic())
        self._labeled_slider(
            control, 2, "目标位置", self.position_var, *POSITION_LIMITS, "count / 100%"
        )
        self._labeled_slider(
            control, 3, "目标速度", self.speed_var, *SPEED_LIMITS, "% 额定转速"
        )
        self._labeled_slider(
            control, 4, "目标 Iq", self.iq_var, *IQ_LIMITS, "% 额定 Iq"
        )

        buttons = ttk.Frame(control)
        buttons.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(8, 3))
        ttk.Button(buttons, text="发送一次", command=self._send_control).pack(
            side="left", padx=4
        )
        ttk.Button(
            buttons, text="立即下使能", style="Danger.TButton", command=self._emergency_disable
        ).pack(side="left", padx=4)
        ttk.Checkbutton(
            buttons, text="周期发送", variable=self.cyclic_var, command=self._configure_cyclic
        ).pack(side="left", padx=(20, 3))
        ttk.Spinbox(
            buttons,
            from_=1,
            to=200,
            textvariable=self.cyclic_ms_var,
            width=7,
            command=self._sync_cyclic,
        ).pack(side="left", padx=3)
        ttk.Label(buttons, text="ms").pack(side="left")

        node_sel = ttk.LabelFrame(control, text="目标节点（最多 5 个）", padding=6)
        node_sel.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        for i in range(5):
            ttk.Checkbutton(
                node_sel,
                text=f"Node {i + 1}",
                variable=self.node_sel_vars[i],
                command=self._sync_cyclic,
            ).pack(side="left", padx=6)

        status = ttk.LabelFrame(tab, text="多节点实时状态", padding=10)
        status.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 8))
        status.columnconfigure(0, weight=1)
        status.rowconfigure(0, weight=1)
        status_columns = ("node", "enable", "mode", "position", "speed", "iq", "voltage", "temp", "error")
        status_headers = {
            "node": "Node",
            "enable": "使能",
            "mode": "模式",
            "position": "位置",
            "speed": "速度",
            "iq": "Iq",
            "voltage": "电压",
            "temp": "温度",
            "error": "故障",
        }
        status_widths = {
            "node": 45,
            "enable": 55,
            "mode": 55,
            "position": 65,
            "speed": 50,
            "iq": 45,
            "voltage": 70,
            "temp": 55,
            "error": 80,
        }
        self.status_table = ttk.Treeview(
            status, columns=status_columns, show="headings", height=6
        )
        for column in status_columns:
            self.status_table.heading(column, text=status_headers[column])
            self.status_table.column(
                column, width=status_widths[column], anchor="center", stretch=True
            )
        self.status_table.grid(row=0, column=0, sticky="nsew")
        self._reset_status_table()

        self.error_label = ttk.Label(status, textvariable=self.status_error, style="Bad.TLabel")
        self.error_label.grid(row=1, column=0, sticky="w", padx=5, pady=6)

        polling = ttk.Frame(status)
        polling.grid(row=2, column=0, sticky="ew", pady=(7, 0))
        ttk.Checkbutton(
            polling, text="自动查询", variable=self.poll_var, command=self._configure_poll
        ).pack(side="left", padx=4)
        ttk.Spinbox(
            polling, from_=20, to=5000, textvariable=self.poll_ms_var, width=7
        ).pack(side="left", padx=3)
        ttk.Label(polling, text="ms").pack(side="left")
        ttk.Button(polling, text="立即查询", command=self._query_status).pack(
            side="left", padx=12
        )

        log_box = ttk.LabelFrame(tab, text="通信日志", padding=6)
        log_box.grid(row=1, column=0, columnspan=2, sticky="nsew")
        log_box.rowconfigure(0, weight=1)
        log_box.columnconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(
            log_box, height=14, font=("Consolas", 9), wrap="none", state="disabled"
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        ttk.Button(log_box, text="清空日志", command=self._clear_log).grid(
            row=1, column=0, sticky="e", pady=(5, 0)
        )

    def _build_multi_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(1, weight=1)

        params = ttk.LabelFrame(tab, text="节点参数（每个节点独立设置）", padding=10)
        params.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        headers = ("节点", "使能", "运行模式", "目标位置", "目标速度", "目标 Iq", "参与下发")
        for col, text in enumerate(headers):
            ttk.Label(params, text=text, style="Section.TLabel").grid(
                row=0, column=col, padx=6, pady=(0, 6)
            )
        for i in range(5):
            row = i + 1
            ttk.Label(params, text=f"Node {i + 1}").grid(
                row=row, column=0, padx=6, pady=4
            )
            ttk.Checkbutton(
                params, variable=self.mn_enable_vars[i], command=self._sync_multi_cyclic
            ).grid(row=row, column=1, padx=6)
            mode_combo = ttk.Combobox(
                params,
                textvariable=self.mn_mode_vars[i],
                values=("位置模式", "速度模式"),
                state="readonly",
                width=10,
            )
            mode_combo.grid(row=row, column=2, padx=6)
            mode_combo.bind(
                "<<ComboboxSelected>>", lambda _event, idx=i: self._sync_multi_cyclic()
            )
            position_entry = ttk.Entry(
                params, textvariable=self.mn_position_vars[i], width=8, justify="right"
            )
            position_entry.grid(row=row, column=3, padx=6)
            speed_entry = ttk.Entry(
                params, textvariable=self.mn_speed_vars[i], width=6, justify="right"
            )
            speed_entry.grid(row=row, column=4, padx=6)
            iq_entry = ttk.Entry(
                params, textvariable=self.mn_iq_vars[i], width=6, justify="right"
            )
            iq_entry.grid(row=row, column=5, padx=6)
            for entry in (position_entry, speed_entry, iq_entry):
                entry.bind("<Return>", lambda _event, idx=i: self._sync_multi_cyclic())
            ttk.Checkbutton(
                params, variable=self.mn_active_vars[i], command=self._sync_multi_cyclic
            ).grid(row=row, column=6, padx=6)
        ttk.Label(
            params,
            text="参与下发的节点需从 Node 1 开始连续勾选；未参与的节点不会收到控制报文。",
            foreground="#555555",
        ).grid(row=6, column=0, columnspan=7, sticky="w", padx=6, pady=(8, 0))

        actions = ttk.Frame(tab)
        actions.grid(row=1, column=0, sticky="ew", padx=(0, 5), pady=(0, 8))
        ttk.Button(actions, text="发送一次", command=self._multi_send_once).pack(
            side="left", padx=4
        )
        ttk.Button(
            actions,
            text="立即下使能",
            style="Danger.TButton",
            command=self._multi_emergency_disable,
        ).pack(side="left", padx=4)
        ttk.Button(actions, text="全选 1~5", command=self._multi_select_all).pack(
            side="left", padx=8
        )
        ttk.Button(actions, text="仅 Node 1", command=self._multi_select_first).pack(
            side="left", padx=4
        )
        ttk.Checkbutton(
            actions,
            text="周期发送",
            variable=self.mn_cyclic_var,
            command=self._configure_multi_cyclic,
        ).pack(side="left", padx=(20, 3))
        ttk.Spinbox(
            actions,
            from_=1,
            to=200,
            textvariable=self.mn_cyclic_ms_var,
            width=7,
            command=self._sync_multi_cyclic,
        ).pack(side="left", padx=3)
        ttk.Label(actions, text="ms").pack(side="left")

        monitor = ttk.LabelFrame(tab, text="多节点实时监控", padding=10)
        monitor.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0), pady=(0, 8))
        monitor.columnconfigure(0, weight=1)
        monitor.rowconfigure(0, weight=1)
        columns = ("node", "enable", "mode", "position", "speed", "iq", "voltage", "temp", "error")
        headers = {
            "node": "Node",
            "enable": "使能",
            "mode": "模式",
            "position": "位置",
            "speed": "速度",
            "iq": "Iq",
            "voltage": "电压",
            "temp": "温度",
            "error": "故障",
        }
        widths = {
            "node": 45,
            "enable": 55,
            "mode": 55,
            "position": 65,
            "speed": 50,
            "iq": 45,
            "voltage": 70,
            "temp": 55,
            "error": 80,
        }
        self.multi_status_table = ttk.Treeview(
            monitor, columns=columns, show="headings", height=7
        )
        for column in columns:
            self.multi_status_table.heading(column, text=headers[column])
            self.multi_status_table.column(
                column, width=widths[column], anchor="center", stretch=True
            )
        self.multi_status_table.grid(row=0, column=0, sticky="nsew")
        self.multi_error_label = ttk.Label(
            monitor, textvariable=self.multi_status_error, style="Bad.TLabel"
        )
        self.multi_error_label.grid(row=1, column=0, sticky="w", padx=5, pady=6)
        polling = ttk.Frame(monitor)
        polling.grid(row=2, column=0, sticky="ew", pady=(7, 0))
        ttk.Checkbutton(
            polling, text="自动查询", variable=self.poll_var, command=self._configure_multi_poll
        ).pack(side="left", padx=4)
        ttk.Spinbox(
            polling, from_=20, to=5000, textvariable=self.poll_ms_var, width=7
        ).pack(side="left", padx=3)
        ttk.Label(polling, text="ms").pack(side="left")
        ttk.Button(polling, text="立即查询", command=self._query_status).pack(
            side="left", padx=12
        )
        self._reset_status_table()

    def _service_row(
        self,
        parent: ttk.Frame,
        row: int,
        title: str,
        read_command: Callable[[], None] | None = None,
        variable: tk.StringVar | None = None,
        write_command: Callable[[], None] | None = None,
        unit: str = "",
    ) -> None:
        ttk.Label(parent, text=title).grid(row=row, column=0, sticky="w", padx=5, pady=7)
        if read_command:
            ttk.Button(parent, text="读取", command=read_command).grid(row=row, column=1, padx=4)
        if variable:
            ttk.Entry(parent, textvariable=variable, width=18).grid(row=row, column=2, padx=4)
        if unit:
            ttk.Label(parent, text=unit).grid(row=row, column=3, sticky="w", padx=3)
        if write_command:
            ttk.Button(parent, text="写入", command=write_command).grid(row=row, column=4, padx=4)

    def _build_service_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        common = ttk.LabelFrame(tab, text="常用服务", padding=10)
        common.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        ttk.Label(
            common, text="运行模式只允许在电机下使能时切换。", foreground="#9A5A00"
        ).grid(row=0, column=0, columnspan=3, sticky="w", padx=5, pady=(0, 10))
        ttk.Button(
            common,
            text="服务上使能",
            command=lambda: self._submit_service(build_motor_enable_service(True), "服务上使能", True),
        ).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(
            common,
            text="服务下使能",
            command=lambda: self._submit_service(build_motor_enable_service(False), "服务下使能"),
        ).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        ttk.Button(
            common,
            text="设置位置模式",
            command=lambda: self._submit_service(build_run_mode_service(0), "设置位置模式"),
        ).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(
            common,
            text="设置速度模式",
            command=lambda: self._submit_service(build_run_mode_service(1), "设置速度模式"),
        ).grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        ttk.Button(
            common, text="当前位置设为零位", command=self._set_zero
        ).grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(
            common,
            text="复位全部故障",
            command=lambda: self._submit_service(build_write_did(0x6040, 0, 0x0080), "复位故障"),
        ).grid(row=3, column=1, padx=5, pady=5, sticky="ew")

        params = ttk.LabelFrame(tab, text="参数读写（DID）", padding=10)
        params.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 8))
        self._service_row(params, 0, "设备 UID", self._read_uid, self.uid_var)
        self._service_row(params, 1, "当前波特率", self._read_baud, self.baud_read_var)
        self._service_row(
            params, 2, "新波特率", None, self.new_baud_var, self._write_baud, "bps"
        )
        self._service_row(
            params,
            3,
            "堵转电流",
            lambda: self._read_did(0x6073, 1),
            self.stall_current_var,
            lambda: self._write_int_did(0x6073, 1, self.stall_current_var, "堵转电流"),
            "mA",
        )
        self._service_row(
            params,
            4,
            "堵转时间",
            lambda: self._read_did(0x6073, 3),
            self.stall_time_var,
            lambda: self._write_int_did(0x6073, 3, self.stall_time_var, "堵转时间"),
            "ms",
        )

        node = ttk.LabelFrame(tab, text="按 UID 修改 Node ID", padding=10)
        node.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        ttk.Label(node, text="UID（十六进制或十进制）").grid(
            row=0, column=0, sticky="w", padx=5, pady=6
        )
        ttk.Entry(node, textvariable=self.uid_var, width=20).grid(row=0, column=1, padx=5)
        ttk.Label(node, text="新 Node ID").grid(row=1, column=0, sticky="w", padx=5, pady=6)
        ttk.Spinbox(node, from_=1, to=31, textvariable=self.new_node_var, width=8).grid(
            row=1, column=1, sticky="w", padx=5
        )
        ttk.Button(node, text="设置 Node ID", command=self._set_node_id).grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=8
        )
        ttk.Label(
            node,
            text="仅可在下使能状态设置；成功后需使用新 ID 上使能，才能持久化。",
            wraplength=420,
            foreground="#9A5A00",
        ).grid(row=3, column=0, columnspan=2, sticky="w", padx=5)

        result = ttk.LabelFrame(tab, text="最近一次服务结果", padding=10)
        result.grid(row=1, column=1, sticky="nsew", padx=(5, 0), pady=(0, 8))
        self.service_result_label = ttk.Label(
            result, textvariable=self.service_result_var, wraplength=470
        )
        self.service_result_label.pack(fill="both", expand=True)

        notes = ttk.LabelFrame(tab, text="协议注意事项", padding=10)
        notes.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(
            notes,
            text=(
                "1. 默认串口：8 数据位、1 停止位、无校验、921600 bps。\n"
                "2. 写入波特率后设备立即切换，上位机也会自动切换；新参数需再次上使能才写入 Flash。\n"
                "3. 文档明确 Data 多字节值采用小端；MID 字节序默认为大端，仍可在连接栏切换。"
            ),
            justify="left",
        ).pack(anchor="w")

    def _build_raw_tab(self, tab: ttk.Frame) -> None:
        frame = ttk.LabelFrame(tab, text="原始报文", padding=12)
        frame.pack(fill="x")
        ttk.Label(frame, text="MID").grid(row=0, column=0, sticky="w", padx=5, pady=6)
        ttk.Entry(frame, textvariable=self.raw_mid_var, width=15).grid(row=0, column=1, padx=5)
        ttk.Label(frame, text="Data（十六进制）").grid(
            row=1, column=0, sticky="w", padx=5, pady=6
        )
        ttk.Entry(frame, textvariable=self.raw_data_var, width=70).grid(
            row=1, column=1, columnspan=4, sticky="ew", padx=5
        )
        ttk.Button(frame, text="发送完整帧", command=self._send_raw_frame).grid(
            row=2, column=1, sticky="w", padx=5, pady=8
        )
        ttk.Button(frame, text="只发送查询头", command=self._send_raw_query).grid(
            row=2, column=2, sticky="w", padx=5, pady=8
        )
        ttk.Label(frame, text="响应 Data").grid(row=3, column=0, sticky="w", padx=5, pady=6)
        ttk.Entry(frame, textvariable=self.raw_reply_var, width=70, state="readonly").grid(
            row=3, column=1, columnspan=4, sticky="ew", padx=5
        )
        frame.columnconfigure(4, weight=1)
        ttk.Label(
            tab,
            text=(
                "完整帧由软件自动添加 0xAA、MID、DLEN 与 CRC8。查询功能按协议只发送 "
                "Header + MID，然后把从机返回的 DLEN + Data + CRC 解析为 Data。"
            ),
            wraplength=1000,
            foreground="#555555",
        ).pack(anchor="w", pady=12)

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
        return "__SIMULATOR__" if text == SIMULATOR_LABEL else text.split(" — ", 1)[0]

    def _validate_node(self) -> int:
        node = int(self.node_var.get())
        if not 1 <= node <= 31:
            raise ProtocolError("Node ID 必须为 1~31")
        return node

    def _selected_nodes(self) -> tuple[int, ...]:
        nodes = tuple(
            i + 1 for i, var in enumerate(self.node_sel_vars) if var.get()
        )
        return nodes if nodes else (self._validate_node(),)

    @staticmethod
    def _check_contiguous_nodes(nodes: tuple[int, ...]) -> tuple[int, ...]:
        if len(nodes) > 1:
            expected = tuple(range(1, max(nodes) + 1))
            if nodes != expected:
                raise ProtocolError("多节点控制需从 Node 1 开始连续勾选（如 1、2、3）")
        return nodes

    def _validate_selected_nodes(self) -> tuple[int, ...]:
        return self._check_contiguous_nodes(self._selected_nodes())

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
            node = self._validate_node()
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("参数错误", str(exc) or "波特率必须为正整数")
            return
        byteorder = MID_BYTEORDER_BY_LABEL.get(self.mid_order_var.get(), "big")
        self.connection_text.set("连接中…")
        self.worker.submit("connect", self._selected_port(), baud, byteorder, node)

    def _apply_node(self) -> None:
        try:
            node = self._validate_node()
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("Node ID 错误", str(exc))
            return
        self.worker.submit("set_node", node)

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
            "发送后电机可能立即运动。请确认机构活动范围安全、目标位置/速度/Iq 已核对。",
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
            nodes = self._validate_selected_nodes()
        except (ValueError, tk.TclError, ProtocolError) as exc:
            messagebox.showerror("控制参数错误", str(exc))
            return
        if self._confirm_enable(control.enable):
            self.worker.submit("send_control", control, nodes)

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
            nodes = self._validate_selected_nodes()
            if not 1 <= interval <= 200:
                raise ProtocolError("周期必须为 1~200 ms")
        except (ValueError, tk.TclError, ProtocolError) as exc:
            self.cyclic_var.set(False)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        if self.cyclic_var.get() and not self._confirm_enable(control.enable):
            self.cyclic_var.set(False)
            return
        self.worker.submit(
            "configure_cyclic",
            control,
            self.cyclic_var.get(),
            interval / 1000,
            nodes,
        )

    def _sync_cyclic(self, _value: object | None = None) -> None:
        if not (self.connected and self.cyclic_var.get()):
            return
        try:
            control = self._control_value()
            control.pack()
            interval = int(self.cyclic_ms_var.get())
            if not 1 <= interval <= 200:
                raise ProtocolError("周期必须为 1~200 ms")
            nodes = self._validate_selected_nodes()
        except (ValueError, tk.TclError):
            return
        except ProtocolError as exc:
            self.cyclic_var.set(False)
            try:
                nodes = self._selected_nodes()
            except ProtocolError:
                nodes = (1,)
            self.worker.submit("configure_cyclic", control, False, 0.02, nodes)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        self.worker.submit("configure_cyclic", control, True, interval / 1000, nodes)

    def _multi_nodes(self) -> tuple[int, ...]:
        nodes = tuple(i + 1 for i, var in enumerate(self.mn_active_vars) if var.get())
        if not nodes:
            raise ProtocolError("多节点控制至少需要勾选一个节点")
        return self._check_contiguous_nodes(nodes)

    def _multi_controls(self) -> dict[int, MotorControl]:
        nodes = self._multi_nodes()
        controls: dict[int, MotorControl] = {}
        for i in range(5):
            node = i + 1
            if node not in nodes:
                continue
            mode = RunMode.POSITION if self.mn_mode_vars[i].get() == "位置模式" else RunMode.SPEED
            position = self._clamp_control_variable(self.mn_position_vars[i], *POSITION_LIMITS)
            speed = self._clamp_control_variable(self.mn_speed_vars[i], *SPEED_LIMITS)
            iq = self._clamp_control_variable(self.mn_iq_vars[i], *IQ_LIMITS)
            control = MotorControl(
                bool(self.mn_enable_vars[i].get()), int(mode), position, speed, iq
            )
            control.pack()
            controls[node] = control
        return controls

    def _multi_select_all(self) -> None:
        for var in self.mn_active_vars:
            var.set(True)
        self._sync_multi_cyclic()

    def _multi_select_first(self) -> None:
        for i, var in enumerate(self.mn_active_vars):
            var.set(i == 0)
        self._sync_multi_cyclic()

    def _multi_send_once(self) -> None:
        if not self._require_connected():
            return
        try:
            controls = self._multi_controls()
        except (ValueError, tk.TclError, ProtocolError) as exc:
            messagebox.showerror("多节点控制参数错误", str(exc))
            return
        if any(control.enable for control in controls.values()) and not self._confirm_enable(
            True
        ):
            return
        self.worker.submit("send_controls", controls)

    def _multi_emergency_disable(self) -> None:
        if not self._require_connected():
            return
        self.mn_cyclic_var.set(False)
        for var in self.mn_enable_vars:
            var.set(False)
        self.worker.submit("emergency_disable")

    def _configure_multi_cyclic(self) -> None:
        if not self._require_connected():
            self.mn_cyclic_var.set(False)
            return
        try:
            controls = self._multi_controls()
            interval = int(self.mn_cyclic_ms_var.get())
            if not 1 <= interval <= 200:
                raise ProtocolError("周期必须为 1~200 ms")
        except (ValueError, tk.TclError, ProtocolError) as exc:
            self.mn_cyclic_var.set(False)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        if self.mn_cyclic_var.get() and any(
            control.enable for control in controls.values()
        ) and not self._confirm_enable(True):
            self.mn_cyclic_var.set(False)
            return
        self.worker.submit(
            "configure_cyclic_group", controls, self.mn_cyclic_var.get(), interval / 1000
        )

    def _sync_multi_cyclic(self, _value: object | None = None) -> None:
        if not (self.connected and self.mn_cyclic_var.get()):
            return
        try:
            controls = self._multi_controls()
            interval = int(self.mn_cyclic_ms_var.get())
            if not 1 <= interval <= 200:
                raise ProtocolError("周期必须为 1~200 ms")
        except (ValueError, tk.TclError):
            return
        except ProtocolError as exc:
            self.mn_cyclic_var.set(False)
            messagebox.showerror("周期控制参数错误", str(exc))
            return
        self.worker.submit("configure_cyclic_group", controls, True, interval / 1000)

    def _configure_multi_poll(self) -> None:
        if not self._require_connected():
            self.poll_var.set(False)
            return
        try:
            nodes = self._multi_nodes()
        except ProtocolError as exc:
            self.poll_var.set(False)
            messagebox.showerror("多节点查询错误", str(exc))
            return
        self._configure_poll(nodes=nodes)

    def _configure_poll(self, nodes: tuple[int, ...] | None = None) -> None:
        if not self._require_connected():
            self.poll_var.set(False)
            return
        try:
            interval = int(self.poll_ms_var.get())
            if not 20 <= interval <= 5000:
                raise ProtocolError("查询周期必须为 20~5000 ms")
            selected = self._selected_nodes() if nodes is None else nodes
        except (ValueError, tk.TclError, ProtocolError) as exc:
            self.poll_var.set(False)
            messagebox.showerror("查询周期错误", str(exc))
            return
        self.worker.submit(
            "configure_poll", self.poll_var.get(), interval / 1000, selected
        )

    def _query_status(self) -> None:
        if self._require_connected():
            self.worker.submit("query_status")

    def _submit_service(self, payload: bytes, note: str, enabling: bool = False) -> None:
        if not self._require_connected():
            return
        if enabling and not self._confirm_enable(True):
            return
        self.worker.submit("service", payload, note)

    def _set_zero(self) -> None:
        if not self._require_connected():
            return
        if messagebox.askyesno("确认设置零位", "将把电机当前位置定义为 0。是否继续？"):
            self.worker.submit("service", build_set_zero_service(), "设置当前位置为零位")

    def _read_did(self, did: int, sub_id: int) -> None:
        self._submit_service(build_read_did(did, sub_id), f"读取 DID 0x{did:04X}/{sub_id}")

    def _read_uid(self) -> None:
        self._read_did(0x1018, 4)

    def _read_baud(self) -> None:
        self._read_did(0x3001, 1)

    def _write_int_did(
        self, did: int, sub_id: int, variable: tk.StringVar, label: str
    ) -> None:
        try:
            value = int(variable.get(), 0)
            payload = build_write_did(did, sub_id, value)
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("参数错误", f"{label}必须是合法的非负整数。\n{exc}")
            return
        if messagebox.askyesno("确认写入", f"确认写入{label}：{value}？"):
            self._submit_service(payload, f"写入{label}")

    def _write_baud(self) -> None:
        if not self._require_connected():
            return
        try:
            baud = int(self.new_baud_var.get(), 0)
            payload = build_write_did(0x3001, 1, baud)
            if not 1200 <= baud <= 10_000_000:
                raise ProtocolError("波特率应位于 1200~10000000")
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("波特率错误", str(exc))
            return
        if messagebox.askyesno(
            "确认切换波特率",
            f"设备和上位机将立即切换到 {baud} bps。参数需随后上使能才会写入 Flash。是否继续？",
        ):
            self.worker.submit("change_baud", payload, baud)

    @staticmethod
    def _parse_int_text(text: str) -> int:
        text = text.strip()
        if text == "--":
            raise ValueError("请先读取 UID，或手工输入 UID")
        return int(text, 0)

    def _set_node_id(self) -> None:
        if not self._require_connected():
            return
        try:
            uid = self._parse_int_text(self.uid_var.get())
            new_node = int(self.new_node_var.get())
            payload = build_set_node_id_service(new_node, uid)
        except (ValueError, ProtocolError) as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        if messagebox.askyesno(
            "确认修改 Node ID",
            f"将当前设备 Node ID 修改为 {new_node}。电机必须处于下使能状态。是否继续？",
        ):
            self.worker.submit("set_node_by_uid", payload, new_node)

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
            data = parse_hex_bytes(self.raw_data_var.get())
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
        self.worker.submit("raw_query", mid)

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

    def _reset_status_table(self) -> None:
        self.node_motion.clear()
        self.node_device.clear()
        for i in range(5):
            iid = str(i + 1)
            values = (i + 1, "--", "--", "--", "--", "--", "--", "--", "--")
            self._apply_status_row(self.status_table, iid, values)
            if getattr(self, "multi_status_table", None) is not None:
                self._apply_status_row(self.multi_status_table, iid, values)
        if getattr(self, "multi_error_label", None) is not None:
            self.multi_status_error.set("未查询")

    @staticmethod
    def _apply_status_row(table: ttk.Treeview, iid: str, values: tuple) -> None:
        if not table.exists(iid):
            table.insert("", "end", iid=iid, values=values)
        else:
            table.item(iid, values=values)

    def _render_node_status(self, node_id: int) -> None:
        if not 1 <= node_id <= 5:
            return
        motion = self.node_motion.get(node_id)
        if motion is None:
            return
        device = self.node_device.get(node_id)
        if motion.error:
            mode_text = "--"
            error_text = f"故障 {motion.error_code}：{motion.error_text}"
            label_style = "Bad.TLabel"
        else:
            mode_names = {0: "位置", 1: "速度"}
            mode_text = mode_names.get(motion.mode, f"模式 {motion.mode}")
            error_text = "无故障"
            label_style = "Ok.TLabel"
        self.status_error.set(f"节点 {node_id}：{error_text}")
        self.error_label.configure(style=label_style)
        values = (
            node_id,
            "已使能" if motion.enabled else "未使能",
            mode_text,
            f"{motion.actual_position}",
            f"{motion.actual_speed}",
            f"{motion.actual_iq}",
            f"{device.voltage_mv / 1000:.3f} V" if device else "--",
            f"{device.temperature_c} °C" if device else "--",
            error_text,
        )
        self._apply_status_row(self.status_table, str(node_id), values)
        if getattr(self, "multi_status_table", None) is not None:
            self.multi_status_error.set(f"节点 {node_id}：{error_text}")
            self.multi_error_label.configure(style=label_style)
            self._apply_status_row(self.multi_status_table, str(node_id), values)

    def _update_motion(self, node_id: int, status: MotionStatus) -> None:
        self.node_motion[node_id] = status
        self._render_node_status(node_id)

    def _update_device(self, node_id: int, status: DeviceStatus) -> None:
        self.node_device[node_id] = status
        self._render_node_status(node_id)

    def _handle_service_result(self, response: object) -> None:
        self.service_result_var.set(response.summary)
        self.service_result_label.configure(
            foreground="#157A36" if response.ok else "#B3261E"
        )
        if response.did == 0x1018 and response.sub_id == 4 and response.ok:
            self.uid_var.set(f"0x{response.value:08X}")
        elif response.did == 0x3001 and response.sub_id == 1 and response.ok:
            self.baud_read_var.set(str(response.value))
        elif response.did == 0x6073 and response.sub_id == 1 and response.ok:
            self.stall_current_var.set(str(response.value))
        elif response.did == 0x6073 and response.sub_id == 3 and response.ok:
            self.stall_time_var.set(str(response.value))

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.worker.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.connection_text.set(f"已连接：{payload}")
                    self.connection_label.configure(style="Ok.TLabel")
                    self.connect_button.configure(text="断开")
                    self._reset_status_table()
                    self._configure_poll()
                elif kind == "disconnected":
                    self.connected = False
                    self.connection_text.set("未连接")
                    self.connection_label.configure(style="Bad.TLabel")
                    self.connect_button.configure(text="连接")
                    self.cyclic_var.set(False)
                    self.mn_cyclic_var.set(False)
                    self._reset_status_table()
                elif kind == "multi_motion":
                    node_id, motion = payload
                    self._update_motion(node_id, motion)
                elif kind == "multi_device":
                    node_id, device = payload
                    self._update_device(node_id, device)
                elif kind == "motion_status":
                    self._update_motion(self.node_var.get(), payload)
                elif kind == "device_status":
                    self._update_device(self.node_var.get(), payload)
                elif kind == "service":
                    self._handle_service_result(payload)
                elif kind == "raw_reply":
                    mid, data = payload
                    self.raw_reply_var.set(f"MID 0x{mid:03X}: {format_hex(data)}")
                elif kind == "baud_changed":
                    self.baud_var.set(str(payload))
                    self.baud_read_var.set(str(payload))
                elif kind == "node_changed":
                    self.node_var.set(int(payload))
                    self._append_log(f"INFO 当前通信 Node ID 已切换为 {payload}")
                elif kind == "disabled":
                    self.enable_var.set(False)
                    self.cyclic_var.set(False)
                    self.mn_cyclic_var.set(False)
                    for var in self.mn_enable_vars:
                        var.set(False)
                elif kind == "log":
                    self._append_log(payload)
                elif kind == "notice":
                    self._append_log(f"INFO {payload}")
                elif kind == "warning":
                    self._append_log(f"WARN {payload}")
                    self.connection_text.set(f"警告：{payload}")
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
