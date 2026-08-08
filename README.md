# 电机模组串口上位机 V1.03

基于所附《电机模组串口通讯协议》制作的 Windows 桌面上位机。程序默认不会在连接时上使能，首次发送上使能命令前会要求人工确认。

> 文件名标注为 V1.2，但 PDF 封面和变更历史中的最新协议版本均写为 V1.1（2026-07-31）。本程序按 PDF 实际内容实现。

## 功能

- 串口参数：8N1，默认 921600 bps，可刷新并选择 COM 口
- Node ID：1~31
- 单控报文：使能、位置/速度模式、目标位置、目标速度、目标 Iq
- 位置、速度和 Iq 支持滑块拖动与精确输入，超出协议量程时自动限幅
- 一次发送或 1~200 ms 周期发送
- 多节点控制页：5 个节点分别设置使能/模式/位置/速度/Iq，支持滑块拖动与精确输入、一键全部使能、独立参数单次与周期下发、逐节点实时监控
- 双次发送“立即下使能”，并默认在断开连接前下使能
- 自动查询状态报文 1/2：使能、模式、位置、速度、Iq、电压、温度、故障，默认 100ms 周期全量刷新
- 服务：服务使能/下使能、切换模式、设置零位、复位故障
- DID：读取 UID/波特率，读写堵转电流和堵转时间，修改波特率
- 根据 UID 修改 Node ID
- 原始 MID/Data 报文发送与查询，多报文脚本调试并集中显示响应
- 完整 TX/RX 十六进制日志
- 内置模拟设备，不接电机也可验证界面和操作流程

## 最快启动

Windows 10/11 安装 Python 3.11 或更高版本后，双击：

```text
setup_and_run.bat
```

脚本会在项目目录建立独立环境、安装 `pyserial` 并启动程序。也可手动运行：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\activate
python -m pip install -r requirements.txt
python main.py
```

先在端口中选择“模拟设备（无需硬件）”，连接后即可测试全部主要功能。

## 生成 Windows EXE

双击：

```text
build_windows.bat
```

构建脚本会依次：

1. 检查 Python 版本；
2. 创建独立构建环境；
3. 安装 PyInstaller 和 pyserial；
4. 运行全部协议与模拟通信测试；
5. 生成单文件、无控制台窗口的 Windows EXE；
6. 计算 SHA-256 并自动打开结果目录。

最终文件：

```text
release\电机模组上位机.exe
release\SHA256.txt
```

EXE 不依赖目标电脑预装 Python。首次构建需要联网下载构建依赖；以后可以离线重复构建。

由于 EXE 没有商业代码签名证书，Windows SmartScreen 可能显示“未知发布者”。这和串口程序是否正确无关，文件完整性可通过同目录 `SHA256.txt` 核对。

## 首次实机联调

1. USB 转串口必须支持 3.3 V TTL 电平和至少 921600 bps；先确认 TX/RX 交叉、GND 共地。
2. 电机先保持下使能，连接默认 Node ID 1。
3. 先点“立即查询”，确认能收到状态 1；再读取 UID。
4. 协议文档明确位置、DID、UID、电压等 Data 多字节值采用小端；但没有明确说明 MID 两字节的字节序。经实机验证，本程序默认 MID 大端，界面仍可切换为小端。
5. 先使用较低的目标速度和 Iq；核对机构实际行程与 `0~65535 count` 的对应关系后，再发送上使能控制。
6. 状态/服务查询采用协议规定的分段时序：主机发送 `AA + MID`，从机补发 `DLEN + Data + CRC`。

## 协议实现摘要

完整发送帧：

```text
Header(AA) + MID(2) + DLEN(1) + Data(n) + CRC8(1)
```

查询响应：

```text
主机：Header(AA) + MID(2)
从机：DLEN(1) + Data(n) + CRC8(1)
```

CRC 参数：

- CRC-8 polynomial: `0x07`
- init: `0x00`
- refin/refout: `false`
- xorout: `0x00`
- 覆盖范围：Header 至 Data 最后一个字节

单控 MCP：

```text
Byte0: bit7 Enable，bit6 reserved，bit[5:0] Mode
Byte1~2: TargetPosition，uint16，小端
Byte3: TargetSpeed，int8
Byte4: TargetIq，int8
```

## 自动测试

在项目目录运行：

```powershell
python -m unittest discover -s tests -v
```

测试覆盖 CRC 固定向量、完整帧、响应尾、符号数、状态解析、DID/UID 编码以及模拟设备端到端交互。

## 当前协议中的不确定项

- MID 为 2 字节且仅低 11 位有效，但 PDF 没有注明 MID 字节序，因此界面可选小端/大端。
- PDF 只写出 CRC8 多项式 `0x07`，没有单列初值、输入/输出反射和最终异或值。程序按标准 CRC-8 参数采用 init=0、refin/refout=false、xorout=0。
- 修改 Node ID 后立即生效，但 PDF 未明确服务响应应使用旧 ID 还是新 ID。程序先查询新 ID，再兼容查询旧 ID。
- 文档未给出服务处理的最短等待时间。程序在服务请求与响应查询之间默认等待 6 ms，设置 Node ID 时等待 10 ms；若目标固件任务周期更慢，可在 `device_worker.py` 中调大。
- 文档没有规定控制报文超时行为。周期发送是否必须、超时后是否自动下使能，取决于电机固件和产品规格书。
