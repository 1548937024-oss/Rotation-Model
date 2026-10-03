# 外置驱动器 MicroDriver 上位机 V2.10 测试用例（协议 V1.4）

对应固件：MicroDriver V2.10，协议 V1.4（2026-10-03）。
设备默认：`460800 8N1`、Node ID `1`、MID 小端。

## A 组：自动化测试（模拟设备，无需硬件）

运行：

```powershell
python -m unittest discover -s tests -v
```

| 编号 | 用例 | 预期 | 覆盖 |
|---|---|---|---|
| A01 | 协议示例帧逐字节校验 | 正文全部示例帧 CRC 一致 | `test_protocol.ProtocolExampleTests` |
| A02 | 应答 CRC 使用 Response MID | `0x181/0x281/0x581` 正确解析，错误 MID 拒绝 | `test_protocol.FramingTests` |
| A03 | 完整零长度询问帧 | `AA 81 01 00 16`、`AA 81 02 00 29` | `test_protocol` |
| A04 | 有符号量纲 | 位置 int16、速度 int8、Iq 峰值百分比 | `test_protocol.SignalTests` |
| A05 | 故障码 3 / 4 | 分别显示 nFAULT 综合与速度失控 | `test_protocol.SignalTests` |
| A06 | 服务响应与 Abort | `0x43/0x60/0x80/0xF1` 解析及 4 种 Abort 码 | `test_protocol.ServiceResponseTests` |
| A07 | DID 目录与符号扩展 | 0x2010 按 S16、0x2015/01 按 S32 | `test_protocol.DidCatalogTests` |
| A08 | 首次使能对齐 | 使能后进入对齐，约 1200 ms 后进入运行 | `test_simulator` |
| A09 | 读只读量程 | Max Iq=1400 mA，Max Speed=18000 rpm | `test_simulator` |
| A10 | 读 `0x6040` | 返回 Abort `0x06010001` | `test_simulator` |
| A11 | 写只读 DID / 越界 | 分别返回 `0x06010002` / `0x06090030` | `test_simulator` |
| A12 | 参数写入与回读 | 写入后回读一致 | `test_simulator` |
| A13 | 保存 / 重新加载 | `0x1010` 保存，改写后 `0x1011` 恢复 | `test_simulator` |
| A14 | 清错流程 | 注入故障→报错→`0xF1 0x12`→无故障 | `test_simulator` / `test_worker` |
| A15 | 设置零位 / 改 Node ID | 均返回 Abort，Node ID 不变 | `test_simulator` |
| A16 | 工作线程控制与状态 | Control1 后 Status1 报出目标速度 | `test_worker` |
| A17 | 工作线程查询默认口径 | 默认发完整零长度帧，短询问为可选项 | `test_worker.WorkerFramingTests` |
| A18 | 扩展遥测 | 读到 0x2010 与 0x2015 各项 | `test_worker` |
| A19 | 脚本解析与执行 | `?`/`??`/delay/注释解析正确并执行 | `test_app` / `test_worker` |
| A20 | 显示滤波 | 关闭直通；开启按 α 平滑；系数夹紧 | `test_app.DisplayFilterTests` |
| A21 | 界面冒烟 | 连接模拟器、回读量程、状态、遥测全通过 | `tools/gui_smoke.py` |
| A22 | 启动握手门控 | 连续两轮合法 Status1+Status2 后才结束握手 | `test_worker.WorkerFramingTests` |
| A23 | 噪声字节重同步 | 前置 `84 00 55` 后仍能 CRC 命中并返回数据 | `test_worker.WorkerFramingTests` |
| A24 | 错误 DLEN 帧跳过 | 收到 Status2 帧后再给 Status1，能跳过不匹配帧 | `test_worker.WorkerFramingTests` |
| A25 | 丢应答自动重试 | 首次无应答、第二次应答可成功；三次都失败才报超时 | `test_worker.WorkerFramingTests` |
| A26 | 服务应答为主动下发 | 延迟 30 ms 的后置应答仍成功，且全程只发 0x601、不发 0x581 查询 | `test_worker.WorkerFramingTests` |
| A27 | 停车服务帧 | 0xF1 0x13/0x14/0x15 三帧逐字节一致 | `test_protocol.StopServiceTests` |
| A28 | 正常/快速停车 | 正常停车先减速再下使能；快速停车立即断开 | `test_simulator` |
| A29 | 急停锁存与恢复 | 0x15 后使能被拒（0xE5），0x12 清错后可重新使能 | `test_simulator` / `test_worker` |
| A30 | 控制帧超时保护 | 超时未收到控制帧 → 受控停车、下使能、bit4 置位 | `test_simulator` |
| A31 | 控制帧续命 | 超时窗口内发一帧 Control1 即保持使能、不触发失联 | `test_simulator` |
| A32 | 安全配置 0x2020 | 读写、越界拒绝、软限位交叉约束、运行中拒写 | `test_simulator` / `test_protocol` |
| A33 | 安全状态 0x2021 | 读到状态机/标志/固件版本 V2.10.3/协议版本 V1.4.0 | `test_simulator` / `test_worker` |
| A34 | 安全量格式化 | 版本 BCD、状态名、标志位文案、停车类型 | `test_protocol.SafetyConfigTests` |

## B 组：实机联调（需硬件）

前置：USB-RS485 转换器 A/B 与驱动板对应，共地；电机处于安全状态。

| 编号 | 步骤 | 预期 |
|---|---|---|
| B01 | 连接 `460800 8N1`，Node 1 | 连接成功，无 CRC 报错 |
| B02 | 发送 `AA 81 01 00 16` | 收到 Status1，使能位与故障位符合实际 |
| B03 | 自动轮询 20 ms，持续 1 分钟 | 无丢帧堆积，电压/温度读数平稳 |
| B04 | 读取 `0x1018/04`、`0x6073/00`、`0x607F/00` | UID 稳定；Max Iq=1400 mA；Max Speed=18000 rpm |
| B05 | 首次点击“上使能” | 电机执行约 1.2 s 吸合对齐，状态可继续查询 |
| B06 | 位置模式发送小幅目标 | 位置按 int16 变化，无方向反转异常 |
| B07 | 速度模式发送 20%、50% | 转速与百分比对应（50% ≈ 9000 rpm） |
| B08 | 直接发送 75%、100% | 可能触发错误码 4，属当前已知限制，不作为通过判据 |
| B09 | 人为触发 nFAULT（或注入故障） | Status1 报 Error=1、ErrorCode=3 |
| B10 | 点击“清错”，等待 ≥10 ms 后重新使能 | 故障清除，可重新使能 |
| B11 | 写入堵转电流 1000 mA 并回读 | 读回 1000 mA |
| B12 | 点击“保存参数到 EEPROM” | 返回写成功 `0x60` |
| B13 | 改写参数后“从 EEPROM 重新加载” | 参数恢复为已保存值，且自动回读 Max Iq / Max Speed |
| B14 | 读取扩展遥测 | 三相电流与编码器累计有合理读数 |
| B15 | 断开连接（勾选断开前下使能） | 发送 Control1 下使能后断开 |
| B16 | 上电瞬间连接，观察前 3 s | 启动窗口内若出现接收错位，日志记录重同步字节数，不弹非法 DLEN 错误 |
| B17 | 连接后立即连续点击查询/读参数 | 发请求前清输入缓冲，不出现上一帧残留串扰 |
| B18 | 读取 UID / Max Iq / Max Speed 各连续 20 次 | 全部返回，无 "read_did 失败"；日志中只出现 0x601 请求与 0x581 应答尾 |
| B19 | 上使能后故意不发任何控制帧 | 约 200 ms 后受控停车并下使能；0x2021/02 bit4 置位 |
| B20 | 点击“急停并锁存”后再点“上使能” | 使能被拒（0xE5）；清错后重新使能成功 |
| B21 | 写 0x2020/01、02 软限位并保存、重载 | 写成功后回读一致；最小/最大顺序错误被拒（0x06090030） |
| B22 | 运行中写 0x2020 | 返回 Abort 0x06010002（仅停机可写） |
| B23 | 读取 0x2021 全部子项 | 状态机/标志/版本/故障快照有合理值，V2.10.3 / V1.4.0 |

## C 组：异常与边界

| 编号 | 用例 | 预期 |
|---|---|---|
| C01 | 拔掉 USB 转换器 | 提示串口异常并断开，可手动重连 |
| C02 | 报文 MID 填 `0x800` | 界面拒绝，提示 MID 范围 0x000~0x7FF |
| C03 | TargetIq 输入 0 | 界面夹紧到 1%，不会误发“默认 1400 mA”语义 |
| C04 | 位置输入 70000 | 夹紧到 32767；输入 -80000 夹紧到 -32768 |
| C05 | 周期设为 0 或 1001 ms | 拒绝并关闭周期发送 |
| C06 | 轮询期间执行长脚本 | 周期控制与轮询继续按各自节拍运行 |
| C07 | 关闭滤波 | 立即显示原始采样，不做平滑 |
