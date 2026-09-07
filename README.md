# 阶段二：云台板 F405 + GM6020（CAN）+ FreeRTOS

> 目标：用 CAN 驱动 DJI GM6020，实现速度环、位置环（双环）、多圈角度、PID 库与在线调参。
> 工程：`MDK-ARM/gimbal.uvprojx`（Keil MDK，ARMCC5，0 Error / 0 Warning）
> 理论笔记与实施计划：见上级仓库 `docs/阶段2-理论笔记-CAN与GM6020.md`、`docs/阶段2-实施计划.md`

---

## 一、接线

| 云台板 | GM6020 | 说明 |
|---|---|---|
| PA11 (CAN1_RX) / PA12 (CAN1_TX) | CAN 线 | 板载收发器已接好 |
| CANH / CANL | 红 / 黑 | 双绞线，两端各 120Ω（电机侧用**拨码第 4 位**接入） |
| PC12 (UART5_TX) / PD2 (UART5_RX) | USB-TTL | 115200，接 VOFA / 发命令 |
| 24V | XT30 | ⚠️ 先确认极性；电机必须固定 |

**上电自检**：电机指示灯**绿灯每秒闪 N 次 = 当前 ID = N**（本工程默认 `MOTOR_ID = 1`，反馈帧 0x205）。
改 ID：断电 → 拨码 Bit0~2 → 上电重数。

---

## 二、上电后串口会打印什么

```
==== GM6020 CAN demo (F405 + FreeRTOS) ====
clock source : HSE 8MHz
SYSCLK=168000000 HCLK=168000000 PCLK1=42000000 PCLK2=84000000
CAN1: NORMAL baud=1000000 bps (prescaler=3, TQ=14)
motor ID=1, feedback ID=0x205, ctrl ID=0x1FF
send 'help' for commands
mode=0 online=1 rx=1234 ang=123.4 cont=123.4 spd=0 cur=0.01A tmp=31C out=0
```

- `baud=1000000` —— 说明 CAN 时序正确（分频由 PCLK1 自动算，换时钟也不会错）
- `online=1` / `rx` 持续增长 —— 说明收到了电机反馈帧
- 若 `online=0`：检查 24V、CANH/CANL 是否接反、终端电阻、拨码 ID

---

## 三、命令表（串口发，回车结束；**改参数不用重新编译**）

| 命令 | 作用 | 例子 |
|---|---|---|
| `help` | 显示帮助 | |
| `v <volt>` | 直接给电压试转（先小后大） | `v 2000` |
| `sp <rpm>` | 速度环目标 | `sp 100` |
| `an <deg>` | 位置环：相对当前转 N 度（支持 720） | `an 180` / `an 720` |
| `kp/ki/kd` | 速度环 Kp/Ki/Kd | `kp 40` |
| `kp2/ki2/kd2` | 位置环参数 | `kp2 1.2` |
| `lv <v>` | 输出限幅（安全，默认 6000） | `lv 12000` |
| `ls <rpm>` | 内环速度限幅（默认 250） | `ls 150` |
| `log 1` / `log 0` | JustFloat 曲线 / 文本打印 | |
| `zero` | 当前角度清零 | |
| `stop` | 停止输出（回 IDLE） | |

**看曲线**：VOFA 选串口 115200 + 协议 **JustFloat**，发 `log 1` 后开始打 4 条曲线：
`ch0 = 目标`、`ch1 = 实际`、`ch2 = 输出`、`ch3 = 误差`。

---

## 四、验收步骤（按顺序做，每步都留曲线）

1. **辨识/修改 ID**：数灯确认 ID；拨码改成另一个 ID，改 `Inc/board_config.h` 的 `MOTOR_ID`，重新编译验证反馈帧号变化。
2. **读反馈**：`log 0` 下看文本行的 `ang / spd / tmp`；Keil Debug 的 Watch 里看 `g_motor`（角度随手动转动变化）。
3. **让电机转**：`v 2000` → 缓速转动；`v -2000` 反向；`stop` 停止。
4. **速度环**：`sp 100` → 调 `kp`/`ki` 到又快又不冲；`log 1` 打阶跃曲线，记录超调/响应时间/稳态误差。
5. **位置环（双环）**：`an 180` → 先调内环速度环，再调 `kp2/kd2`；`an 720` 验证多圈。
6. **保护验证**：拔掉 CAN 线 → 串口 `online=0`，电机立即停止出力。

---

## 五、代码结构

| 文件 | 作用 |
|---|---|
| `Src/pid.c` / `Inc/pid.h` | **PID 库**：`PID_t` 结构体 + `PID_Init/Calc/SetParam/Reset`，支持多实例、积分限幅、输出限幅、死区 |
| `Src/motor_gm6020.c/.h` | GM6020 对象：反馈解析（大端）、**连续多圈角度**（跨零点 ±8192 修正）、电压指令组帧 |
| `Src/can.c/.h` | CAN1 初始化（PA11/PA12）、**1Mbps 分频自动计算**、掩码过滤器收 0x200~0x20F、ISR 只入队 |
| `Src/usart.c/.h` | UART5（PC12/PD2）+ printf 重定向 |
| `Src/uart_vofa.c`、`vofa_send.c` | 空闲中断+DMA 收不定长命令；JustFloat 曲线发送 |
| `Src/app_tasks.c/.h` | 三个任务：控制(1ms) / 日志(5ms) / 命令；双环逻辑与全局 `g_ctrl`、`g_motor`（Debug 可直接改） |

**双环结构**（`Task_MotorCtrl`）：
```
目标角度 --[位置环 pidAngle]--> 速度目标(限幅 ±speedLimit) --[速度环 pidSpeed]--> 电压 --> CAN
```

---

## 六、已知约束

- **GM6020 最高 320rpm**，任务书的"1000rpm"是 M3508 量级，本工程速度上限设为 250rpm（`MOTOR_SPEED_LIMIT_RPM`）。
- 上电默认 `MODE_IDLE`（不输出），必须先发命令才转，避免上电飞车。
- 反馈超过 200ms 没更新 → 自动停止输出并清 PID 积分。
