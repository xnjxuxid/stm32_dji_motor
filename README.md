# 阶段三：富斯遥控器（DR16/SBUS）+ BMI088 IMU 驱动

> 基于阶段二 gimbal 工程扩展（阶段二全部功能保留：CAN 驱动 GM6020 + 双环 PID + 在线调参）。
> 工程：`MDK-ARM/stage3.uvprojx`（0 Error / 0 Warning），配置记录：`stage3.ioc`（CubeMX 可打开）

---

## 一、任务与实现方式

| 任务书要求 | 实现 | 说明 |
|---|---|---|
| 能用富斯遥控控制 DJI 电机 | ✅ | DR16(DBUS)/SBUS 自动识别，S1 拨杆 = 安全开关，右摇杆控制速度 |
| 遥控保护：关遥控电机立即停止输出、没力 | ✅ | 失联 100ms → 立即 MODE_IDLE + 清 PID 积分 + 发 0 电压 |
| IMU 驱动"使用 IIC" | ⚠️ SPI | **原理图确认板载 IMU 是 SPI 接口**（BMI088 的 PS 脚接地锁死 SPI 模式，且只连了 SPI1），板上 I2C1 只挂了 EEPROM，故按硬件事实用 SPI |
| "使用 DMP 库" | ⚠️ 不适用 | BMI088（博世）与 ICM42688（TDK）**都没有 DMP**（DMP 是 InvenSense MPU 系列专属）。本阶段按确认方案：先做**自检 + 原始六轴**，姿态解算后续补 |
| "需要自检" | ✅ | 三重自检：①加速度计 CHIP_ID=0x1E ②陀螺 CHIP_ID=0x0F ③陀螺自检寄存器流程 ④静止加速度模长≈1g 校验 |

## 二、接线

| 云台板 | 外设 | 说明 |
|---|---|---|
| **DBUS 座**（板上标 DBUS，下层） | DR16 接收机 CH5（标 DBUS 的那个口） | 板上 SS8050 已做电平反相，直插 |
| DR16 供电 | 5V / GND | 接收机要 5V |
| 遥控器 | FS-i6X，需切到 **SBUS/DBUS 输出模式** | 见"遥控器设置" |
| 电机 / VOFA / SWD | 与阶段二相同 | CAN1 PA11/PA12、UART5 PC12/PD2 |

板载 IMU（BMI088）已在板上，**无需接线**：SPI1=PA5/PA6/PA7，CS_Accel=PC4，CS_Gyro=PA4。

## 三、遥控器设置（FS-i6X）

1. 遥控器开机 → 长按 OK 进菜单 → **系统设置**
2. 找到 **RX SETUP（接收机设置）→ 输出模式**，切换为 **SBUS**
3. 接收机重新对码（按接收机对码键 + 遥控器菜单里 Bind）
4. 对码成功后 DR16/接收机的 DBUS 口输出 100000 8E2 反相信号

## 四、上电自检输出（期望）

```
==== GM6020 CAN demo (F405 + FreeRTOS) ====
clock source : HSE 25000000 Hz
SYSCLK=168000000 ... CAN1: NORMAL baud=1000000 bps
motor ID=1, feedback ID=0x205, ctrl ID=0x1FF
BMI088 self-test:
  acc  CHIP_ID = 0x1E (expect 0x1E) : OK
  gyro CHIP_ID = 0x0F (expect 0x0F) : OK
  gyro self-test reg = 0x02 : OK
  acc magnitude = 1.012 g (expect ~1.0, keep still) : OK
BMI088 self-test PASSED
```

之后串口两种日志交替（各 500ms 一行）：

```
mode=0 online=1 rx=... ang=... spd=... tmp=... out=0 | can lec=0 ...
imu: acc[ 0.02 -0.01  1.01]g gyro[  0.10  -0.20   0.05]dps temp=31.5C n=123
```

- `acc[2] ≈ 1.0g`（板子平放）→ BMI088 工作正常
- `gyro` 静止时应 < ±1°/s

## 五、遥控控制电机（验收项 1）

```
rc 1          ← 串口命令：开启遥控控制
```

| 操作 | 期望 |
|---|---|
| **S1 拨杆拨到上位** | 才允许输出（安全开关，默认停机） |
| **右摇杆上下推** | 电机按摇杆量调速（速度环，±250rpm） |
| 摇杆回中 | 电机停（速度目标 0） |
| S1 拨到中/下 | 立即停机 |
| **关遥控器** | **0.1 秒内电机立即没力**（失联保护）——串口打印 `[RC] link lost -> motor disabled` |
| 重新开遥控器 | 串口恢复，S1 上位后可继续控制 |

串口还可以看遥控原始数据：Watch 加 `g_rc`（`ch[0..3]`、`s1`、`frameCount`、`linked`）。

## 六、代码结构（阶段三新增）

| 文件 | 作用 |
|---|---|
| `Src/rc_dr16.c/h` | DR16(DBUS)/SBUS 自动识别解析、逐字节中断+帧超时组装、链路状态 |
| `Src/bmi088.c/h` | BMI088 SPI 驱动：初始化、**三重自检**、六轴+温度读取（加速度计读带 dummy 字节） |
| `Src/spi.c/h` | SPI1 初始化（2.625MHz，Mode0）+ 软件片选 |
| `Src/app_tasks.c` | 新增 Task_Rc（10ms，失联保护）与 Task_Imu（10ms 读取） |

## 七、已知说明（报告用）

1. **IIC→SPI**：原理图 IMU_V4.7.4 中 BMI088 的 PS 脚接地（强制 SPI 协议），且只连了
   SPI1（PA5/6/7）与独立片选；板上 I2C1(PB6/PB7) 仅挂 24C02 EEPROM。
   按"原理图确认 IMU 为 SPI 接口"处理。
2. **DMP**：BMI088 / ICM-42688 均无 DMP 引擎（DMP 为 InvenSense MPU6050/ICM-20948 系列
   专属）。本阶段交付：自检 + 原始六轴；姿态解算（互补滤波，阶段二已有 `attitude.c` 可移植）
   在下一迭代补充。
3. **DR16 vs SBUS**：两者同为 100000bps / 8E2 / 反相电平，代码按帧长（18/25 字节）
   与帧头帧尾自动识别，两种接收机都能用。

## 八、阶段二功能回归

阶段二的所有验收功能保留：CAN 驱动 GM6020、速度环/位置环双环、多圈角度、
VOFA 曲线、串口免编译调参（命令表见阶段二 README，新增 `rc 1/0` 与 `imust`）。
