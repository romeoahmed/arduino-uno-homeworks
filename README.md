# Arduino UNO Homeworks — PlatformIO

## 快速开始

```bash
# 编译单题
pio run -e blink

# 编译并烧录
pio run -e blink -t upload

# 串口监视（57600 baud）
pio device monitor -e state_machine
```

---

## HW-1 — LED Blink

| 引脚 | 功能 |
|------|------|
| D13  | 内置 LED |

500 ms 亮 → 2000 ms 灭，循环。

---

## HW-2 — Serial 状态机

| 引脚 | 功能 |
|------|------|
| D13  | 奇数模式：1 Hz 闪烁 |
| D8   | 偶数模式：500 Hz 方波（无源蜂鸣器）|

串口发送整数并回车：

- **奇数** → LED 1 Hz 闪，蜂鸣器静音
- **偶数** → LED 熄灭，蜂鸣器 500 Hz

实现：Timer1 CTC @ 1 kHz，ISR 驱动；主循环仅做串口解析。

---

## HW-3 — Hardware PWM

| 引脚 | 功能 |
|------|------|
| D10  | PWM 输出（OC1B）|

Timer1 Fast PWM 8-bit，prescale = 64：

- **频率** ≈ 976 Hz（对比 `analogWrite` 的相位修正 PWM ≈ 490 Hz）
- **占空比** ≈ 25 %（OCR1B = 63）

---

## HW-4 — 《小星星》播放器

| 引脚 | 功能 |
|------|------|
| D8   | 无源蜂鸣器 |
| D9   | 按钮（接地，内部上拉）|

- 按下按钮：暂停 ↔ 播放
- 暂停时立即静音，恢复时从当前音符重新播放
- 歌曲循环播放

使用 Arduino `tone()` / `noTone()`（内部占用 Timer2）。

---

## HW-5 — W25Q SPI Flash 读写测试

| 引脚 | 功能 |
|------|------|
| D10  | CS（手动 GPIO）|
| D11  | MOSI |
| D12  | MISO |
| D13  | SCK |

测试流程：

1. 读取 JEDEC ID — 验证芯片连接
2. 擦除 Sector 0（4 KB @ 0x000000）
3. Page Program — 写入测试数据
4. 回读比较 — 逐字节 memcmp → PASS / FAIL

---

## HW-6 — DS1307 RTC 读写测试

| 引脚 | 功能 |
|------|------|
| A4   | SDA |
| A5   | SCL |

- `setup()` 中可选写入一次时间
- `loop()` 每秒读取并打印当前时间

---

## HW-7 — 持久化时间戳事件日志

| 引脚 | 功能 |
|------|------|
| A4/A5 | DS1307 RTC（I2C）|
| D10–D13 | W25Qxx NOR Flash（SPI）|

Flash 布局：顺序写入 8 字节 LogEntry，二分查找定位写入位置（O(log N)），最多 4096 条。

串口命令（57600 baud）：

| 命令 | 功能 |
|------|------|
| `log`   | 读 RTC 时间戳，写入一条日志 |
| `dump`  | 打印全部已存日志 |
| `clear` | 擦除所有扇区 |

---

## HW-8 — UART 串口输入

两种串口解析方式体会（57600 baud）：

- **Part A** — `Serial.read()`：发送 `a <整数>`（支持负数）
- **Part B** — `Serial.parseInt()`：发送 `b <n1> <n2> …`（回车结束）

---

## HW-9 — MPU6500 加速度计滤波

| 引脚 | 功能 |
|------|------|
| A4   | SDA |
| A5   | SCL |

X 轴加速度经三种滤波后以 CSV 格式 10 Hz 推流：`raw,avg,median,lp`

串口命令（57600 baud）：

| 命令 | 功能 |
|------|------|
| `start` | 开始推流 |
| `stop`  | 暂停推流 |
| `alpha <0.0‥1.0>` | 设置低通系数（默认 0.1）|

---

## HW-10 — 速度融合：超声波 + 加速度计

| 引脚 | 功能 |
|------|------|
| D3   | SR04 Trig |
| D4   | SR04 Echo |
| A4   | MPU6500 SDA |
| A5   | MPU6500 SCL |

超声波测距（有限差分）+ 三轴加速度积分 → 互补滤波融合瞬时速度。

输出格式（57600 baud，10 Hz）：`D=25.3cm  Vu=12.1  Va=11.8  Vf=11.9 cm/s`

串口命令：

| 命令 | 功能 |
|------|------|
| `start` | 开始推流 |
| `stop`  | 暂停推流 |
| `alpha <0.0..1.0>` | 互补滤波权重（默认 0.5，越大越信超声波）|
| `lp <0.0..1.0>`    | 加速度低通系数（默认 0.1）|
