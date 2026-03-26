# Arduino UNO Homeworks — PlatformIO

## 快速开始

```bash
# 编译单题
pio run -e hw1

# 编译并烧录
pio run -e hw2 -t upload

# 串口监视（hw2 专用，57600 baud）
pio device monitor -e hw2
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
