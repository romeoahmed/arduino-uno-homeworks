// ─────────────────────────────────────────────────────────────────────────────
//  MPU6500 │ Accelerometer Filter Demo
//
//  MPU6500 (I2C 0x68) — 加速度计 X 轴数据经三种滤波后输出到串口
//
//  输出格式 (CSV, 10 Hz):
//    raw,avg,median,lp
//
//  Serial commands (57600 baud):
//    start  – 开始推流
//    stop   – 暂停推流
//    alpha  <0.0‥1.0>  – 设置低通系数 (默认 0.1)
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>

// ══ MPU6500 (I2C) ═════════════════════════════════════════════════════════════

namespace Mpu {

static constexpr uint8_t ADDR = 0x68;
static constexpr uint8_t REG_PWR = 0x6B;
static constexpr uint8_t REG_ACCEL_CFG = 0x1C;
static constexpr uint8_t REG_GYRO_CFG = 0x1B;
static constexpr uint8_t REG_ACCEL = 0x3B;
static constexpr float ACCEL_LSB = 16384.0f; // ±2 g → LSB/g

// 当前采样值 (g)
float ax = 0, ay = 0, az = 0;

static void init() {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_PWR);
    Wire.write(0x00); // 退出睡眠
    Wire.endTransmission(true);

    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACCEL_CFG);
    Wire.write(0x00); // ±2 g
    Wire.endTransmission(true);

    Wire.beginTransmission(ADDR);
    Wire.write(REG_GYRO_CFG);
    Wire.write(0x00); // ±250 °/s
    Wire.endTransmission(true);

    delay(100);
}

// 读取加速度计三轴; 返回 false 表示 I2C 失败.
static bool read() {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACCEL);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom(ADDR, uint8_t{6}) != 6)
        return false;

    auto raw16 = []() -> int16_t { return static_cast<int16_t>((Wire.read() << 8) | Wire.read()); };
    ax = raw16() / ACCEL_LSB;
    ay = raw16() / ACCEL_LSB;
    az = raw16() / ACCEL_LSB;
    return true;
}

} // namespace Mpu

// ══ Filter ════════════════════════════════════════════════════════════════════

namespace Filter {

static constexpr uint8_t WIN = 5;

// ── 均值滤波 ─────────────────────────────────────────────────────────────────
static float avgBuf[WIN];

static float average(float v) {
    float sum = v;
    for (uint8_t i = 0; i < WIN - 1; ++i) {
        avgBuf[i] = avgBuf[i + 1];
        sum += avgBuf[i];
    }
    avgBuf[WIN - 1] = v;
    return sum / WIN;
}

// ── 中值滤波 ─────────────────────────────────────────────────────────────────
static float medBuf[WIN];

static float median(float v) {
    for (uint8_t i = 0; i < WIN - 1; ++i)
        medBuf[i] = medBuf[i + 1];
    medBuf[WIN - 1] = v;

    float s[WIN];
    for (uint8_t i = 0; i < WIN; ++i)
        s[i] = medBuf[i]; // 复制全部窗口

    // 冒泡排序
    for (uint8_t i = 0; i < WIN - 1; ++i)
        for (uint8_t j = 0; j < WIN - i - 1; ++j)
            if (s[j] > s[j + 1]) {
                float t = s[j];
                s[j] = s[j + 1];
                s[j + 1] = t;
            }

    return s[WIN / 2];
}

// ── 一阶低通滤波 ─────────────────────────────────────────────────────────────
static float alpha = 0.1f;
static float lpLast = 0.0f;

static float lowPass(float v) {
    lpLast = alpha * v + (1.0f - alpha) * lpLast;
    return lpLast;
}

// 用第一次真实采样值预热各缓冲区, 避免冷启动的阶跃瞬态.
static void warmUp(float v) {
    for (uint8_t i = 0; i < WIN; ++i) {
        avgBuf[i] = v;
        medBuf[i] = v;
    }
    lpLast = v;
}

} // namespace Filter

// ══ Serial command parser ═════════════════════════════════════════════════════

static bool gStreaming = true;

static void handleCommand(char const* cmd) {
    if (strcmp(cmd, "start") == 0) {
        gStreaming = true;
        Serial.println(F("OK: streaming started"));
    } else if (strcmp(cmd, "stop") == 0) {
        gStreaming = false;
        Serial.println(F("OK: streaming stopped"));
    } else if (strncmp(cmd, "alpha ", 6) == 0) {
        char* end;
        float a = static_cast<float>(strtod(cmd + 6, &end));
        if (end == cmd + 6) {
            Serial.println(F("ERR: invalid number"));
        } else if (a < 0.0f || a > 1.0f) {
            Serial.println(F("ERR: alpha must be 0.0‥1.0"));
        } else {
            Filter::alpha = a;
            Serial.print(F("OK: alpha = "));
            Serial.println(a, 3);
        }
    } else {
        Serial.println(F("Commands: start  stop  alpha <0.0‥1.0>"));
    }
}

// ══ Arduino 入口 ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(57600);
    Wire.begin();
    Mpu::init();

    // 预热: 先读一次真实数据再填充滤波缓冲区
    Mpu::read();
    Filter::warmUp(Mpu::ax);

    Serial.println(F("━━━ MPU6500 Filter Demo ━━━"));
    Serial.println(F("raw,avg,median,lp"));
}

void loop() {
    // ── 串口命令解析 (非阻塞) ─────────────────────────────────────────────
    static char buf[24];
    static uint8_t len = 0;
    static bool overflow = false;

    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (!overflow && len > 0) {
                buf[len] = '\0';
                handleCommand(buf);
            }
            overflow = false;
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            overflow = true;
            len = 0;
        }
    }

    // ── 采样 & 推流 (10 Hz) ──────────────────────────────────────────────
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < 100)
        return;
    lastMs = now;

    if (!Mpu::read()) {
        Serial.println(F("ERR: I2C"));
        return;
    }

    if (gStreaming) {
        float raw = Mpu::ax;
        Serial.print(raw, 3);
        Serial.print(',');
        Serial.print(Filter::average(raw), 3);
        Serial.print(',');
        Serial.print(Filter::median(raw), 3);
        Serial.print(',');
        Serial.println(Filter::lowPass(raw), 3);
    }
}
