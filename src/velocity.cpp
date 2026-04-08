// ─────────────────────────────────────────────────────────────────────────────
//  HW-10 │ Velocity Fusion: Ultrasonic + Accelerometer
//
//  HC-SR04 (D3/D4)    — ultrasonic distance → velocity via finite difference
//  MPU6500 (I2C 0x68) — 3-axis accelerometer → velocity via integration
//  Complementary filter fuses both estimates.
//
//  引脚接线:
//    D3  → SR04 Trig
//    D4  → SR04 Echo
//    A4  → MPU6500 SDA
//    A5  → MPU6500 SCL
//
//  输出格式 (57600 baud, human-readable):
//    D=25.3cm  Vu=12.1  Va=11.8  Vf=11.9 cm/s
//
//  Serial commands:
//    start              – 开始推流
//    stop               – 暂停推流
//    alpha <0.0..1.0>   – 互补滤波权重 (default 0.5, higher = trust ultrasonic)
//    lp <0.0..1.0>      – 加速度低通系数 (default 0.1)
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <math.h> // avr-libc; <cmath> is not reliably available on AVR toolchain

// ══ Sonar (HC-SR04) ═══════════════════════════════════════════════════════════

namespace Sonar {

static constexpr uint8_t TRIG = 3;
static constexpr uint8_t ECHO = 4;
static constexpr uint32_t TIMEOUT_US = 60000; // ~10 m max range

static void init() {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    digitalWrite(TRIG, LOW);
}

// Returns distance in cm. Returns 0.0 on timeout (no echo within ~5 m).
static float measure() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    uint32_t t = pulseIn(ECHO, HIGH, TIMEOUT_US);
    if (t == 0)
        return 0.0f;
    return static_cast<float>(t) / 58.0f;
}

} // namespace Sonar

// ══ Mpu (MPU6500) ═════════════════════════════════════════════════════════════

namespace Mpu {

static constexpr uint8_t ADDR = 0x68;
static constexpr uint8_t REG_PWR = 0x6B;
static constexpr uint8_t REG_ACCEL_CFG = 0x1C;
static constexpr uint8_t REG_ACCEL = 0x3B;
static constexpr float ACCEL_LSB = 16384.0f; // ±2 g

static float ax = 0.0f, ay = 0.0f, az = 0.0f;

static void init() {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_PWR);
    Wire.write(0x00); // exit sleep
    Wire.endTransmission(true);

    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACCEL_CFG);
    Wire.write(0x00); // ±2 g
    Wire.endTransmission(true);

    delay(100);
}

// Reads ax, ay, az into namespace globals. Returns false on I2C failure.
static bool read() {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACCEL);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom(ADDR, uint8_t{6}) != 6)
        return false;

    auto raw16 = []() -> int16_t {
        uint8_t hi = static_cast<uint8_t>(Wire.read());
        uint8_t lo = static_cast<uint8_t>(Wire.read());
        return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
    };
    ax = raw16() / ACCEL_LSB;
    ay = raw16() / ACCEL_LSB;
    az = raw16() / ACCEL_LSB;
    return true;
}

} // namespace Mpu

// ══ Velocity (fusion) ═════════════════════════════════════════════════════════

namespace Velocity {

static float alpha = 0.5f;   // complementary weight (0=accel only, 1=ultrasonic only)
static float lp_coef = 0.1f; // acceleration low-pass coefficient

static float prev_dist = 0.0f;
static float v_accel = 0.0f;
static float a_prev = 0.0f;
static float a_lp = 0.0f;
static uint8_t still_cnt = 0;

// Call once with the first sonar distance to warm up internal state.
// Passing 0.0 (timeout on startup) is valid — starts at rest.
static void init(float dist) {
    prev_dist = dist;
    v_accel = 0.0f;
    a_prev = 0.0f;
    a_lp = 0.0f;
    still_cnt = 0;
}

// Fuse one 10 Hz tick of sensor data.
//   dist     — sonar distance in cm; 0.0 means timeout (hold previous)
//   ax,ay,az — MPU6500 readings in g
//   dt_s     — elapsed time since last call, in seconds
// Writes v_ultra_out and v_accel_out (cm/s); returns v_fused (cm/s).
static float update(float dist, float ax, float ay, float az, float dt_s, float& v_ultra_out,
                    float& v_accel_out) {
    // Gravity-compensated acceleration magnitude (g). Stationary reads ~0.
    float a_mag = sqrtf(ax * ax + ay * ay + az * az) - 1.0f;

    // Low-pass filter on acceleration
    a_lp = lp_coef * a_mag + (1.0f - lp_coef) * a_lp;

    // Ultrasonic velocity: finite difference, hold previous on timeout
    float effective_dist = (dist > 0.0f) ? dist : prev_dist;
    float v_ultra = (effective_dist - prev_dist) / dt_s;
    if (dist > 0.0f)
        prev_dist = effective_dist;

    // Trapezoidal integration of acceleration (g → cm/s²: × 981)
    v_accel += (a_lp + a_prev) * 0.5f * dt_s * 981.0f;
    a_prev = a_lp;

    // Drift correction: blend v_accel toward v_ultra each tick
    v_accel = 0.95f * v_accel + 0.05f * v_ultra;

    // Stationary snap-to-zero: 3 consecutive still ticks (300 ms) → zero v_accel.
    // Only count ticks where sonar returned a valid reading (dist > 0), so
    // a sensor timeout cannot falsely increment the counter while moving.
    if (dist > 0.0f && fabsf(a_lp) < 0.02f && fabsf(v_ultra) < 0.5f) {
        if (still_cnt < 255)
            ++still_cnt;
        if (still_cnt >= 3)
            v_accel = 0.0f;
    } else if (dist > 0.0f) {
        still_cnt = 0;
    }

    v_ultra_out = v_ultra;
    v_accel_out = v_accel;
    return alpha * v_ultra + (1.0f - alpha) * v_accel;
}

} // namespace Velocity

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
        float v = static_cast<float>(strtod(cmd + 6, &end));
        if (end == cmd + 6) {
            Serial.println(F("ERR: invalid number"));
        } else if (v < 0.0f || v > 1.0f) {
            Serial.println(F("ERR: alpha must be 0.0..1.0"));
        } else {
            Velocity::alpha = v;
            Serial.print(F("OK: alpha = "));
            Serial.println(v, 3);
        }
    } else if (strncmp(cmd, "lp ", 3) == 0) {
        char* end;
        float v = static_cast<float>(strtod(cmd + 3, &end));
        if (end == cmd + 3) {
            Serial.println(F("ERR: invalid number"));
        } else if (v < 0.0f || v > 1.0f) {
            Serial.println(F("ERR: lp must be 0.0..1.0"));
        } else {
            Velocity::lp_coef = v;
            Serial.print(F("OK: lp = "));
            Serial.println(v, 3);
        }
    } else {
        Serial.println(F("Commands: start  stop  alpha <0.0..1.0>  lp <0.0..1.0>"));
    }
}

// ══ Arduino entry ═════════════════════════════════════════════════════════════

static uint32_t gLastMs = 0; // initialised in setup() to avoid garbage first-tick dt

void setup() {
    Serial.begin(57600);
    Wire.begin();
    Sonar::init();
    Mpu::init();

    // Warm up: first sonar reading initialises velocity state.
    // On timeout (0.0) start at rest — still a valid initial state.
    Velocity::init(Sonar::measure());

    Serial.println(F("━━━ Velocity Fusion Demo ━━━"));
    Serial.println(F("alpha=0.5  lp=0.1  10 Hz"));
    Serial.println(F("Commands: start  stop  alpha <n>  lp <n>"));

    gLastMs = millis(); // sync timer after init so first dt is ~100 ms, not startup time
}

void loop() {
    // ── Serial command parser (non-blocking) ──────────────────────────────
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

    // ── Sample & stream (10 Hz) ───────────────────────────────────────────
    uint32_t const now = millis();
    if (now - gLastMs < 100)
        return;
    float const dt = static_cast<float>(now - gLastMs) * 0.001f;
    gLastMs = now;

    float const dist = Sonar::measure();

    if (!Mpu::read()) {
        Serial.println(F("ERR: I2C"));
        return;
    }

    float v_u, v_a;
    float const v_f = Velocity::update(dist, Mpu::ax, Mpu::ay, Mpu::az, dt, v_u, v_a);

    if (gStreaming) {
        Serial.print(F("D="));
        if (dist > 0.0f)
            Serial.print(dist, 1);
        else
            Serial.print(F("---"));
        Serial.print(F("cm  Vu="));
        Serial.print(v_u, 1);
        Serial.print(F("  Va="));
        Serial.print(v_a, 1);
        Serial.print(F("  Vf="));
        Serial.print(v_f, 1);
        Serial.println(F(" cm/s"));
    }
}
