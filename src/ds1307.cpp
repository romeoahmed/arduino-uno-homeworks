// ─────────────────────────────────────────────────────────────────────────────
//  DS1307 RTC — 读写测试
//
//  Wiring (Arduino UNO):
//    A4 — SDA
//    A5 — SCL
//    (Wire uses hardware I2C; no pin config needed)
//
//  Test sequence (setup):
//    1. Set time once (uncomment setTime call to use)
//    2. Read back immediately and print to serial
//  Loop:
//    Read & print current time every second.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>

// ── DS1307 constants ──────────────────────────────────────────────────────────
static constexpr uint8_t DS1307_ADDR = 0x68;
static constexpr uint8_t REG_SECONDS = 0x00; // first timekeeping register
static constexpr uint8_t TIME_REG_COUNT = 7; // sec/min/hr/dow/day/mon/yr

// Register bit masks
static constexpr uint8_t MASK_SEC_CH = 0x7F;  // bit7 = Clock Halt; mask it out on read
static constexpr uint8_t MASK_HOUR_24 = 0x3F; // bit6 = 12/24 mode; always use 24 h

// ── Time representation ───────────────────────────────────────────────────────
struct DateTime {
    uint8_t second;    // 0–59
    uint8_t minute;    // 0–59
    uint8_t hour;      // 0–23  (24-hour)
    uint8_t dayOfWeek; // 1–7   (1 = Sunday … 7 = Saturday, DS1307 convention)
    uint8_t day;       // 1–31
    uint8_t month;     // 1–12
    uint8_t year;      // 0–99  (year offset from 2000)
};

// ── BCD helpers ───────────────────────────────────────────────────────────────
static constexpr uint8_t toBcd(uint8_t v) {
    return static_cast<uint8_t>(((v / 10) << 4) | (v % 10));
}
static constexpr uint8_t fromBcd(uint8_t v) {
    return static_cast<uint8_t>(((v >> 4) * 10) + (v & 0x0F));
}

// ── Forward declarations ──────────────────────────────────────────────────────
[[maybe_unused]] static bool setTime(DateTime const& dt);
static bool readTime(DateTime& dt);
static void printTime(DateTime const& dt);
static void printDayOfWeek(uint8_t dow);

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Wire.begin();
    Serial.begin(57600);
    Serial.println(F("DS1307 实时时钟测试"));
    Serial.println(F("==================="));

    // ── Optional: set time ────────────────────────────────────────────────────
    // Uncomment the block below once to initialise the RTC, then re-comment and
    // re-flash so the clock is not reset on every power cycle.
    //
    // constexpr DateTime kInitTime{ .second=0, .minute=44, .hour=12,
    //                               .dayOfWeek=4, .day=18, .month=3, .year=26 };
    // if (!setTime(kInitTime)) {
    //     Serial.println(F("ERR: setTime failed"));
    // }

    Serial.println(F("读取 RTC 当前时间...\n"));
}

void loop() {
    DateTime dt{};
    if (readTime(dt)) {
        printTime(dt);
    } else {
        Serial.println(F("ERR: I2C read failed"));
    }
    delay(1000);
}

// ── DS1307 I/O ────────────────────────────────────────────────────────────────

// Write dt into the DS1307 timekeeping registers.
// Returns true on success (Wire.endTransmission() == 0).
[[maybe_unused]] static bool setTime(DateTime const& dt) {
    Wire.beginTransmission(DS1307_ADDR);
    Wire.write(REG_SECONDS);
    // Writing toBcd(second) with bit7=0 also clears the Clock Halt flag,
    // starting the oscillator if it was stopped.
    Wire.write(toBcd(dt.second));
    Wire.write(toBcd(dt.minute));
    Wire.write(toBcd(dt.hour));
    Wire.write(toBcd(dt.dayOfWeek));
    Wire.write(toBcd(dt.day));
    Wire.write(toBcd(dt.month));
    Wire.write(toBcd(dt.year));
    return Wire.endTransmission() == 0;
}

// Read the DS1307 timekeeping registers into dt.
// Returns true when exactly TIME_REG_COUNT bytes were received.
//
// Protocol: write the register pointer (0x00) with a repeated START
// (endTransmission(false)), then requestFrom to clock out 7 bytes.
// requestFrom() sends the STOP itself — do NOT call endTransmission() after.
static bool readTime(DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDR);
    Wire.write(REG_SECONDS);
    if (Wire.endTransmission(false) != 0)
        return false; // repeated START

    uint8_t received = Wire.requestFrom(DS1307_ADDR, TIME_REG_COUNT);
    if (received != TIME_REG_COUNT)
        return false;

    dt.second = fromBcd(static_cast<uint8_t>(Wire.read()) & MASK_SEC_CH);
    dt.minute = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.hour = fromBcd(static_cast<uint8_t>(Wire.read()) & MASK_HOUR_24);
    dt.dayOfWeek = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.day = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.month = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.year = fromBcd(static_cast<uint8_t>(Wire.read()));
    return true;
}

// ── Formatting ────────────────────────────────────────────────────────────────

// Print the Chinese day-of-week label from Flash (F macro → no RAM cost).
static void printDayOfWeek(uint8_t dow) {
    switch (dow) {
    case 1:
        Serial.print(F("周日"));
        break;
    case 2:
        Serial.print(F("周一"));
        break;
    case 3:
        Serial.print(F("周二"));
        break;
    case 4:
        Serial.print(F("周三"));
        break;
    case 5:
        Serial.print(F("周四"));
        break;
    case 6:
        Serial.print(F("周五"));
        break;
    case 7:
        Serial.print(F("周六"));
        break;
    default:
        Serial.print(F("???"));
        break;
    }
}

// Print one line: "2026-03-27 (周四) 12:44:00"
// snprintf into a stack buffer keeps the serial call count to a minimum and
// avoids the repeated if (x < 10) Serial.print("0") pattern.
static void printTime(DateTime const& dt) {
    char buf[24];

    // snprintf returns the number of characters it *would* write (excluding '\0').
    // A negative return signals an encoding error; a value >= sizeof(buf) means the
    // output was silently truncated.  cert-err33-c requires both cases to be handled.
    int n = snprintf(buf, sizeof(buf), "20%02u-%02u-%02u (", dt.year, dt.month, dt.day);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
        Serial.println(F("ERR: date format"));
        return;
    }
    Serial.print(buf);

    printDayOfWeek(dt.dayOfWeek);

    n = snprintf(buf, sizeof(buf), ") %02u:%02u:%02u", dt.hour, dt.minute, dt.second);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
        Serial.println(F("ERR: time format"));
        return;
    }
    Serial.println(buf);
}
