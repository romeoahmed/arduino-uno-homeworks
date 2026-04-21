// ─────────────────────────────────────────────────────────────────────────────
//  HW-11 │ 2-Digit 7-Segment Voltmeter
//
//  A0 → 16-sample oversampled ADC → mV → two-digit display (e.g. "33" = 3.3 V)
//  Timer1 CTC @ 1 kHz drives multiplexing ISR (500 Hz per-digit refresh)
//
//  Segment pins : D2(a) D3(b) D4(c) D5(d) D6(e) D7(f) D8(g) D9(dp)
//  Digit anodes : D10(left / integer)  D11(right / decimal)
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// ══ Display ═══════════════════════════════════════════════════════════════════

namespace Display {

// Segment pins D2–D9 map to bits 0–7 of a pattern byte (common-cathode active-high).
// Written inverted (~) to pins for common-anode logic.
static constexpr uint8_t DP_BIT = 0b10000000; // bit 7 = decimal point

static const uint8_t kSeg[10] PROGMEM = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
};

static volatile uint8_t gPattern[2]; // [0]=left [1]=right; CA: 0=on, 1=off
static volatile uint8_t gDigitIdx = 0;

// D2–D7 = PORTD[2:7] = pattern bits 0–5
// D8–D9 = PORTB[0:1] = pattern bits 6–7
// One register write per port — no loop, no digitalWrite overhead.
static void writePattern(uint8_t pat) {
    PORTD = (PORTD & 0x03) | static_cast<uint8_t>(pat << 2);  // bits 0–5 → PD2–PD7
    PORTB = (PORTB & 0xFC) | static_cast<uint8_t>(pat >> 6);  // bits 6–7 → PB0–PB1
}

static void init() {
    DDRD |= 0xFC;                           // PD2–PD7 output (seg a–f)
    DDRB |= 0x0F;                           // PB0–PB3 output (seg g/dp + digit anodes)
    PORTD = (PORTD & 0x03) | 0xFC;         // segments off (CA: HIGH = off)
    PORTB = (PORTB & 0xF0) | 0x03;         // seg g/dp off; anodes LOW (digits disabled)
    gPattern[0] = gPattern[1] = 0xFF;
}

// Precompute both patterns; single-byte writes are atomic on AVR.
static void show(uint8_t tens, uint8_t units) {
    if (tens  > 9) tens  = 9;
    if (units > 9) units = 9;
    gPattern[0] = static_cast<uint8_t>(~(pgm_read_byte(&kSeg[tens]) | DP_BIT));
    gPattern[1] = static_cast<uint8_t>(~pgm_read_byte(&kSeg[units]));
}

} // namespace Display

ISR(TIMER1_COMPA_vect) {
    using namespace Display;
    PORTB &= ~0x0C;                                         // disable both anodes (PB2/PB3)
    gDigitIdx ^= 1;
    writePattern(gPattern[gDigitIdx]);
    PORTB |= static_cast<uint8_t>(gDigitIdx == 0 ? _BV(PB2) : _BV(PB3)); // enable new digit
}

// ══ ADC ═══════════════════════════════════════════════════════════════════════

namespace Adc {

static constexpr uint8_t PIN = A0;

static void init() { pinMode(PIN, INPUT); }

// 16-sample oversampling → ~11-bit effective resolution.
// Returns millivolts (0–5000).
static uint16_t read() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16; ++i) sum += analogRead(PIN);
    sum >>= 4; // divide by 16
    return static_cast<uint16_t>(sum * 5000UL / 1023UL);
}

} // namespace Adc

// ══ Timer1 CTC @ 1 kHz ════════════════════════════════════════════════════════

static void setupTimer1() {
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11);          // CTC, prescaler = 8
    OCR1A  = F_CPU / (8UL * 1000UL) - 1;      // 1999 → 1 kHz
    TIMSK1 = _BV(OCIE1A);
}

// ══ Serial command parser ═════════════════════════════════════════════════════

static bool gStreaming = false;

static void handleCommand(char const* cmd) {
    if      (strcmp(cmd, "start") == 0) { gStreaming = true;  Serial.println(F("OK: echo on")); }
    else if (strcmp(cmd, "stop")  == 0) { gStreaming = false; Serial.println(F("OK: echo off")); }
    else                                  Serial.println(F("Commands: start  stop"));
}

// ══ Arduino 入口 ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(9600);
    Adc::init();
    Display::init();
    setupTimer1();
    sei();
    Serial.println(F("━━━ Voltmeter ━━━"));
}

void loop() {
    // ── 串口命令解析 (非阻塞) ─────────────────────────────────────────────
    static char    buf[12];
    static uint8_t len = 0;
    static bool    overflow = false;

    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (!overflow && len > 0) { buf[len] = '\0'; handleCommand(buf); }
            overflow = false; len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            overflow = true; len = 0;
        }
    }

    // ── 采样 & 显示 (5 Hz) ───────────────────────────────────────────────
    static uint32_t lastMs = 0;
    uint32_t const  now    = millis();
    if (now - lastMs < 200) return;
    lastMs = now;

    uint16_t const mV      = Adc::read();
    uint8_t  const rounded = static_cast<uint8_t>((mV + 50U) / 100U); // 0–50
    uint8_t  const tens    = static_cast<uint8_t>(rounded / 10U);
    uint8_t  const units   = static_cast<uint8_t>(rounded % 10U);
    Display::show(tens, units);

    if (gStreaming) {
        Serial.print(F("V="));
        Serial.print(tens);
        Serial.println(units);
    }
}
