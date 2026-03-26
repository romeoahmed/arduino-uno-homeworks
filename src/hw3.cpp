// ─────────────────────────────────────────────────────────────────────────────
//  HW-3 │ Hardware PWM  (Timer1 Fast PWM, OC1B → D10)
//
//  Wiring: output pin D10.
//
//  Timer1 Fast PWM 8-bit (WGM = 0b0101), prescale = 64:
//    F_PWM = 16 MHz / (64 × 256) ≈ 976 Hz
//    Duty  = (OCR1B + 1) / 256  ≈ 25 %   (OCR1B = 63)
//
//  Note: Arduino's analogWrite() uses Phase-Correct PWM → ≈ 490 Hz.
//  Direct register setup is used here to match Fast PWM behaviour.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>

static constexpr uint8_t PWM_PIN = 10;   // OC1B
static constexpr uint8_t DUTY_8BIT = 63; // (63+1)/256 = 25 %

void setup() {
    pinMode(PWM_PIN, OUTPUT);

    // WGM13:0 = 0b0101  →  Fast PWM 8-bit, TOP = 0xFF
    // COM1B1:0 = 0b10   →  Clear OC1B on match, set at BOTTOM (non-inverting)
    // CS12:0   = 0b011  →  prescale = 64
    TCCR1A = _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1B = DUTY_8BIT;
}

void loop() {
    // Hardware drives the PWM signal continuously; CPU has nothing to do.
}
