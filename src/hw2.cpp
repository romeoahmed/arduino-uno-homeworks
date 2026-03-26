// ─────────────────────────────────────────────────────────────────────────────
//  HW-2 │ Serial-driven State Machine  (Timer1 CTC @ 1 kHz)
//
//  Serial (57600 baud): send an integer and press Enter.
//    Odd  → D13 LED blinks at 1 Hz  (500 ms half-period)
//    Even → D8  buzzer runs at 500 Hz (1 ms toggle period)
//
//  Timer1 is configured in CTC mode:
//    F_CPU=16 MHz, prescale=64, OCR1A=249 → 16 000 000/64/250 = 1 000 Hz
//  All time-critical work lives in the ISR; the main loop only does serial I/O.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <avr/interrupt.h>

// ── Pin assignments ───────────────────────────────────────────────────────────
static constexpr uint8_t LED_PIN = 13;   // PB5
static constexpr uint8_t BUZZER_PIN = 8; // PB0

// ── Timer tick counts ────────────────────────────────────────────────────────
static constexpr uint16_t BLINK_TICKS = 500; // LED toggle every 500 ms
static constexpr uint16_t BUZZER_TICKS = 1;  // buzzer toggle every 1 ms → 500 Hz

// ── Board state ──────────────────────────────────────────────────────────────
enum class State : uint8_t { Idle, Odd, Even };

struct ISRCtx {
    State state = State::Idle;
    uint16_t blinkCnt = 0;
    uint16_t buzzerCnt = 0;
};

static ISRCtx volatile ctx;

// ── ISR ─────────────────────────────────────────────────────────────────────
ISR(TIMER1_COMPA_vect) {
    switch (ctx.state) {

    case State::Odd:
        if (++ctx.blinkCnt >= BLINK_TICKS) {
            ctx.blinkCnt = 0;
            PINB = _BV(PB5); // toggle D13 via PINB write (atomic)
        }
        break;

    case State::Even:
        if (++ctx.buzzerCnt >= BUZZER_TICKS) {
            ctx.buzzerCnt = 0;
            PINB = _BV(PB0); // toggle D8
        }
        break;

    default:
        break;
    }
}

// ── Timer1 CTC setup ─────────────────────────────────────────────────────────
static void setupTimer1() {
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10); // CTC, prescale = 64
    OCR1A = 249;                                 // → 1 000 Hz
    TIMSK1 = _BV(OCIE1A);
}

// ── Thread-safe state transition ─────────────────────────────────────────────
static void applyState(State s) {
    noInterrupts();
    ctx.state = s;
    ctx.blinkCnt = 0;
    ctx.buzzerCnt = 0;
    PORTB &= ~(_BV(PB5) | _BV(PB0)); // LED off, buzzer off
    interrupts();
}

static __FlashStringHelper const* stateLabel(State s) {
    switch (s) {
    case State::Odd:
        return F("ODD  → LED blink 1 Hz, buzzer off");
    case State::Even:
        return F("EVEN → LED off, buzzer 500 Hz");
    default:
        return F("IDLE");
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    setupTimer1();

    Serial.begin(57600);
    Serial.println(F("Ready. Send an integer."));
}

void loop() {
    static char buf[16];
    static uint8_t len = 0;
    static bool overflow = false;

    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (overflow) {
                Serial.println(F("ERR: input too long."));
                overflow = false;
            } else if (len > 0) {
                buf[len] = '\0';
                char* end = buf;
                unsigned long val = strtoul(buf, &end, 10);
                if (end != buf) {
                    State s = (val & 1UL) ? State::Odd : State::Even;
                    applyState(s);
                    Serial.print(F("RX: "));
                    Serial.print(val);
                    Serial.print(F("  →  "));
                    Serial.println(stateLabel(s));
                } else {
                    Serial.println(F("ERR: not a valid integer."));
                }
            }
            len = 0;

        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            overflow = true;
            len = 0;
        }
    }
}
