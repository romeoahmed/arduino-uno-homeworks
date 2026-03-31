// ─────────────────────────────────────────────────────────────────────────────
//  HW-4 │ 《小星星》Player  (passive buzzer + pause button)
//
//  Wiring:
//    D8  — passive buzzer (driven by tone())
//    D9  — push-button to GND (internal pull-up enabled)
//
//  Behaviour:
//    • Press button  →  toggle Play / Pause
//    • Pausing stops the tone immediately; resuming restarts the current note.
//    • Song loops indefinitely.
//
//  tone() uses Timer2 on the UNO, leaving Timer0 (millis) untouched.
// ─────────────────────────────────────────────────────────────────────────────
#include "pitches.h"
#include <Arduino.h>

// ── Pin assignments ───────────────────────────────────────────────────────────
static constexpr uint8_t BUZZER_PIN = 8;
static constexpr uint8_t BUTTON_PIN = 9;

// ── Timing ───────────────────────────────────────────────────────────────────
static constexpr uint8_t NOTE_GAP_MS = 30; // silence between notes (ms)
static constexpr uint8_t DEBOUNCE_MS = 20; // button debounce window (ms)

// ── Melody (NOTE_* from pitches.h, duration ms) ───────────────────────────────
//  REST = 0 (not defined in pitches.h; silence)
//  ♩ ≈ 400 ms  (≈ 150 BPM)
#define REST 0

struct Note {
    uint16_t freq;
    uint16_t dur;
};

static Note const MELODY[] PROGMEM = {
    // ── 一闪一闪亮晶晶 ─────────────────────────────────────────
    {NOTE_C4, 400},
    {NOTE_C4, 400},
    {NOTE_G4, 400},
    {NOTE_G4, 400},
    {NOTE_A4, 400},
    {NOTE_A4, 400},
    {NOTE_G4, 800},
    // ── 满天都是小星星 ─────────────────────────────────────────
    {NOTE_F4, 400},
    {NOTE_F4, 400},
    {NOTE_E4, 400},
    {NOTE_E4, 400},
    {NOTE_D4, 400},
    {NOTE_D4, 400},
    {NOTE_C4, 800},
    // ── 挂在天上放光明 ─────────────────────────────────────────
    {NOTE_G4, 400},
    {NOTE_G4, 400},
    {NOTE_F4, 400},
    {NOTE_F4, 400},
    {NOTE_E4, 400},
    {NOTE_E4, 400},
    {NOTE_D4, 800},
    // ── 好像许多小眼睛 ─────────────────────────────────────────
    {NOTE_G4, 400},
    {NOTE_G4, 400},
    {NOTE_F4, 400},
    {NOTE_F4, 400},
    {NOTE_E4, 400},
    {NOTE_E4, 400},
    {NOTE_D4, 800},
    // ── 一闪一闪亮晶晶 ─────────────────────────────────────────
    {NOTE_C4, 400},
    {NOTE_C4, 400},
    {NOTE_G4, 400},
    {NOTE_G4, 400},
    {NOTE_A4, 400},
    {NOTE_A4, 400},
    {NOTE_G4, 800},
    // ── 满天都是小星星 ─────────────────────────────────────────
    {NOTE_F4, 400},
    {NOTE_F4, 400},
    {NOTE_E4, 400},
    {NOTE_E4, 400},
    {NOTE_D4, 400},
    {NOTE_D4, 400},
    {NOTE_C4, 1200}, // last note held longer
};

static constexpr uint8_t MELODY_LEN = sizeof(MELODY) / sizeof(MELODY[0]);

// ── Read one note from PROGMEM ────────────────────────────────────────────────
static Note noteAt(uint8_t idx) {
    Note n;
    memcpy_P(&n, &MELODY[idx], sizeof(Note));
    return n;
}

// ── Debounced button: returns true exactly once per physical press ────────────
static bool buttonEdge() {
    static uint32_t lastChange = 0;
    static bool lastState = HIGH;

    bool cur = digitalRead(BUTTON_PIN);
    if (cur != lastState) {
        lastChange = millis();
        lastState = cur;
    }

    static bool confirmed = false;
    bool pressed = (lastState == LOW) && ((millis() - lastChange) >= DEBOUNCE_MS);
    bool edge = pressed && !confirmed;
    confirmed = pressed;
    return edge;
}

// ── Start playing the note at idx; returns the timestamp when it should end ───
static uint32_t startNote(uint8_t idx) {
    Note n = noteAt(idx);
    if (n.freq != REST) {
        tone(BUZZER_PIN, n.freq);
    } else {
        noTone(BUZZER_PIN);
    }
    return millis() + n.dur;
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    static bool playing = false;
    static uint8_t noteIdx = 0;
    static uint32_t noteEnd = 0;

    // ── Button: toggle play / pause ──────────────────────────────────────────
    if (buttonEdge()) {
        playing = !playing;
        if (playing) {
            noteEnd = startNote(noteIdx);
        } else {
            noTone(BUZZER_PIN);
        }
    }

    // ── Advance melody while playing ─────────────────────────────────────────
    if (playing && millis() >= noteEnd) {
        noTone(BUZZER_PIN);
        delay(NOTE_GAP_MS);
        noteIdx = (noteIdx + 1) % MELODY_LEN;
        noteEnd = startNote(noteIdx);
    }
}
