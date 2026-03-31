// ─────────────────────────────────────────────────────────────────────────────
//  HW-1 │ LED Blink
//
//  D13 (built-in LED): 500 ms HIGH → 2000 ms LOW, repeat.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>

void setup() { pinMode(LED_BUILTIN, OUTPUT); }

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
}
