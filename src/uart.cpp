// ─────────────────────────────────────────────────────────────────────────────
//  UART 串口输入体会
//  Part A — Serial.read()    : 发送 "a <整数>"      (支持负数)
//  Part B — Serial.parseInt(): 发送 "b <n1> <n2> …" (以回车结束)
//  57600 baud, Both NL & CR
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>

// ══ Part A: Serial.read() 手动解析多位整数 ════════════════════════════════
// 三状态机 + 霍纳累积: value = value * 10 + (c - '0')
// 每次 loop() 只处理缓冲区现有字节, 完全非阻塞.

namespace PartA {

enum class State : uint8_t { IDLE, SKIP_SP, COLLECT };
static State state = State::IDLE;
static int32_t value = 0;
static bool negative = false;

static void reset() {
    state = State::IDLE;
    value = 0;
    negative = false;
}

static void tick() {
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        switch (state) {

        case State::IDLE:
            if (c == 'a')
                state = State::SKIP_SP;
            break;

        case State::SKIP_SP:
            if (c == ' ' || c == '\t') { /* skip */
            } else if (c == '-') {
                negative = true;
                state = State::COLLECT;
            } else if (c >= '0' && c <= '9') {
                value = c - '0';
                state = State::COLLECT;
            } else if (c == '\n' || c == '\r') {
                Serial.println(F("[A] ERR: no number"));
                reset();
            }
            break;

        case State::COLLECT:
            if (c >= '0' && c <= '9') {
                if (value > 214748364L) {
                    Serial.println(F("[A] ERR: overflow"));
                    reset();
                    break;
                }
                value = value * 10 + (c - '0');
            } else if (c == '\n' || c == '\r') {
                Serial.print(F("[A] read() result: "));
                Serial.println(negative ? -value : value);
                reset();
            } else {
                Serial.print(F("[A] ERR: unexpected '"));
                Serial.print(c);
                Serial.println('\'');
                reset();
            }
            break;
        }
    }
}

} // namespace PartA

// ══ Part B: Serial.parseInt() 读取多个整数 ════════════════════════════════
// parseInt() 自动跳过非数字字符, 依赖超时(200 ms)判断序列结束.
// 局限: 超时返回 0 与真实数据 0 无法区分, 用"缓冲区为空"作近似判断.

namespace PartB {

static bool waiting = true;

static void configure() { Serial.setTimeout(200); }

static void tick() {
    // 等待命令头 'b'
    if (waiting) {
        while (Serial.available()) {
            char c = static_cast<char>(Serial.peek());
            if (c == 'b') {
                Serial.read();
                waiting = false;
                break;
            } else if (c == '\n' || c == '\r') {
                Serial.read();
            } else {
                break;
            } // 留给 Part A
        }
        return;
    }

    // 循环读取整数, 直到行尾或超时
    Serial.println(F("[B] reading integers..."));
    uint8_t count = 0;
    int32_t sum = 0;

    while (count < 8) {
        // 消耗行尾字符; 若缓冲区已空且有数据则结束
        while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r'))
            Serial.read();
        if (!Serial.available() && count > 0)
            break;

        int32_t v = Serial.parseInt();
        if (v == 0 && !Serial.available())
            break; // 超时返回的 0

        Serial.print(F("  got: "));
        Serial.println(v);
        sum += v;
        ++count;
    }

    if (count == 0)
        Serial.println(F("[B] ERR: no numbers"));
    else {
        Serial.print(F("[B] parseInt count="));
        Serial.print(count);
        Serial.print(F("  sum="));
        Serial.println(sum);
    }

    waiting = true;
}

} // namespace PartB

// ══ Arduino 入口 ══════════════════════════════════════════════════════════

void setup() {
    Serial.begin(57600);
    PartB::configure();
    Serial.println(F("━━━ UART Input Demo ━━━"));
    Serial.println(F("a <int>          e.g. a -9876"));
    Serial.println(F("b <n1> <n2> ...  e.g. b 10 200 3000"));
}

void loop() {
    PartA::tick();
    PartB::tick();
}
