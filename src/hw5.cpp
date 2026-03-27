// ─────────────────────────────────────────────────────────────────────────────
//  W25Q SPI Flash — 读写测试
//
//  Wiring (Arduino UNO):
//    D10 — CS   (手动 GPIO，不用硬件 SS)
//    D11 — MOSI
//    D12 — MISO
//    D13 — SCK
//
//  Test sequence:
//    1. Read JEDEC ID     → verify the chip is present and wired correctly
//    2. Erase sector 0    → 4 KB at 0x000000 (mandatory before any write)
//    3. Page Program      → write TEST_DATA into the erased sector
//    4. Read back         → byte-by-byte memcmp → PASS / FAIL
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
static constexpr uint8_t CS_PIN = 10;

// W25Q supports up to 104 MHz; 8 MHz is conservative and safe over breadboard.
// Mode 0 (CPOL=0, CPHA=0) matches W25Q datasheet.
//
// Not stored as a static SPISettings object: cert-err58-cpp flags any static
// with a non-constexpr constructor (SPISettings qualifies) because a throw
// during static initialisation cannot be caught.  Instead, the three
// parameters are kept as constexpr scalars and SPISettings is constructed
// inline at each beginTransaction() call — it is a trivially cheap value type.
static constexpr uint32_t kSpiClock = 8'000'000UL;
static constexpr uint8_t kSpiBitOrd = MSBFIRST;
static constexpr uint8_t kSpiDataMod = SPI_MODE0;

// ── W25Q instruction set (partial) ───────────────────────────────────────────
namespace cmd {
static constexpr uint8_t JEDEC_ID = 0x9F;         // read manufacturer + device ID (3 B)
static constexpr uint8_t READ_STATUS_REG1 = 0x05; // SR1: bit0 = BUSY, bit1 = WEL
static constexpr uint8_t WRITE_ENABLE = 0x06;     // set WEL; required before write/erase
static constexpr uint8_t SECTOR_ERASE_4K = 0x20;  // erase one 4 KB sector
static constexpr uint8_t PAGE_PROGRAM = 0x02;     // write up to 256 B into one page
static constexpr uint8_t READ_DATA = 0x03;        // read at full clock speed
} // namespace cmd

static constexpr uint8_t SR1_BUSY = 0x01;    // Status Register 1, BUSY bit
static constexpr uint8_t WINBOND_MFR = 0xEF; // expected JEDEC manufacturer byte

// ── Test parameters ───────────────────────────────────────────────────────────
static constexpr uint32_t TEST_ADDR = 0x000000UL;
static uint8_t const TEST_DATA[] = "SPI Flash Test 123"; // incl. '\0'
static constexpr uint8_t DATA_LEN = static_cast<uint8_t>(sizeof(TEST_DATA));

// ── Forward declarations ──────────────────────────────────────────────────────
static void send3ByteAddr(uint32_t addr);
static void waitReady();
static void writeEnable();
static uint32_t readJedecId();
static void sectorErase(uint32_t addr);
static void pageProgram(uint32_t addr, uint8_t const* data, uint8_t len);
static void readData(uint32_t addr, uint8_t* buf, uint8_t len);

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(57600);

    // Pull CS high *before* SPI.begin() to avoid a spurious falling edge on
    // power-up that could trigger an unintended command on the flash chip.
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();

    Serial.println(F("=== W25Q SPI Flash 读写测试 ==="));

    // ── Step 1: Identify chip ─────────────────────────────────────────────────
    uint32_t jedec = readJedecId();
    Serial.print(F("JEDEC ID : 0x"));
    Serial.println(jedec, HEX);

    if ((jedec >> 16) != WINBOND_MFR) {
        Serial.println(F("WARN: unexpected manufacturer byte — check wiring."));
    }

    // ── Step 2: Erase sector ──────────────────────────────────────────────────
    Serial.print(F("Erasing  : sector 0 (4 KB) ... "));
    sectorErase(TEST_ADDR);
    Serial.println(F("done"));

    // ── Step 3: Write ─────────────────────────────────────────────────────────
    Serial.print(F("Writing  : "));
    Serial.println(reinterpret_cast<char const*>(TEST_DATA));
    pageProgram(TEST_ADDR, TEST_DATA, DATA_LEN);

    // ── Step 4: Read back and verify ──────────────────────────────────────────
    uint8_t readBuf[DATA_LEN];
    readData(TEST_ADDR, readBuf, DATA_LEN);

    Serial.print(F("Read back: "));
    Serial.println(reinterpret_cast<char const*>(readBuf));

    bool pass = (memcmp(TEST_DATA, readBuf, DATA_LEN) == 0);
    Serial.println(pass ? F("Result   : PASS ✓") : F("Result   : FAIL ✗"));
    Serial.println(F("=== 测试结束 ==="));
}

void loop() {} // single-shot test; nothing to repeat

// ── Low-level helpers ─────────────────────────────────────────────────────────

// Emit a 24-bit address MSB-first.
// Must be called inside an active SPI transaction with CS asserted.
static void send3ByteAddr(uint32_t addr) {
    SPI.transfer(static_cast<uint8_t>((addr >> 16) & 0xFF));
    SPI.transfer(static_cast<uint8_t>((addr >> 8) & 0xFF));
    SPI.transfer(static_cast<uint8_t>(addr & 0xFF));
}

// Block until SR1 BUSY clears.
// The W25Q continuously outputs SR1 while CS is held low after the command
// byte, so we poll in a tight loop without re-sending the command each time.
static void waitReady() {
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::READ_STATUS_REG1);
    while (SPI.transfer(0xFF) & SR1_BUSY) {
    }
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

// Set the Write Enable Latch; must precede every write or erase command.
// WEL is automatically cleared by the chip after each write/erase completes.
static void writeEnable() {
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::WRITE_ENABLE);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

// Return the 3-byte JEDEC ID packed as 0x00MMTTCC
// (MM = manufacturer, TT = memory type, CC = capacity).
static uint32_t readJedecId() {
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::JEDEC_ID);
    uint32_t id = static_cast<uint32_t>(SPI.transfer(0xFF)) << 16;
    id |= static_cast<uint32_t>(SPI.transfer(0xFF)) << 8;
    id |= static_cast<uint32_t>(SPI.transfer(0xFF));
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return id;
}

// Erase one 4 KB sector containing addr.
// Blocks until the erase completes (typically 50–400 ms per datasheet).
static void sectorErase(uint32_t addr) {
    writeEnable();
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::SECTOR_ERASE_4K);
    send3ByteAddr(addr);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    waitReady();
}

// Write len bytes (≤ 256) to addr.
// The destination sector must have been erased first (NOR Flash can only
// flip bits from 1 → 0; erase resets all bits back to 1).
static void pageProgram(uint32_t addr, uint8_t const* data, uint8_t len) {
    writeEnable();
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::PAGE_PROGRAM);
    send3ByteAddr(addr);
    for (uint8_t i = 0; i < len; ++i) {
        SPI.transfer(data[i]);
    }
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    waitReady();
}

// Read len bytes from addr into buf.
static void readData(uint32_t addr, uint8_t* buf, uint8_t len) {
    SPI.beginTransaction(SPISettings(kSpiClock, kSpiBitOrd, kSpiDataMod));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(cmd::READ_DATA);
    send3ByteAddr(addr);
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = SPI.transfer(0xFF);
    }
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}
