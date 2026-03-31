// ─────────────────────────────────────────────────────────────────────────────
//  HW-7 │ Persistent Time-Stamped Event Logger
//
//  DS1307  (I2C A4/A5) — real-time clock, provides timestamps
//  W25Qxx  (SPI D10–D13) — NOR Flash, stores log entries across power cycles
//
//  Flash layout (scan-based, no metadata sector):
//    Sector 0  0x000000  ┐
//    Sector 1  0x001000  │ log data — sequential 8-byte LogEntry structs
//    …         …         ┘ (LOG_SECTORS sectors = 4096 entries max)
//
//  LogEntry (8 bytes — aligns to Flash page and sector boundaries exactly):
//    year month day hour minute second id checksum
//    checksum = XOR of the preceding 7 bytes
//
//  Scan invariant:
//    Entries are written strictly sequentially.  An erased NOR Flash slot
//    reads as all-0xFF.  On power-up, a binary search over [0, MAX_ENTRIES)
//    finds the boundary between written and erased slots in O(log N) reads
//    (12 reads for 4096 entries) — no metadata sector, no per-write overhead.
//
//    Edge case — interrupted write (power loss mid-program):
//    The partial entry will not pass checksum validation and shows as
//    CORRUPTED in `dump`, but the scan still finds the correct next-write
//    position because the partial bytes are not all-0xFF.
//
//  Serial commands (57600 baud):
//    log    – read RTC, stamp & store one entry
//    dump   – print all stored entries to serial
//    clear  – erase all sectors (restores all-0xFF state)
//    status – entry count, capacity, next-write address
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ══ Shared types ══════════════════════════════════════════════════════════════

struct DateTime {
    uint8_t second, minute, hour;
    uint8_t dayOfWeek; // 1 = Sunday … 7 = Saturday (DS1307 convention)
    uint8_t day, month;
    uint8_t year; // 0–99, offset from 2000
};

// 8 bytes: fits 32 entries per 256-byte Flash page, 512 per 4 KB sector.
struct LogEntry {
    uint8_t year, month, day;
    uint8_t hour, minute, second;
    uint8_t id;       // wrapping sequential counter (0–255)
    uint8_t checksum; // XOR of bytes [year … id]
};

static_assert(sizeof(LogEntry) == 8, "LogEntry must be exactly 8 bytes");

// ══ DS1307 (I2C RTC) ══════════════════════════════════════════════════════════

namespace Rtc {

static constexpr uint8_t ADDR = 0x68;
static constexpr uint8_t REG_SECONDS = 0x00;
static constexpr uint8_t REG_COUNT = 7;
static constexpr uint8_t MASK_SECONDS = 0x7F; // bit7 = Clock Halt flag
static constexpr uint8_t MASK_HOURS = 0x3F;   // bit6 = 12/24 mode select

static constexpr uint8_t toBcd(uint8_t v) {
    return static_cast<uint8_t>(((v / 10) << 4) | (v % 10));
}
static constexpr uint8_t fromBcd(uint8_t v) {
    return static_cast<uint8_t>(((v >> 4) * 10) + (v & 0x0F));
}

// Uses repeated START (endTransmission(false)) → requestFrom protocol.
// Do NOT call endTransmission() after requestFrom(); it issues its own STOP.
static bool read(DateTime& dt) {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_SECONDS);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom(ADDR, REG_COUNT) != REG_COUNT)
        return false;
    dt.second = fromBcd(static_cast<uint8_t>(Wire.read()) & MASK_SECONDS);
    dt.minute = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.hour = fromBcd(static_cast<uint8_t>(Wire.read()) & MASK_HOURS);
    dt.dayOfWeek = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.day = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.month = fromBcd(static_cast<uint8_t>(Wire.read()));
    dt.year = fromBcd(static_cast<uint8_t>(Wire.read()));
    return true;
}

[[maybe_unused]] static bool set(DateTime const& dt) {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_SECONDS);
    // Writing toBcd(second) with bit7 = 0 also clears Clock Halt,
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

} // namespace Rtc

// ══ W25Q (SPI NOR Flash) ══════════════════════════════════════════════════════

namespace Flash {

static constexpr uint8_t CS_PIN = 10;
static constexpr uint32_t SPI_CLK = 8'000'000UL; // safe over breadboard
static constexpr uint8_t SPI_ORD = MSBFIRST;
static constexpr uint8_t SPI_MODE = SPI_MODE0; // W25Q: CPOL=0, CPHA=0

namespace cmd {
static constexpr uint8_t READ_SR1 = 0x05;
static constexpr uint8_t WRITE_ENABLE = 0x06;
static constexpr uint8_t SECTOR_ERASE = 0x20;
static constexpr uint8_t PAGE_PROGRAM = 0x02;
static constexpr uint8_t READ_DATA = 0x03;
} // namespace cmd

static void csAssert() { digitalWrite(CS_PIN, LOW); }
static void csRelease() { digitalWrite(CS_PIN, HIGH); }

static void sendAddr(uint32_t addr) {
    SPI.transfer(static_cast<uint8_t>((addr >> 16) & 0xFF));
    SPI.transfer(static_cast<uint8_t>((addr >> 8) & 0xFF));
    SPI.transfer(static_cast<uint8_t>(addr & 0xFF));
}

// Poll SR1 BUSY bit; blocks until the chip is idle.
static void waitReady() {
    SPI.beginTransaction(SPISettings(SPI_CLK, SPI_ORD, SPI_MODE));
    csAssert();
    SPI.transfer(cmd::READ_SR1);
    while (SPI.transfer(0xFF) & 0x01) {
    }
    csRelease();
    SPI.endTransaction();
}

static void writeEnable() {
    SPI.beginTransaction(SPISettings(SPI_CLK, SPI_ORD, SPI_MODE));
    csAssert();
    SPI.transfer(cmd::WRITE_ENABLE);
    csRelease();
    SPI.endTransaction();
}

// Erase the 4 KB sector containing addr.
static void eraseSector(uint32_t addr) {
    writeEnable();
    SPI.beginTransaction(SPISettings(SPI_CLK, SPI_ORD, SPI_MODE));
    csAssert();
    SPI.transfer(cmd::SECTOR_ERASE);
    sendAddr(addr);
    csRelease();
    SPI.endTransaction();
    waitReady();
}

// Write len bytes (≤ 256, must not cross a 256-byte page boundary).
static void write(uint32_t addr, uint8_t const* data, uint8_t len) {
    writeEnable();
    SPI.beginTransaction(SPISettings(SPI_CLK, SPI_ORD, SPI_MODE));
    csAssert();
    SPI.transfer(cmd::PAGE_PROGRAM);
    sendAddr(addr);
    for (uint8_t i = 0; i < len; ++i)
        SPI.transfer(data[i]);
    csRelease();
    SPI.endTransaction();
    waitReady();
}

static void read(uint32_t addr, uint8_t* buf, uint8_t len) {
    SPI.beginTransaction(SPISettings(SPI_CLK, SPI_ORD, SPI_MODE));
    csAssert();
    SPI.transfer(cmd::READ_DATA);
    sendAddr(addr);
    for (uint8_t i = 0; i < len; ++i)
        buf[i] = SPI.transfer(0xFF);
    csRelease();
    SPI.endTransaction();
}

static void init() {
    // Pull CS high before SPI.begin() to avoid a spurious command on power-up.
    pinMode(CS_PIN, OUTPUT);
    csRelease();
    SPI.begin();
}

} // namespace Flash

// ══ Logger ════════════════════════════════════════════════════════════════════

namespace Logger {

static constexpr uint32_t DATA_START = 0x000000UL;  // sector 0, no metadata sector
static constexpr uint32_t SECTOR_SIZE = 0x001000UL; // 4 KB
static constexpr uint8_t LOG_SECTORS = 8;
static constexpr uint32_t ENTRIES_PER_SECTOR = SECTOR_SIZE / sizeof(LogEntry); // 512
static constexpr uint32_t MAX_ENTRIES = LOG_SECTORS * ENTRIES_PER_SECTOR;      // 4096

static uint32_t gCount = 0; // index of the next free slot; set once by scanCount()

// ── Helpers ───────────────────────────────────────────────────────────────────

static uint8_t entryChecksum(LogEntry const& e) {
    return e.year ^ e.month ^ e.day ^ e.hour ^ e.minute ^ e.second ^ e.id;
}

static uint32_t entryAddr(uint32_t idx) { return DATA_START + idx * sizeof(LogEntry); }

// An erased NOR Flash slot reads as all-0xFF.
// Valid timestamps can never have all 8 bytes as 0xFF simultaneously
// (that would require year=255, month=255, day=255 … checksum=0xFF).
static bool isErased(LogEntry const& e) {
    auto const* b = reinterpret_cast<uint8_t const*>(&e);
    for (uint8_t i = 0; i < sizeof(LogEntry); ++i) {
        if (b[i] != 0xFF)
            return false;
    }
    return true;
}

// ── Scan ─────────────────────────────────────────────────────────────────────

// Binary search for the first erased slot.
//
// Invariant maintained throughout: every entry in [0, lo) is written
// (valid or corrupted), every entry in [hi, MAX_ENTRIES) is erased.
// The search converges when lo == hi, which is gCount.
//
// Cost: ceil(log2(MAX_ENTRIES)) = 12 Flash reads for 4096 entries,
// regardless of how many entries have been logged — O(log N) vs O(N)
// for a linear scan.
static uint32_t scanCount() {
    uint32_t lo = 0;
    uint32_t hi = MAX_ENTRIES;

    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        LogEntry e;
        Flash::read(entryAddr(mid), reinterpret_cast<uint8_t*>(&e), sizeof(LogEntry));

        if (isErased(e)) {
            hi = mid; // mid and everything above is erased → search lower half
        } else {
            lo = mid + 1; // mid is written → search upper half
        }
    }

    return lo;
}

// ── Public API ────────────────────────────────────────────────────────────────

static void init() {
    gCount = scanCount();
    Serial.print(F("LOG: scan found "));
    Serial.print(gCount);
    Serial.print(F(" existing entries ("));
    Serial.print(MAX_ENTRIES - gCount);
    Serial.println(F(" free)"));
}

// Append one timestamped entry.  Returns false if Flash is full.
// Erases a sector exactly once, on the first write to that sector.
// No metadata write — the scan on next boot rediscovers gCount from Flash state.
static bool append(DateTime const& dt) {
    if (gCount >= MAX_ENTRIES)
        return false;

    // First write to a sector: erase it (NOR Flash can only flip 1→0;
    // erase resets all bits back to 1 so new data can be written cleanly).
    if (gCount % ENTRIES_PER_SECTOR == 0) {
        Flash::eraseSector(entryAddr(gCount));
    }

    LogEntry e;
    e.year = dt.year;
    e.month = dt.month;
    e.day = dt.day;
    e.hour = dt.hour;
    e.minute = dt.minute;
    e.second = dt.second;
    e.id = static_cast<uint8_t>(gCount & 0xFF);
    e.checksum = entryChecksum(e);

    Flash::write(entryAddr(gCount), reinterpret_cast<uint8_t const*>(&e), sizeof(LogEntry));

    ++gCount;
    // No writeMeta() call — the written entry itself is the persistent record.
    return true;
}

// Print a single entry from Flash; validates checksum before printing.
static void printEntry(uint32_t idx) {
    LogEntry e;
    Flash::read(entryAddr(idx), reinterpret_cast<uint8_t*>(&e), sizeof(LogEntry));

    if (e.checksum != entryChecksum(e)) {
        char buf[24];
        int n = snprintf(buf, sizeof(buf), "  [%4lu] CORRUPTED", static_cast<unsigned long>(idx));
        if (n > 0 && static_cast<size_t>(n) < sizeof(buf))
            Serial.println(buf);
        return;
    }

    // "  [4095] 2099-12-31 23:59:59  id=255" → 38 chars + '\0' → 48 is safe
    char buf[48];
    int n = snprintf(buf, sizeof(buf), "  [%4lu] 20%02u-%02u-%02u %02u:%02u:%02u  id=%u",
                     static_cast<unsigned long>(idx), e.year, e.month, e.day, e.hour, e.minute,
                     e.second, e.id);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
        Serial.println(F("  ERR: format overflow"));
        return;
    }
    Serial.println(buf);
}

static void dump() {
    if (gCount == 0) {
        Serial.println(F("LOG: no entries"));
        return;
    }
    Serial.print(F("LOG: "));
    Serial.print(gCount);
    Serial.println(F(" entries:"));
    for (uint32_t i = 0; i < gCount; ++i)
        printEntry(i);
}

static void clear() {
    Serial.print(F("LOG: erasing all sectors... "));
    for (uint8_t s = 0; s < LOG_SECTORS; ++s) {
        Flash::eraseSector(DATA_START + static_cast<uint32_t>(s) * SECTOR_SIZE);
    }
    gCount = 0;
    // Flash is now all-0xFF; the next scanCount() will return 0 correctly.
    Serial.println(F("done"));
}

static void status() {
    // "LOG: 4096 / 4096 entries  next @ 0x008000" → 43 chars + '\0' → 48 safe
    char buf[48];
    int n = snprintf(buf, sizeof(buf), "LOG: %lu / %lu entries  next @ 0x%06lX",
                     static_cast<unsigned long>(gCount), static_cast<unsigned long>(MAX_ENTRIES),
                     static_cast<unsigned long>(entryAddr(gCount)));
    if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
        Serial.println(F("ERR: format overflow"));
        return;
    }
    Serial.println(buf);
}

} // namespace Logger

// ══ Serial command parser ═════════════════════════════════════════════════════

static void handleCommand(char const* cmd) {
    if (strcmp(cmd, "log") == 0) {
        DateTime dt{};
        if (!Rtc::read(dt)) {
            Serial.println(F("ERR: RTC read failed"));
            return;
        }
        if (!Logger::append(dt)) {
            Serial.println(F("ERR: Flash full"));
            return;
        }

        // "OK: #4095  2099-12-31 23:59:59" → 32 chars + '\0' → 40 safe
        char buf[40];
        int n = snprintf(buf, sizeof(buf), "OK: #%lu  20%02u-%02u-%02u %02u:%02u:%02u",
                         static_cast<unsigned long>(Logger::gCount - 1), dt.year, dt.month, dt.day,
                         dt.hour, dt.minute, dt.second);
        if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
            Serial.println(F("OK: logged (format err)"));
            return;
        }
        Serial.println(buf);

    } else if (strcmp(cmd, "dump") == 0) {
        Logger::dump();
    } else if (strcmp(cmd, "clear") == 0) {
        Logger::clear();
    } else if (strcmp(cmd, "status") == 0) {
        Logger::status();
    } else {
        Serial.println(F("Commands: log  dump  clear  status"));
    }
}

// ══ Arduino entry points ══════════════════════════════════════════════════════

void setup() {
    Serial.begin(57600);
    Wire.begin();
    Flash::init();

    Serial.println(F("━━━ Persistent Event Logger ━━━"));
    Serial.println(F("DS1307 + W25Q  (scan-based)"));

    // ── Optional: set RTC once, then re-comment and re-flash ─────────────────
    // The DS1307 keeps time on its coin cell; setting it on every boot would
    // reset the clock each time the board is powered.  Workflow:
    //   1. Fill in the current time below and uncomment.
    //   2. Flash once → clock is set.
    //   3. Re-comment and flash again → clock runs freely from here on.
    //
    // constexpr DateTime kNow{ .second=0, .minute=0, .hour=12,
    //                          .dayOfWeek=5, .day=27, .month=3, .year=26 };
    // if (!Rtc::set(kNow)) Serial.println(F("ERR: RTC set failed"));

    Logger::init();
    Logger::status();
    Serial.println(F("Ready.\n"));
}

void loop() {
    static char buf[16];
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
}
