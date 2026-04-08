# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Arduino UNO homework assignments (HW-1 through HW-9) built with PlatformIO. Each homework is a standalone `.cpp` file in `src/` compiled via its own PlatformIO environment. The README and code comments are partly in Chinese.

## Build Commands

```bash
pio run -e <env>              # compile (envs: blink, state_machine, pwm, buzzer, w25qxx, ds1307, logger, uart, mpu6500)
pio run -e <env> -t upload    # compile + flash to board
pio device monitor -e <env>   # serial monitor (most envs use 57600 baud)
pio check                     # static analysis (clang-tidy: cert-*, clang-analyzer-*)
```

## Architecture

Each homework is a single self-contained `src/<name>.cpp` — no shared source files, no library code. The `platformio.ini` `[common]` section holds shared config; each `[env:<name>]` selects one `.cpp` via `build_src_filter`.

Hardware protocols (SPI, I2C, UART) are implemented with direct AVR register manipulation, grouped into namespaces within each file (e.g., `Rtc`, `Flash`, `Logger`, `Mpu`, `Filter`). ISRs handle time-critical work; the main loop handles serial I/O.

## Compiler Settings

- C++17 (`-std=gnu++17`), LTO enabled (`-flto`)
- `-Wall -Werror` — no warnings allowed
- CERT-compliant static analysis via clang-tidy

## Code Conventions

- LLVM-based formatting via `.clang-format`: 100-char columns, 4-space indent, `Type*` pointer style
- Qualifier order: `static`, `inline`, type, `const`, `volatile`
- File headers use `// ───` box comments with homework number and description
- Section separators: `// ── Label ──` within files
- `static constexpr` for compile-time constants; `volatile` for ISR-shared state
- PROGMEM for flash-stored data; `F()` macro for serial string literals
- No dynamic allocation; stack buffers with `snprintf`
- Boolean returns for hardware I/O success/failure; no exceptions
