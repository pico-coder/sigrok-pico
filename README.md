# sigrok-pico

Use a Raspberry Pi PICO (RP2040) as a logic analyzer and oscilloscope with sigrok.

## Status
=======
Please start with the Getting Started page : [github.com/pico-coder/sigrok-pico/blob/main/GettingStarted.md](https://github.com/pico-coder/sigrok-pico/blob/main/USER_GUIDE.md#getting-started)

**Merged to mainline sigrok** (September 2023)

Install from [sigrok.org/downloads](https://sigrok.org/wiki/Downloads). PulseView 0.4.2 and sigrok-cli 0.7.2 do not support sigrok-pico.

## Quick Links

| Document | Description |
|----------|-------------|
| [USER_GUIDE.md](USER_GUIDE.md) | Getting started and analyzer operations |
| [TECHNICAL.md](TECHNICAL.md) | Serial protocol and build instructions |
| [pulseview/Readme.md](pulseview/Readme.md) | Windows installer |

## Directory Structure

```
sigrok-pico/
├── pico_pgen/              # Digital function generator for testing
├── pico_sdk_sigrok/        # RP2040 firmware (see release/ for UF2 files)
└── pulseview/              # Windows installer (unofficial)
```

## Overview

This project implements a sigrok driver for the Raspberry Pi PICO RP2040 using the PICO SDK CDC serial library. It works with both **PulseView** (GUI) and **sigrok-cli** (command-line):

- **21 digital channels** (D2-D22)
- **3 analog channels** (A0-A2)
- **Mixed-mode capture** (combined digital + analog)

### Using with PulseView

PulseView is the recommended graphical interface for sigrok-pico:

1. Install PulseView from [sigrok.org/downloads](https://sigrok.org/wiki/Downloads)
2. Flash the PICO with the appropriate UF2 firmware
3. In PulseView, select "raspberrypi_pico" driver and configure the serial port

> **Note**: PulseView 0.4.2 does not support sigrok-pico. Use a newer version or the [unofficial Windows installer](pulseview/Readme.md).

## Firmware

### Precompiled

Pre-compiled UF2 files are available in [`pico_sdk_sigrok/release/`](pico_sdk_sigrok/release/):

| File | Description |
|------|-------------|
| pico_baseline.uf2 | Standard firmware |
| pico_dig26.uf2 | 26-channel digital |
| pico_dig32.uf2 | 32-channel digital |
| pico2_*.uf2 | PICO 2 variants |

### Build Your Own

Building is straightforward using the VSCode extension:

1. Install the "Raspberry Pi Pico Project" extension in VSCode
2. Import the project using the extension
3. Select the latest SDK version when prompted
4. Select the board type (e.g., `pico` or `pico2`)
5. Click "Run Project (USB)" to build and flash

For command-line builds and more details, see [TECHNICAL.md](TECHNICAL.md).

## License

See [LICENSE](LICENSE) for details.
