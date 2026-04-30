# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Firmware for the **AirGradient Open Air Max** air quality monitor. Built on **ESP-IDF v5.5** (installed at `~/esp/v5.5.2/esp-idf/`), targeting the **ESP32-C6-WROOM-1** module. The device uses deep sleep between measurement cycles and supports both Cellular and Wi-Fi transmission modes.

## PCB design reference

The hardware design, schematics, and IC datasheets for this firmware live in a separate repo:
- **PCB project:** `~/Documents/airgradient_pcb/openair-max-pcb/`
- **Schematic:** `~/Documents/airgradient_pcb/openair-max-pcb/One-Air-Max.kicad_sch`
- **Datasheets:** `~/Documents/airgradient_pcb/openair-max-pcb/Doc/` (organized by `ic/`, `module/`, `battery/`)

When answering questions about register-level behavior, pin connections, or IC configuration, proactively read the relevant datasheet from that path before responding. Key datasheets:

| IC | Path |
|----|------|
| BQ25798 (charger) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/bq25798.pdf` |
| BQ76905 / BQ7790512 (BMS) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/bq76905.pdf` |
| TPS2117 (power mux) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/tps2117.pdf` |
| TPS27081A (load switch) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/tps27081a.pdf` |
| TPS62902/TPS62903 (buck) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/tps62902.pdf` |
| WK2132 (I2C→dual UART) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/wk2132.pdf` |
| AT24CM02 (EEPROM) | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/ic/at24cm02.pdf` |
| ESP32-C6-WROOM | `~/Documents/airgradient_pcb/openair-max-pcb/Doc/module/esp32-c6-wroom-1_datasheet.pdf` |

## Build commands

ESP-IDF must be sourced into the shell before any `idf.py` command — the toolchain, Python env, and `IDF_PATH` are all set by `export.sh`. Source it once per shell session:

```bash
source ~/esp/v5.5.2/esp-idf/export.sh
```

Then from the project root (`~/Documents/airgradient_code/openair-max/`):

```bash
idf.py build                        # build
idf.py -p <PORT> flash monitor      # flash and open serial monitor
idf.py -p <PORT> monitor            # monitor only
idf.py fullclean                    # nuke build/ if CMake gets confused
```

One-liner that works from any cwd (sources IDF, then builds in this project):
```bash
source ~/esp/v5.5.2/esp-idf/export.sh && \
  cd ~/Documents/airgradient_code/openair-max && idf.py build
```

Submodules must be initialized before the first build:
```bash
git submodule update --init --recursive
```

Target chip is `esp32c6` (set in `sdkconfig`); do not run `idf.py set-target` unless intentionally switching SoC.

## Architecture

### Entry point & sleep loop
`main/max.cpp` — device wakes from deep sleep, runs one measurement + transmission cycle, then sleeps again. `RTC_DATA_ATTR` variables persist across sleep cycles (wake counter, cache index, measure interval).

### Key config
`main/MaxConfig.h` — all GPIO assignments, I2C addresses, timing constants, and feature flags.

Notable pins:
- `IO_BMS_BATT_SEL` (GPIO18): **HIGH = LFP, LOW = Li-ion** — set before BMS init to select chemistry
- `IO_BMS_ALERT` (GPIO19): BMS fault alert input
- `EN_CO2`, `EN_PMS1`, `EN_PMS2`, `EN_ALPHASENSE`: power enable for each sensor
- `IO_WDT` (GPIO2): external hardware watchdog kick

### Components (`components/`)

| Component | Purpose |
|-----------|---------|
| `BQ25672/` | Charger IC driver (note: v4.0 board uses BQ25798 — same register map, BQ25672 is a subset) |
| `BQ76905/` | BMS driver |
| `ADS1115/` | ADC for battery/solar voltage |
| `PMS/` | Plantower PM sensor driver |
| `Sunlight/` | SenseAir Sunrise CO2 sensor driver |
| `AlphaSenseSensor/` | Alphasense electrochemical gas sensor |
| `AirgradientSerial/` | UART abstraction over WK2132 I2C→UART bridge |
| `WiFiManager/` | Wi-Fi connection management |
| `UdpLogger/` | Debug telemetry over UDP (see below) |
| `AirgradientClient` | HTTP/MQTT client (submodule) |
| `AirgradientOTA` | OTA update logic (submodule) |

### Measurement & transmission cycle
- **Measure** every 3 minutes (cellular: buffer 3 sets → transmit every 9 min; Wi-Fi: transmit each set)
- **Full transmission cycle** every 30 wake-ups (~1 hour): fetch remote config + check OTA
- Missed transmissions are cached in `PayloadCache` (RAM only, lost on power cycle)

### Debug UDP logging
Controlled by `#define AG_DEBUG_UDP_LOG` in `MaxConfig.h`. When enabled, the device connects to a 2.4 GHz Wi-Fi AP in parallel with cellular and mirrors all `ESP_LOGx` output + telemetry JSON to a UDP host. Comment out to disable for production builds.

## v5.0 planned changes

- **LFP battery support** — `IO_BMS_BATT_SEL` (GPIO18) already wired; firmware must set HIGH for LFP before BMS init. BQ76905 OV/UV thresholds and BQ25798 charge voltage must be reconfigured per chemistry (LFP: 3.65V/cell charge, 2.5V cutoff vs Li-ion: 4.2V/cell).
- Power switch, ADC for VBAT, power consumption reduction, technician display — see Jira AGFW-619.
