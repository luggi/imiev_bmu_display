# Mitsubishi i-MiEV BMS viewer — CrowPanel 5" v3.0 (PlatformIO)

A standalone CAN listener + display for snooping the BMU/CMU broadcasts on
a Mitsubishi i-MiEV (and clones: Peugeot iOn, Citroën C-Zero) battery pack.
Runs on the **Elecrow CrowPanel 5-inch v3.0** ESP32-S3 HMI (800×480 RGB,
GT911 touch) with **LVGL 9.2**, **LovyanGFX 1.2**, and **Arduino-ESP32
2.0.14** under PlatformIO.

Shows one CMU at a time with all 8 cell voltages, 6 temperatures, and the
balancing bitmask. Auto-follows whichever CMU is currently broadcasting,
or tap the on-screen arrows for manual navigation (lock for 5 s, then
resumes auto-follow).

## What's in here

```
crowpanel5/
├── platformio.ini                  # build config (toolchain pinned)
├── README.md                       # this file
├── include/
│   ├── LGFX_CrowPanel5.hpp         # panel + GT911 touch config (v3.0)
│   └── lv_conf.h                   # LVGL config (committed; no template copy needed)
└── src/
    └── main.cpp                    # TWAI driver + BMS decode + LVGL UI
```

## Hardware you need

| Item | Notes |
|---|---|
| Elecrow CrowPanel 5" **v3.0** | Older revisions use different touch wiring — see "v3.0 specifics" below |
| 3.3 V CAN transceiver | SN65HVD230, MCP2562FD, ISO1042… **not** 5 V parts — they'll brown out the ESP RX pin |
| 12 V bench supply or SLA | To wake the BMU. Plan for ~300 mA headroom (real draw not specified by Mitsubishi) |
| 8-pin connector to BMU | Pin 1 CAN-L, pin 4 +12 V, pin 5 CAN-H, pin 6 GND |

Transceiver wiring to the ESP32-S3:

| Transceiver | ESP32-S3 |
|---|---|
| CTX  (TX into transceiver) | **GPIO 10** |
| CRX  (RX out of transceiver) | **GPIO 13** |
| VCC | 3V3 from the CrowPanel |
| GND | GND from the CrowPanel **and** from the i-MiEV bus, tied together |
| CAN-H / CAN-L | BMU pins 5 / 1 |

The i-MiEV bus is already terminated at both ends; **do not add a 120 Ω
terminator** unless you're snooping outside the car with the BMU as the
only other node.

## First-time setup

### 1. Tools (Windows)
- **VS Code** + **PlatformIO IDE** extension.
- **Silicon Labs CP210x VCP** driver so the board enumerates as a COM port.

### 2. Open & build
1. `File → Open Folder…` and pick this directory.
2. PlatformIO downloads the toolchain and libs (`espressif32@6.7.0`,
   `lvgl@^9.2.2`, `LovyanGFX@^1.2.0`). First run takes a few minutes.
3. **Build** (✓) → **Upload** (→) on the PlatformIO toolbar.
4. Open **Serial Monitor** (🔌, 115200 baud). On boot you should see:
   ```
   CrowPanel 5 v3.0 / LVGL 9 / i-MiEV BMS viewer booting...
   I2C scan: 0x19 0x5D  (2 devices)
   CAN: 500 kbps  TX=GPIO10  RX=GPIO13
   Ready.
   ```
   `0x19` is the PCA9557 I/O expander; `0x5D` is the GT911 after reset.
   If you only see `0x19`, touch latched the wrong I²C address — see the
   gotchas table.

### 3. Try it without a CAN bus — test mode
At the top of [`src/main.cpp`](src/main.cpp):

```c
#define BMS_TEST_MODE 1   // 0 = real CAN, 1 = synthetic frames
```

With `BMS_TEST_MODE = 1`, the firmware skips TWAI init and runs an
internal task that feeds plausible-looking BMS frames through the real
decoder. The title shows `CMU N  [TEST]` so it's obvious you're not
looking at live data. Every cell color (healthy, near-top, out-of-range,
balancing, no-data) is exercised; CMUs 11 and 12 are treated as 4-cell
modules per spec so cells 5–8 stay grey.

## UI tour

```
┌──────────────────────────────────────────────────────┐
│ CMU 4                                          AUTO  │   <- montserrat_28 / badge
│ Pack: min 3.92 V  max 4.01 V  avg 3.96 V  delta 90 mV│   <- unscii_16 (bitmap, no AA)
│                                                      │
│ Cell 1:  3.951 V        T1:  21 C                    │
│ Cell 2:  3.948 V        T2:  22 C                    │
│ Cell 3:  3.952 V  BAL   T3:  21 C                    │
│ Cell 4:  3.961 V        T4:  23 C                    │
│ Cell 5:  3.955 V        T5:  22 C                    │
│ Cell 6:  3.949 V        T6:  24 C                    │
│ Cell 7:  3.962 V                                     │
│ Cell 8:  3.945 V        Balance: 3                   │
│                                                      │
│ CMU 4: last frame 87 ms ago (fresh)                  │
│ frames this CMU: 1240    pack frames: 14882          │
│                                                      │
│  [   <   ]                            [   >   ]      │
└──────────────────────────────────────────────────────┘
```

| Color | Meaning |
|---|---|
| Dark green | Healthy cell (3.30 V – 4.05 V, not balancing) |
| Navy | Balancing — bleed resistor active |
| Dark amber | Near top of charge (4.05 – 4.15 V) |
| Dark red | Out of range (< 3.30 V or > 4.15 V) |
| Light grey | No data yet for that cell / sensor |

The badge top-right is **green AUTO** when auto-follow is engaged
(latest CMU on screen) and **blue MANUAL** when you've tapped a nav
button (locks the view for 5 s, then resumes auto).

## Why these specific versions?

- `platform = espressif32@6.7.0` pins Arduino-ESP32 to **2.0.14**.
  LovyanGFX 1.2's RGB-panel driver currently does **not** build cleanly
  against the 3.x core because `esp_lcd_*` APIs were rearranged.
- `lvgl @ ^9.2.2` is the most recent 9.x line that's been thoroughly
  shaken out on ESP32-S3. 9.5 (Feb 2026) adds MPU-class features
  (NanoVG, glTF) that don't matter here.
- `LovyanGFX @ ^1.2.0` ships the ESP32-S3 `Panel_RGB` + `Bus_RGB` +
  `Touch_GT911` you need; older 1.1.x versions did not.
- TWAI driver is built into ESP-IDF — no extra `lib_deps` entry.

## v3.0 specifics (touch bring-up)

On the **v3.0** revision, the GT911's `RST` and `INT` lines are not
wired to ESP32 GPIOs — they pass through a **PCA9557 I²C I/O expander**
sharing the same I²C bus as the touch controller. LovyanGFX has no
concept of the expander, so [`crowpanel_v3_touch_bringup()`](src/main.cpp)
in `main.cpp` performs the reset/address-latch sequence manually before
`lcd.init()` runs:

1. PCA9557 `IO0` LOW (assert GT911 RST), `IO1` LOW (forces address `0x5D`)
2. 20 ms delay
3. `IO0` HIGH (release RST while INT is still LOW → GT911 latches `0x5D`)
4. 100 ms delay
5. `IO1` back to input (high-Z) so the GT911 can drive its INT line

LovyanGFX is configured with `pin_rst = -1` and `pin_int = -1` so it
leaves those lines alone afterwards.

If you have an older v1.x or v2.x board where RST/INT *are* on GPIOs 38
and 18, replace `crowpanel_v3_touch_bringup()` with a direct
`pinMode/digitalWrite` reset and put those GPIO numbers back into
`LGFX_CrowPanel5.hpp`.

## Common gotchas

| Symptom | Likely cause |
|---|---|
| Black screen, board boots fine | Wrong `psram_type` — must be `opi` on N4R8 |
| Display works, touch doesn't, `I2C scan` shows only `0x19` | Wrong revision detection — try `PCA9557_ADDR = 0x18` |
| `I2C scan` shows **nothing** | Wrong I²C pins, broken pull-ups, or 400 kHz too fast — try 100 kHz |
| Tearing / wobbly horizontal lines | `freq_write` too high in `LGFX_CrowPanel5.hpp`; drop to 12 MHz |
| Watchdog reset on boot | LVGL draw buffers couldn't allocate — PSRAM not enabled |
| Upload fails, "wrong chip detected" | Hold BOOT, tap RESET, release BOOT, retry upload |
| `twai_driver_install failed` | Pin conflict (10/13 already used) or transceiver not powered |
| All cells show grey `--` despite CAN being up | Bus is alive but BMU asleep — needs `+12 V` on pin 4 to wake |
| Cells flicker between values and `--` | Transceiver power or ground bounce — recheck wiring |

## Protocol reference

The decoder in [`decode_frame()`](src/main.cpp) implements:

- **CAN ID:** `0x600 + (CMU_id × 0x10) + pair_id`, where `CMU_id` is 1–12
  and `pair_id` is 1–4 (cell pairs).
- **Voltage:** `(byte_H << 8 | byte_L) × 0.005 + 2.1` V
- **Temperature:** `raw - 50` °C
- **Balance mask:** byte 1 of frame 1 — bit `i` = cell `i+1` actively
  bleeding.
- **4-cell modules:** frames 3 & 4 carry zeros for the missing cells;
  the UI renders those rows as grey `--`.
