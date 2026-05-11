# CrowPanel 5" ESP32-S3 HMI — LVGL 9 starter (PlatformIO / Windows)

A minimal, known-good starting point for the Elecrow CrowPanel 5-inch
ESP32-S3 HMI (800×480 RGB, GT911 touch) using **LVGL 9.2**, **LovyanGFX
1.2**, and the **Arduino-ESP32 2.0.14** core under **PlatformIO**.

## What's in here

```
crowpanel5-lvgl9/
├── platformio.ini                  # build config (toolchain pinned)
├── README.md                       # this file
├── include/
│   ├── LGFX_CrowPanel5.hpp         # panel + GT911 touch config
│   └── lv_conf.h                   # YOU CREATE THIS (see step 3 below)
└── src/
    └── main.cpp                    # LVGL 9 glue + tiny demo UI
```

## First-time setup

### 1. Tools (one-time, Windows)
- Install **VS Code** and the **PlatformIO IDE** extension.
- Install the **Silicon Labs CP210x VCP driver** so the board enumerates
  as a COM port.

### 2. Open the project
1. `File → Open Folder…` and pick the `crowpanel5-lvgl9` folder.
2. Wait for PlatformIO to install the toolchain and libraries
   (`platform = espressif32@6.7.0`, plus `lvgl` and `LovyanGFX`).
   First time can take a few minutes.

### 3. Create `lv_conf.h` (one-time)
LVGL ships a template config you must copy and tweak. **Do this once
after PlatformIO has finished downloading libraries**:

1. Find the file
   `.pio/libdeps/crowpanel5/lvgl/lv_conf_template.h`
   inside the project folder.
2. Copy it to `include/lv_conf.h`.
3. Open `include/lv_conf.h` and change these specific lines:

   ```c
   /* line near the very top: enable the file */
   #if 0          →   #if 1

   /* Color depth — match the panel */
   #define LV_COLOR_DEPTH 16

   /* Memory — put the LVGL heap in PSRAM */
   #define LV_USE_STDLIB_MALLOC    LV_STDLIB_CLIB     /* or BUILTIN */
   #define LV_MEM_SIZE            (96U * 1024U)      /* if BUILTIN */

   /* Tick source — we feed it from millis() in main.cpp */
   /* (Nothing to set; lv_tick_set_cb is the v9 way.)  */

   /* Fonts used by the demo UI */
   #define LV_FONT_MONTSERRAT_14   1
   #define LV_FONT_MONTSERRAT_28   1

   /* Optional: built-in demos for quick sanity checks */
   #define LV_USE_DEMO_WIDGETS     1
   ```

   Leave everything else at defaults. You can come back later and trim.

### 4. Build & flash
- Connect the board via USB-C.
- In the PlatformIO toolbar: **Build** (✓), then **Upload** (→).
- Open the **Serial Monitor** (🔌). You should see
  `CrowPanel 5 / LVGL 9 starter booting...` and then a dark screen
  with a title, a status line, and a "Press me" button. Tapping the
  button increments a counter on screen — that confirms display **and**
  touch are alive.

## Next steps

- **Design a UI in SquareLine Studio.** Create a project as
  "Arduino with TFT_eSPI", resolution 800×480, color depth 16-bit, and
  export. Copy the generated `ui/` folder into `src/`, then call
  `ui_init()` from `setup()` (after `lv_init()` but instead of
  `build_starter_ui()`).
- **Add CAN.** The ESP32-S3 TWAI controller can be routed to the SD
  pads — see the earlier message for the pinout and a starter snippet.
  Run TWAI on a separate FreeRTOS task and push frames into a queue;
  the LVGL task pops the queue and updates labels/tables.

## Why these specific versions?

- `platform = espressif32@6.7.0` pins Arduino-ESP32 to **2.0.14**.
  LovyanGFX 1.2's RGB-panel driver currently does **not** build cleanly
  against the 3.x core because `esp_lcd_*` APIs were rearranged. The
  Elecrow demos also target 2.0.14, so you stay aligned with what
  upstream knows works.
- `lvgl @ ^9.2.2` is the most recent 9.x line that's been thoroughly
  shaken out on ESP32-S3. 9.5 (Feb 2026) adds MPU-class features
  (NanoVG, glTF) you don't need on this hardware.
- `LovyanGFX @ ^1.2.0` ships the ESP32-S3 `Panel_RGB` + `Bus_RGB` +
  `Touch_GT911` you need; older 1.1.x versions did not.

## Common gotchas

| Symptom                                | Likely cause                                         |
|---|---|
| Black screen, board boots fine         | Wrong `psram_type` — must be `opi` on N4R8           |
| Display works, touch doesn't           | I2C address fight — try 0x5D instead of 0x14         |
| Tearing / wobbly horizontal lines      | `freq_write` too high in `LGFX_CrowPanel5.hpp`; drop to 12 MHz |
| `lv_conf.h not found` on build         | Forgot step 3, or didn't flip `#if 0` to `#if 1`     |
| Watchdog reset on boot                 | LVGL draw buffers couldn't allocate — PSRAM not enabled |
| Upload fails, "wrong chip detected"    | Hold BOOT, tap RESET, release BOOT, retry upload     |
