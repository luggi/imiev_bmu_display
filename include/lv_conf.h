/**
 * @file lv_conf.h
 * Minimal LVGL v9.2 configuration for CrowPanel 5" ESP32-S3 (800x480 RGB, PSRAM).
 *
 * Everything not set here falls back to the defaults in lv_conf_internal.h.
 * If you need to tweak features, copy more lines from
 *   .pio/libdeps/crowpanel5/lvgl/lv_conf_template.h
 * and override them below.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
 * COLOR / DISPLAY
 *====================*/
#define LV_COLOR_DEPTH      16          /* RGB565 to match the panel */

/*====================
 * MEMORY (use PSRAM via Arduino heap)
 *====================*/
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_CLIB   /* malloc/free; ESP32 routes large allocs to PSRAM */
#define LV_USE_STDLIB_STRING    LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_CLIB

/*====================
 * TICK / OS
 *====================*/
/* v9 tick: set in main.cpp via lv_tick_set_cb(millis). No macros needed here. */
#define LV_USE_OS                LV_OS_NONE      /* call lv_timer_handler() from loop() */

/*====================
 * RENDERING / PERF
 *====================*/
#define LV_DRAW_BUF_ALIGN          4
#define LV_DRAW_SW_COMPLEX         1
#define LV_USE_DRAW_SW             1

/*====================
 * LOGGING (optional)
 *====================*/
#define LV_USE_LOG          0
/* If you turn logging on:
 * #define LV_LOG_LEVEL     LV_LOG_LEVEL_WARN
 * #define LV_LOG_PRINTF    1
 */

/*====================
 * FONTS
 *====================*/
#define LV_FONT_MONTSERRAT_14    1
#define LV_FONT_MONTSERRAT_20    1
#define LV_FONT_MONTSERRAT_28    1       /* CMU title + nav buttons */
#define LV_FONT_UNSCII_16        1       /* crisp 8x16 bitmap, no anti-aliasing */
#define LV_FONT_DEFAULT          &lv_font_unscii_16

/*====================
 * WIDGETS (defaults enable all; keep that)
 *====================*/

/*====================
 * DEMOS (enable if you want to call lv_demo_widgets() etc.)
 *====================*/
#define LV_USE_DEMO_WIDGETS        1
#define LV_USE_DEMO_BENCHMARK      0
#define LV_USE_DEMO_STRESS         0
#define LV_USE_DEMO_MUSIC          0

#endif /* LV_CONF_H */
