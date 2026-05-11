/* ---------------------------------------------------------------------
 * CrowPanel 5" v3.0 ESP32-S3 HMI — Mitsubishi i-MiEV BMS viewer
 *
 * Listens on CAN (TWAI) for the BMU/CMU broadcasts and shows one CMU at
 * a time on the LVGL UI. Tap NEXT/PREV to navigate; if you leave it
 * alone for AUTO_FOLLOW_DELAY_MS, the display auto-jumps to whichever
 * CMU was most recently received.
 *
 * Hardware: external CAN transceiver wired to
 *   ESP32 GPIO10  -> transceiver CTX  (TX from MCU)
 *   ESP32 GPIO13  -> transceiver CRX  (RX into MCU)
 * --------------------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <driver/twai.h>
#include <lvgl.h>

#include "LGFX_CrowPanel5.hpp"

/* =====================================================================
 * Config
 * ===================================================================== */
static constexpr uint32_t SCREEN_W = 800;
static constexpr uint32_t SCREEN_H = 480;
static constexpr uint32_t DRAW_BUF_LINES = 48;
static constexpr uint32_t DRAW_BUF_BYTES = SCREEN_W * DRAW_BUF_LINES * 2;

static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_10;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_13;

static constexpr int NUM_CMUS       = 12;
static constexpr int CELLS_PER_CMU  = 8;
static constexpr int TEMPS_PER_CMU  = 6;
static constexpr uint32_t AUTO_FOLLOW_DELAY_MS = 5000;
static constexpr uint32_t STALE_DATA_MS        = 2000;

// Set to 1 to skip CAN init and feed synthetic frames so you can preview
// the layout on the bench without the i-MiEV pack attached.
#define BMS_TEST_MODE 0

/* =====================================================================
 * CrowPanel 5" v3.0 touch bring-up (PCA9557 -> GT911)
 * ===================================================================== */
static constexpr uint8_t PCA9557_ADDR = 0x19;

static bool pca9557_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(PCA9557_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool crowpanel_v3_touch_bringup() {
    Wire.begin(GPIO_NUM_19, GPIO_NUM_20, 300000);
    if (!pca9557_write(0x01, 0xFC)) {                   // IO0 LOW (RST), IO1 LOW (INT->0x5D)
        Serial.println("PCA9557 not responding at 0x19 -- touch will fail.");
        Wire.end();
        return false;
    }
    pca9557_write(0x03, 0x00);                          // all outputs
    delay(20);
    pca9557_write(0x01, 0xFD);                          // release RST; INT still LOW
    delay(100);
    pca9557_write(0x03, 0x02);                          // IO1 -> input (high-Z)
    Wire.end();
    return true;
}

static void i2c_scan() {
    Wire.begin(GPIO_NUM_19, GPIO_NUM_20, 100000);
    Serial.print("I2C scan:");
    int n = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", addr);
            n++;
        }
    }
    Serial.printf("  (%d device%s)\n", n, n == 1 ? "" : "s");
    Wire.end();
}

/* =====================================================================
 * Display + touch (LovyanGFX <-> LVGL 9 glue)
 * ===================================================================== */
static uint8_t *draw_buf1 = nullptr;
static uint8_t *draw_buf2 = nullptr;
static LGFX lcd;

static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    const uint32_t w = lv_area_get_width(area);
    const uint32_t h = lv_area_get_height(area);
    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.writePixels(reinterpret_cast<uint16_t *>(px_map), w * h);
    lcd.endWrite();
    lv_display_flush_ready(disp);
}

static void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
    uint16_t x, y;
    if (lcd.getTouch(&x, &y)) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* =====================================================================
 * BMS data model
 * The CAN RX task writes into g_cmu[] under g_cmu_mux. The UI timer
 * (LVGL task) takes a snapshot under the same mutex and renders.
 * ===================================================================== */
struct CmuState {
    uint16_t cell_raw[CELLS_PER_CMU];   // 16-bit BMU value (before 5mV*x + 2.1 conversion)
    bool     cell_valid[CELLS_PER_CMU];
    int8_t   temp_c[TEMPS_PER_CMU];     // already offset-corrected (raw - 50)
    bool     temp_valid[TEMPS_PER_CMU];
    uint8_t  balance_mask;
    uint32_t last_update_ms;
    uint32_t frames_seen;
};

static CmuState     g_cmu[NUM_CMUS];
static portMUX_TYPE g_cmu_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int g_latest_cmu = -1;
static volatile uint32_t g_total_frames = 0;
static volatile uint32_t g_rx_errors    = 0;

static inline float cell_volts(uint16_t raw) {
    return raw * 0.005f + 2.1f;
}

/* =====================================================================
 * CAN (TWAI) — init + receive task
 * ===================================================================== */
static void decode_frame(const twai_message_t &msg) {
    if (msg.flags & TWAI_MSG_FLAG_EXTD) return;
    if (msg.data_length_code != 8) return;

    const uint32_t id = msg.identifier;
    if (id < 0x611 || id > 0x6C4) return;
    const int cmu_idx = ((id - 0x600) >> 4) - 1;     // 0..11
    const int pair    = (id & 0x0F) - 1;             // 0..3
    if (cmu_idx < 0 || cmu_idx >= NUM_CMUS) return;
    if (pair < 0 || pair > 3) return;

    const uint8_t *d = msg.data;
    const int c1 = pair * 2;
    const int c2 = pair * 2 + 1;
    const uint16_t v1 = (uint16_t(d[4]) << 8) | d[5];
    const uint16_t v2 = (uint16_t(d[6]) << 8) | d[7];

    portENTER_CRITICAL(&g_cmu_mux);
    CmuState &c = g_cmu[cmu_idx];
    c.cell_raw[c1]   = v1;
    c.cell_raw[c2]   = v2;
    c.cell_valid[c1] = (v1 != 0);
    c.cell_valid[c2] = (v2 != 0);

    switch (pair) {
        case 0:                                       // frame 1: balance + T1, T2
            c.balance_mask = d[1];
            c.temp_c[0]     = int8_t(int(d[2]) - 50);
            c.temp_valid[0] = (d[2] != 0);
            c.temp_c[1]     = int8_t(int(d[3]) - 50);
            c.temp_valid[1] = (d[3] != 0);
            break;
        case 1:                                       // frame 2: T3, T4
            c.temp_c[2]     = int8_t(int(d[2]) - 50);
            c.temp_valid[2] = (d[2] != 0);
            c.temp_c[3]     = int8_t(int(d[3]) - 50);
            c.temp_valid[3] = (d[3] != 0);
            break;
        case 2:                                       // frame 3: T5, T6
            c.temp_c[4]     = int8_t(int(d[1]) - 50);
            c.temp_valid[4] = (d[1] != 0);
            c.temp_c[5]     = int8_t(int(d[2]) - 50);
            c.temp_valid[5] = (d[2] != 0);
            break;
        case 3:                                       // frame 4: no temps
            break;
    }
    c.last_update_ms = millis();
    c.frames_seen++;
    g_latest_cmu    = cmu_idx;
    g_total_frames++;
    portEXIT_CRITICAL(&g_cmu_mux);
}

static void can_rx_task(void *) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(200)) == ESP_OK) {
            decode_frame(msg);
        } else {
            // periodic status check
            twai_status_info_t st;
            if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
                Serial.println("TWAI BUS_OFF -- recovering");
                twai_initiate_recovery();
            }
        }
    }
}

static bool can_init() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g.rx_queue_len = 64;
    g.tx_queue_len = 8;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("twai_start failed");
        return false;
    }
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, nullptr, 5, nullptr, 0);
    return true;
}

/* =====================================================================
 * Test-mode synthetic frame generator
 * Feeds plausible BMS frames into decode_frame() so the UI lights up
 * with no CAN bus attached. Compile-out when BMS_TEST_MODE == 0.
 * ===================================================================== */
#if BMS_TEST_MODE
static void test_data_task(void *) {
    // i-MiEV layout: ten 8-cell modules, two 4-cell modules.
    // We treat CMUs 11 and 12 (indices 10, 11) as the 4-cell modules,
    // so frames 3 & 4 carry zeros for them (per spec).
    auto is_4cell = [](int cmu_idx) { return cmu_idx >= 10; };

    int cmu = 0;
    int pair = 0;
    while (true) {
        twai_message_t m{};
        m.identifier       = 0x600 + (cmu + 1) * 0x10 + (pair + 1);
        m.data_length_code = 8;
        m.data[0]          = uint8_t(cmu + 1);

        // Cell voltages: baseline ~3.95V with small jitter; sprinkle a few
        // out-of-range cells so every color is exercised.
        auto fake_cell_raw = [&](int cmu_, int cell_) -> uint16_t {
            int base = 370;                                    // ~3.95 V
            if      ((cmu_ + cell_ * 3) %  31 == 0) base = 220; // ~3.20 V (low / red)
            else if ((cmu_ + cell_ * 5) %  29 == 0) base = 425; // ~4.225 V (high / red)
            else if ((cmu_ + cell_ * 7) %  13 == 0) base = 395; // ~4.075 V (near top / amber)
            int jitter = (int)random(-12, 13);
            int v = base + jitter;
            if (v < 0)   v = 0;
            if (v > 600) v = 600;
            return uint16_t(v);
        };

        const bool pair_has_cells = !is_4cell(cmu) || pair < 2;
        const uint16_t v1 = pair_has_cells ? fake_cell_raw(cmu, pair * 2)     : 0;
        const uint16_t v2 = pair_has_cells ? fake_cell_raw(cmu, pair * 2 + 1) : 0;
        m.data[4] = uint8_t(v1 >> 8);
        m.data[5] = uint8_t(v1 & 0xFF);
        m.data[6] = uint8_t(v2 >> 8);
        m.data[7] = uint8_t(v2 & 0xFF);

        auto fake_temp = [](int /*cmu_*/, int sensor) -> uint8_t {
            int t = 18 + sensor + (int)random(0, 4);   // 18..27 C-ish
            return uint8_t(50 + t);
        };

        // Slow-toggling balance pattern: every ~4s, CMUs 2 and 5 balance a few cells
        const bool balance_phase = (millis() / 4000) & 1;
        const uint8_t balance_mask =
            (cmu == 1 && balance_phase) ? 0b00010101 :        // cells 1,3,5
            (cmu == 4 && balance_phase) ? 0b10000010 : 0;     // cells 2,8

        switch (pair) {
            case 0:
                m.data[1] = balance_mask;
                m.data[2] = fake_temp(cmu, 1);
                m.data[3] = fake_temp(cmu, 2);
                break;
            case 1:
                m.data[1] = 0;
                m.data[2] = pair_has_cells ? fake_temp(cmu, 3) : 0;
                m.data[3] = pair_has_cells ? fake_temp(cmu, 4) : 0;
                break;
            case 2:
                m.data[1] = pair_has_cells ? fake_temp(cmu, 5) : 0;
                m.data[2] = pair_has_cells ? fake_temp(cmu, 6) : 0;
                m.data[3] = 0;
                break;
            case 3:
                m.data[1] = 0;
                m.data[2] = 0;
                m.data[3] = 0;
                break;
        }

        decode_frame(m);

        pair = (pair + 1) & 0x03;
        if (pair == 0) cmu = (cmu + 1) % NUM_CMUS;
        delay(80);                  // ~320 ms per CMU, ~4 s full sweep
    }
}
#endif

/* =====================================================================
 * BMS UI (LVGL)
 * ===================================================================== */
static int      g_view_cmu        = 0;     // 0..11
static uint32_t g_last_user_input = 0;

static lv_obj_t *ui_cmu_title;
static lv_obj_t *ui_mode_badge;
static lv_obj_t *ui_summary;
static lv_obj_t *ui_cell_labels[CELLS_PER_CMU];
static lv_obj_t *ui_temp_labels[TEMPS_PER_CMU];
static lv_obj_t *ui_balance_label;
static lv_obj_t *ui_state_label;

static void on_prev(lv_event_t *) {
    g_view_cmu        = (g_view_cmu + NUM_CMUS - 1) % NUM_CMUS;
    g_last_user_input = millis();
}
static void on_next(lv_event_t *) {
    g_view_cmu        = (g_view_cmu + 1) % NUM_CMUS;
    g_last_user_input = millis();
}

static lv_obj_t *make_nav_button(lv_obj_t *parent, const char *text,
                                 lv_align_t align, int x_ofs,
                                 lv_event_cb_t cb) {
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 160, 80);
    lv_obj_align(btn, align, x_ofs, -20);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
    return btn;
}

static void build_bms_ui() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Header: CMU title + AUTO/MANUAL badge
    ui_cmu_title = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_cmu_title, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(ui_cmu_title, &lv_font_montserrat_28, 0);
    lv_obj_align(ui_cmu_title, LV_ALIGN_TOP_LEFT, 20, 14);
    lv_label_set_text(ui_cmu_title, "CMU --");

    ui_mode_badge = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_mode_badge, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(ui_mode_badge, lv_color_hex(0x2E7D32), 0);
    lv_obj_set_style_bg_opa(ui_mode_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ui_mode_badge, 6, 0);
    lv_obj_set_style_radius(ui_mode_badge, 4, 0);
    lv_obj_align(ui_mode_badge, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_label_set_text(ui_mode_badge, "AUTO");

    // Pack summary line (unscii_16 by default)
    ui_summary = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_summary, lv_color_hex(0x000000), 0);
    lv_obj_align(ui_summary, LV_ALIGN_TOP_LEFT, 20, 60);
    lv_label_set_text(ui_summary, "Waiting for CAN data...");

    // Cell labels: 8 rows on the left. 28 px pitch fits unscii_16 with 12 px gap.
    constexpr int ROW_Y0 = 95;
    constexpr int ROW_PITCH = 28;
    for (int i = 0; i < CELLS_PER_CMU; i++) {
        ui_cell_labels[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(ui_cell_labels[i], lv_color_hex(0x000000), 0);
        lv_obj_align(ui_cell_labels[i], LV_ALIGN_TOP_LEFT, 30, ROW_Y0 + i * ROW_PITCH);
        lv_label_set_text_fmt(ui_cell_labels[i], "Cell %d: --", i + 1);
    }
    // Temp labels: 6 rows on the right
    for (int i = 0; i < TEMPS_PER_CMU; i++) {
        ui_temp_labels[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(ui_temp_labels[i], lv_color_hex(0x000000), 0);
        lv_obj_align(ui_temp_labels[i], LV_ALIGN_TOP_LEFT, 440, ROW_Y0 + i * ROW_PITCH);
        lv_label_set_text_fmt(ui_temp_labels[i], "T%d: --", i + 1);
    }

    ui_balance_label = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_balance_label, lv_color_hex(0x000000), 0);
    lv_obj_align(ui_balance_label, LV_ALIGN_TOP_LEFT, 440, ROW_Y0 + 6 * ROW_PITCH + 8);
    lv_label_set_text(ui_balance_label, "Balance: --");

    // State/diagnostics block: two lines, sitting just above the nav buttons
    // (buttons start at y=380, so two 16px lines from y=336 leave a small gap).
    ui_state_label = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_state_label, lv_color_hex(0x666666), 0);
    lv_obj_align(ui_state_label, LV_ALIGN_TOP_LEFT, 20, 336);
    lv_label_set_text(ui_state_label, "no frames");

    make_nav_button(scr, "<",   LV_ALIGN_BOTTOM_LEFT,  30, on_prev);
    make_nav_button(scr, ">",   LV_ALIGN_BOTTOM_RIGHT, -30, on_next);
}

static void refresh_ui_timer(lv_timer_t *) {
    const uint32_t now = millis();
    const bool auto_mode = (now - g_last_user_input) > AUTO_FOLLOW_DELAY_MS;

    // Auto-follow: jump to whichever CMU most recently produced a frame
    if (auto_mode) {
        int latest = g_latest_cmu;
        if (latest >= 0 && latest < NUM_CMUS) {
            g_view_cmu = latest;
        }
    }
    lv_label_set_text(ui_mode_badge, auto_mode ? "AUTO" : "MANUAL");
    lv_obj_set_style_bg_color(ui_mode_badge,
                              lv_color_hex(auto_mode ? 0x2E7D32 : 0x1565C0), 0);

    // Snapshot the viewed CMU + compute pack-wide stats in one critical section
    CmuState snap;
    float min_v = 99.f, max_v = 0.f, sum_v = 0.f;
    int   valid_cells = 0;
    int8_t min_t = 127, max_t = -127;
    bool any_temp = false;

    portENTER_CRITICAL(&g_cmu_mux);
    snap = g_cmu[g_view_cmu];
    for (int i = 0; i < NUM_CMUS; i++) {
        for (int j = 0; j < CELLS_PER_CMU; j++) {
            if (g_cmu[i].cell_valid[j]) {
                float v = cell_volts(g_cmu[i].cell_raw[j]);
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
                sum_v += v;
                valid_cells++;
            }
        }
        for (int j = 0; j < TEMPS_PER_CMU; j++) {
            if (g_cmu[i].temp_valid[j]) {
                int8_t t = g_cmu[i].temp_c[j];
                if (t < min_t) min_t = t;
                if (t > max_t) max_t = t;
                any_temp = true;
            }
        }
    }
    const uint32_t total_frames = g_total_frames;
    portEXIT_CRITICAL(&g_cmu_mux);

#if BMS_TEST_MODE
    lv_label_set_text_fmt(ui_cmu_title, "CMU %d  [TEST]", g_view_cmu + 1);
#else
    lv_label_set_text_fmt(ui_cmu_title, "CMU %d", g_view_cmu + 1);
#endif

    if (valid_cells > 0) {
        float avg_v = sum_v / valid_cells;
        if (any_temp) {
            lv_label_set_text_fmt(ui_summary,
                "Pack:  min %.3f V   max %.3f V   avg %.3f V   delta %.0f mV   T %d..%d C",
                min_v, max_v, avg_v, (max_v - min_v) * 1000.f, min_t, max_t);
        } else {
            lv_label_set_text_fmt(ui_summary,
                "Pack:  min %.3f V   max %.3f V   avg %.3f V   delta %.0f mV",
                min_v, max_v, avg_v, (max_v - min_v) * 1000.f);
        }
    } else {
        lv_label_set_text(ui_summary, "Waiting for CAN data...");
    }

    // Per-cell rows for the selected CMU
    for (int i = 0; i < CELLS_PER_CMU; i++) {
        if (snap.cell_valid[i]) {
            float v = cell_volts(snap.cell_raw[i]);
            bool balancing = (snap.balance_mask >> i) & 0x01;
            uint32_t color = 0x000000;
            if      (v < 3.30f) color = 0x800000;        // dark red: low
            else if (v > 4.15f) color = 0x800000;        // dark red: high
            else if (v > 4.05f) color = 0x803000;        // dark amber: near top
            else if (balancing) color = 0x000080;        // navy: balancing
            else                color = 0x004000;        // dark green: healthy
            lv_obj_set_style_text_color(ui_cell_labels[i], lv_color_hex(color), 0);
            lv_label_set_text_fmt(ui_cell_labels[i],
                "Cell %d:  %.3f V%s", i + 1, v, balancing ? "   BAL" : "");
        } else {
            lv_obj_set_style_text_color(ui_cell_labels[i], lv_color_hex(0xBBBBBB), 0);
            lv_label_set_text_fmt(ui_cell_labels[i], "Cell %d:  --", i + 1);
        }
    }
    for (int i = 0; i < TEMPS_PER_CMU; i++) {
        if (snap.temp_valid[i]) {
            lv_obj_set_style_text_color(ui_temp_labels[i], lv_color_hex(0x000000), 0);
            lv_label_set_text_fmt(ui_temp_labels[i], "T%d:  %d C", i + 1, snap.temp_c[i]);
        } else {
            lv_obj_set_style_text_color(ui_temp_labels[i], lv_color_hex(0xBBBBBB), 0);
            lv_label_set_text_fmt(ui_temp_labels[i], "T%d:  --", i + 1);
        }
    }

    char balstr[24];
    int n = 0;
    for (int i = 0; i < CELLS_PER_CMU; i++) {
        if ((snap.balance_mask >> i) & 1) {
            n += snprintf(balstr + n, sizeof(balstr) - n, "%s%d", n ? "," : "", i + 1);
            if (n >= (int)sizeof(balstr) - 4) break;
        }
    }
    if (n == 0) lv_label_set_text(ui_balance_label, "Balance: none");
    else        lv_label_set_text_fmt(ui_balance_label, "Balance: %s", balstr);

    if (snap.last_update_ms == 0) {
        lv_label_set_text_fmt(ui_state_label,
            "CMU %d: no frames yet\n"
            "pack frames: %lu",
            g_view_cmu + 1, (unsigned long)total_frames);
    } else {
        uint32_t age = now - snap.last_update_ms;
        const char *fresh = (age < STALE_DATA_MS) ? "fresh" : "STALE";
        lv_label_set_text_fmt(ui_state_label,
            "CMU %d: last frame %lu ms ago (%s)\n"
            "frames this CMU: %lu     pack frames: %lu",
            g_view_cmu + 1, (unsigned long)age, fresh,
            (unsigned long)snap.frames_seen, (unsigned long)total_frames);
    }
}

/* =====================================================================
 * setup / loop
 * ===================================================================== */
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\nCrowPanel 5 v3.0 / LVGL 9 / i-MiEV BMS viewer booting...");

    if (!crowpanel_v3_touch_bringup()) {
        Serial.println("Touch bring-up failed -- running I2C scan for diagnostics:");
        i2c_scan();
        Serial.println("Expected: 0x19 (PCA9557) and 0x5D (GT911) after reset.");
    } else {
        i2c_scan();
    }

#if BMS_TEST_MODE
    Serial.println("BMS_TEST_MODE: feeding synthetic frames (CAN driver NOT started)");
    xTaskCreatePinnedToCore(test_data_task, "bms_test", 4096, nullptr, 4, nullptr, 0);
#else
    if (can_init()) {
        Serial.printf("CAN: 500 kbps  TX=GPIO%d  RX=GPIO%d\n", CAN_TX_PIN, CAN_RX_PIN);
    } else {
        Serial.println("WARNING: CAN init failed -- UI will show no data.");
    }
#endif

    lcd.init();
    lcd.setRotation(0);
    lcd.setBrightness(200);

    lv_init();
    lv_tick_set_cb([]() -> uint32_t { return millis(); });

    draw_buf1 = static_cast<uint8_t *>(heap_caps_malloc(DRAW_BUF_BYTES, MALLOC_CAP_SPIRAM));
    draw_buf2 = static_cast<uint8_t *>(heap_caps_malloc(DRAW_BUF_BYTES, MALLOC_CAP_SPIRAM));
    if (!draw_buf1 || !draw_buf2) {
        Serial.println("FATAL: could not allocate LVGL draw buffers in PSRAM");
        while (true) delay(1000);
    }

    lv_display_t *disp = lv_display_create(SCREEN_W, SCREEN_H);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf1, draw_buf2, DRAW_BUF_BYTES,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read);

    build_bms_ui();
    lv_timer_create(refresh_ui_timer, 250, nullptr);

    Serial.println("Ready.");
}

void loop() {
    lv_timer_handler();
    delay(5);
}
