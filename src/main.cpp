/* ---------------------------------------------------------------------
 * CrowPanel 5" v3.0 ESP32-S3 HMI — Mitsubishi i-MiEV BMS viewer
 *
 * Two modes (picked at boot from a 5 s selection screen):
 *
 *   CMU bus mode   - plugged into the BMU/CMU internal bus (IDs
 *                    0x611..0x6C4). Shows one CMU at a time with all
 *                    cells, temps and balancing.
 *   OBD2 mode      - plugged into the car's OBD2 / CAR-CAN bus
 *                    (IDs 0x373/0x374/0x6E1..0x6E4). Pack-level
 *                    dashboard: SoC, V, A, kW, min/max cell, temp.
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
static constexpr uint32_t MODE_SELECT_TIMEOUT_S = 5;

// Set to 1 to skip CAN init and feed synthetic frames so you can preview
// the layout on the bench without a pack attached.
#define BMS_TEST_MODE 0

/* =====================================================================
 * Mode state
 * ===================================================================== */
enum BmsMode {
    MODE_NONE = 0,
    MODE_CMU,
    MODE_OBD2,
};
static volatile BmsMode g_mode = MODE_NONE;

// Forward declarations (mode transitions hop between modules)
static void enter_mode(BmsMode m);
static void build_cmu_screen();
static void build_obd2_screen();
static void refresh_cmu_timer(lv_timer_t *);
static void refresh_obd2_timer(lv_timer_t *);

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
    if (!pca9557_write(0x01, 0xFC)) {
        Serial.println("PCA9557 not responding at 0x19 -- touch will fail.");
        Wire.end();
        return false;
    }
    pca9557_write(0x03, 0x00);
    delay(20);
    pca9557_write(0x01, 0xFD);
    delay(100);
    pca9557_write(0x03, 0x02);
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
 * Data model — CMU bus + OBD2 pack
 *
 * Both modes write into g_cmu[] (per-CMU cells/temps). OBD2 mode also
 * fills g_pack with the aggregated values from 0x373/0x374. All access
 * is serialized by g_data_mux so the CAN RX task (core 0) and LVGL
 * timer (core 1) can't tear each other.
 * ===================================================================== */
struct CmuState {
    uint16_t cell_raw[CELLS_PER_CMU];
    bool     cell_valid[CELLS_PER_CMU];
    int8_t   temp_c[TEMPS_PER_CMU];
    bool     temp_valid[TEMPS_PER_CMU];
    uint8_t  balance_mask;
    uint32_t last_update_ms;
    uint32_t frames_seen;
};

struct PackState {
    float    pack_v;             // V       (0x373 b4-b5 * 0.1)
    float    pack_a;             // A       (0x373 b2-b3, signed)
    float    power_kw;           // V * A / 1000
    float    cell_min_v;         // V       (0x373 b1 + 210)/100
    float    cell_max_v;         // V       (0x373 b0 + 210)/100
    float    soc1, soc2;         // %       (0x374 b0, b1)
    int8_t   max_temp_c;         // °C      (0x374 b4)
    bool     have_373, have_374;
    uint32_t last_update_ms;
    uint32_t frames_seen;
};

static CmuState     g_cmu[NUM_CMUS];
static PackState    g_pack;
static portMUX_TYPE g_data_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int g_latest_cmu = -1;
static volatile uint32_t g_total_frames = 0;

static inline float cell_volts(uint16_t raw) {
    return raw * 0.005f + 2.1f;
}

/* =====================================================================
 * CAN (TWAI) — frame decode dispatcher
 *
 * Always decodes everything we know how to. Whether the user is on
 * the CMU bus or the OBD2 bus determines which subset of frames they
 * actually see — the rest of the decoders simply never fire.
 * ===================================================================== */
static void decode_bmu_frame(const twai_message_t &msg) {
    // CMU bus: per-cell data (IDs 0x611..0x6C4, paragraph 2 of the spec)
    const uint32_t id = msg.identifier;
    const int cmu_idx = ((id - 0x600) >> 4) - 1;
    const int pair    = (id & 0x0F) - 1;
    if (cmu_idx < 0 || cmu_idx >= NUM_CMUS) return;
    if (pair    < 0 || pair    > 3)         return;

    const uint8_t *d = msg.data;
    const int c1 = pair * 2;
    const int c2 = pair * 2 + 1;
    const uint16_t v1 = (uint16_t(d[4]) << 8) | d[5];
    const uint16_t v2 = (uint16_t(d[6]) << 8) | d[7];

    portENTER_CRITICAL(&g_data_mux);
    CmuState &c = g_cmu[cmu_idx];
    c.cell_raw[c1]   = v1;
    c.cell_raw[c2]   = v2;
    c.cell_valid[c1] = (v1 != 0);
    c.cell_valid[c2] = (v2 != 0);
    switch (pair) {
        case 0:
            c.balance_mask  = d[1];
            c.temp_c[0]     = int8_t(int(d[2]) - 50);  c.temp_valid[0] = (d[2] != 0);
            c.temp_c[1]     = int8_t(int(d[3]) - 50);  c.temp_valid[1] = (d[3] != 0);
            break;
        case 1:
            c.temp_c[2]     = int8_t(int(d[2]) - 50);  c.temp_valid[2] = (d[2] != 0);
            c.temp_c[3]     = int8_t(int(d[3]) - 50);  c.temp_valid[3] = (d[3] != 0);
            break;
        case 2:
            c.temp_c[4]     = int8_t(int(d[1]) - 50);  c.temp_valid[4] = (d[1] != 0);
            c.temp_c[5]     = int8_t(int(d[2]) - 50);  c.temp_valid[5] = (d[2] != 0);
            break;
        case 3:
            break;
    }
    c.last_update_ms = millis();
    c.frames_seen++;
    g_latest_cmu = cmu_idx;
    g_total_frames++;
    portEXIT_CRITICAL(&g_data_mux);
}

// OBD2 0x6E1..0x6E4 cell-voltage stream. data[0] is a 1-based CMU index.
// Per the c-zero_dashboard reference, the second half of the pack uses
// a -4 cell-slot offset and indices 5/11 (the two 4-cell modules) skip
// frames 3 & 4. Because our store is a 2D g_cmu[12][8], we don't need
// that offset — we just write into the natural module slot.
static void decode_obd2_cells_frame(const twai_message_t &msg) {
    const uint32_t id = msg.identifier;
    const uint8_t *d = msg.data;
    const int idx = int(d[0]) - 1;
    if (idx < 0 || idx >= NUM_CMUS) return;

    const uint16_t va = (uint16_t(d[4]) << 8) | d[5];
    const uint16_t vb = (uint16_t(d[6]) << 8) | d[7];
    const bool is_4cell = (idx == 5 || idx == 11);

    portENTER_CRITICAL(&g_data_mux);
    CmuState &c = g_cmu[idx];
    if (id == 0x6E1) {
        c.cell_raw[0] = va;  c.cell_valid[0] = (va != 0);
        c.cell_raw[1] = vb;  c.cell_valid[1] = (vb != 0);
        c.temp_c[0]     = int8_t(int(d[2]) - 50);  c.temp_valid[0] = (d[2] != 0);
        c.temp_c[1]     = int8_t(int(d[3]) - 50);  c.temp_valid[1] = (d[3] != 0);
    } else if (id == 0x6E2) {
        c.cell_raw[2] = va;  c.cell_valid[2] = (va != 0);
        c.cell_raw[3] = vb;  c.cell_valid[3] = (vb != 0);
        c.temp_c[2]     = int8_t(int(d[1]) - 50);  c.temp_valid[2] = (d[1] != 0);
        c.temp_c[3]     = int8_t(int(d[2]) - 50);  c.temp_valid[3] = (d[2] != 0);
    } else if (id == 0x6E3 && !is_4cell) {
        c.cell_raw[4] = va;  c.cell_valid[4] = (va != 0);
        c.cell_raw[5] = vb;  c.cell_valid[5] = (vb != 0);
        c.temp_c[4]     = int8_t(int(d[1]) - 50);  c.temp_valid[4] = (d[1] != 0);
        c.temp_c[5]     = int8_t(int(d[2]) - 50);  c.temp_valid[5] = (d[2] != 0);
    } else if (id == 0x6E4 && !is_4cell) {
        c.cell_raw[6] = va;  c.cell_valid[6] = (va != 0);
        c.cell_raw[7] = vb;  c.cell_valid[7] = (vb != 0);
    }
    c.last_update_ms = millis();
    c.frames_seen++;
    g_latest_cmu = idx;
    g_total_frames++;
    portEXIT_CRITICAL(&g_data_mux);
}

// 0x373: max/min cell V, pack current, pack voltage.
static void decode_obd2_373(const twai_message_t &msg) {
    const uint8_t *d = msg.data;
    const float cell_max = (float(d[0]) + 210.0f) / 100.0f;
    const float cell_min = (float(d[1]) + 210.0f) / 100.0f;
    const float amps     = ((float((uint16_t(d[2]) << 8) | d[3]) - 32768.0f) * -0.01f);
    const float volts    = float((uint16_t(d[4]) << 8) | d[5]) * 0.1f;
    portENTER_CRITICAL(&g_data_mux);
    g_pack.cell_max_v   = cell_max;
    g_pack.cell_min_v   = cell_min;
    g_pack.pack_a       = amps;
    g_pack.pack_v       = volts;
    g_pack.power_kw     = (amps * volts) / 1000.0f;
    g_pack.have_373     = true;
    g_pack.last_update_ms = millis();
    g_pack.frames_seen++;
    g_total_frames++;
    portEXIT_CRITICAL(&g_data_mux);
}

// 0x374: SoC, max temperature.
static void decode_obd2_374(const twai_message_t &msg) {
    const uint8_t *d = msg.data;
    const float s1 = (float(d[0]) - 10.0f) / 2.0f;
    const float s2 = (float(d[1]) - 10.0f) / 2.0f;
    const int8_t tmax = int8_t(int(d[4]) - 50);
    portENTER_CRITICAL(&g_data_mux);
    g_pack.soc1       = s1;
    g_pack.soc2       = s2;
    g_pack.max_temp_c = tmax;
    g_pack.have_374   = true;
    g_pack.last_update_ms = millis();
    g_pack.frames_seen++;
    g_total_frames++;
    portEXIT_CRITICAL(&g_data_mux);
}

static void decode_frame(const twai_message_t &msg) {
    if (msg.flags & TWAI_MSG_FLAG_EXTD) return;
    if (msg.data_length_code != 8)      return;
    const uint32_t id = msg.identifier;

    if (id >= 0x611 && id <= 0x6C4) {
        decode_bmu_frame(msg);
    } else if (id >= 0x6E1 && id <= 0x6E4) {
        decode_obd2_cells_frame(msg);
    } else if (id == 0x373) {
        decode_obd2_373(msg);
    } else if (id == 0x374) {
        decode_obd2_374(msg);
    }
}

/* =====================================================================
 * CAN (TWAI) init + receive task
 * ===================================================================== */
static void can_rx_task(void *) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(200)) == ESP_OK) {
            decode_frame(msg);
        } else {
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
    if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("twai_driver_install failed"); return false; }
    if (twai_start() != ESP_OK)                    { Serial.println("twai_start failed");          return false; }
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, nullptr, 5, nullptr, 0);
    return true;
}

/* =====================================================================
 * Test-mode synthetic frames (mode-aware)
 * ===================================================================== */
#if BMS_TEST_MODE
static void test_cmu_step(int &cmu, int &pair) {
    auto is_4cell = [](int cmu_idx) { return cmu_idx >= 10; };
    twai_message_t m{};
    m.identifier       = 0x600 + (cmu + 1) * 0x10 + (pair + 1);
    m.data_length_code = 8;
    m.data[0]          = uint8_t(cmu + 1);

    auto fake_cell_raw = [&](int cmu_, int cell_) -> uint16_t {
        int base = 370;
        if      ((cmu_ + cell_ * 3) %  31 == 0) base = 220;
        else if ((cmu_ + cell_ * 5) %  29 == 0) base = 425;
        else if ((cmu_ + cell_ * 7) %  13 == 0) base = 395;
        int v = base + (int)random(-12, 13);
        if (v < 0) v = 0;  if (v > 600) v = 600;
        return uint16_t(v);
    };
    const bool pair_has_cells = !is_4cell(cmu) || pair < 2;
    const uint16_t v1 = pair_has_cells ? fake_cell_raw(cmu, pair * 2)     : 0;
    const uint16_t v2 = pair_has_cells ? fake_cell_raw(cmu, pair * 2 + 1) : 0;
    m.data[4] = uint8_t(v1 >> 8);  m.data[5] = uint8_t(v1 & 0xFF);
    m.data[6] = uint8_t(v2 >> 8);  m.data[7] = uint8_t(v2 & 0xFF);

    auto fake_temp = [](int sensor) -> uint8_t { return uint8_t(50 + 18 + sensor + (int)random(0, 4)); };
    const bool balance_phase = (millis() / 4000) & 1;
    const uint8_t bal =
        (cmu == 1 && balance_phase) ? 0b00010101 :
        (cmu == 4 && balance_phase) ? 0b10000010 : 0;
    switch (pair) {
        case 0:  m.data[1] = bal; m.data[2] = fake_temp(1); m.data[3] = fake_temp(2); break;
        case 1:  m.data[1] = 0;   m.data[2] = pair_has_cells ? fake_temp(3) : 0; m.data[3] = pair_has_cells ? fake_temp(4) : 0; break;
        case 2:  m.data[1] = pair_has_cells ? fake_temp(5) : 0; m.data[2] = pair_has_cells ? fake_temp(6) : 0; m.data[3] = 0; break;
        case 3:  m.data[1] = 0; m.data[2] = 0; m.data[3] = 0; break;
    }
    decode_frame(m);
    pair = (pair + 1) & 0x03;
    if (pair == 0) cmu = (cmu + 1) % NUM_CMUS;
}

static void test_obd2_step(int &cell_idx, uint32_t &phase_ms) {
    const uint32_t now = millis();
    // 0x373 every 100 ms
    if (now - phase_ms >= 100) {
        phase_ms = now;
        twai_message_t m{};
        m.identifier       = 0x373;
        m.data_length_code = 8;
        // Simulate pack: 345 V, oscillating current
        float amps  = -12.0f + 6.0f * sinf((float)now / 800.0f);
        float volts = 345.0f + 3.0f * sinf((float)now / 1500.0f);
        // amps raw: amps = (raw - 32768) * -0.01  =>  raw = 32768 - amps*100
        uint16_t a_raw = uint16_t(32768.0f - amps * 100.0f);
        uint16_t v_raw = uint16_t(volts * 10.0f);
        m.data[0] = uint8_t(int((4.10f * 100.0f) - 210.0f));    // cell max ~4.10
        m.data[1] = uint8_t(int((3.92f * 100.0f) - 210.0f));    // cell min ~3.92
        m.data[2] = uint8_t(a_raw >> 8);  m.data[3] = uint8_t(a_raw & 0xFF);
        m.data[4] = uint8_t(v_raw >> 8);  m.data[5] = uint8_t(v_raw & 0xFF);
        decode_frame(m);

        // 0x374 every 100 ms too
        m = {};
        m.identifier = 0x374;
        m.data_length_code = 8;
        float soc = 87.5f + 0.5f * sinf((float)now / 5000.0f);
        m.data[0] = uint8_t(soc * 2.0f + 10.0f);
        m.data[1] = uint8_t(soc * 2.0f + 10.0f);
        m.data[4] = uint8_t(50 + 26);   // max temp 26 C
        decode_frame(m);
    }

    // Cell stream: one CMU per call. Cycle 0..11.
    twai_message_t m{};
    m.data_length_code = 8;
    m.data[0] = uint8_t(cell_idx + 1);
    const bool is_4cell = (cell_idx == 5 || cell_idx == 11);
    auto cell_raw = [&](int c) -> uint16_t {
        int base = 370 + ((cell_idx * 7 + c * 3) % 17) - 8;
        return uint16_t(base + (int)random(-3, 4));
    };
    auto temp_raw = [](int sensor) -> uint8_t { return uint8_t(50 + 22 + sensor + (int)random(0, 3)); };
    auto pack = [&](uint16_t a, uint16_t b) {
        m.data[4] = uint8_t(a >> 8);  m.data[5] = uint8_t(a & 0xFF);
        m.data[6] = uint8_t(b >> 8);  m.data[7] = uint8_t(b & 0xFF);
    };

    m.identifier = 0x6E1;
    pack(cell_raw(0), cell_raw(1));
    m.data[2] = temp_raw(1);  m.data[3] = temp_raw(2);
    decode_frame(m);

    m = {};  m.data_length_code = 8;  m.data[0] = uint8_t(cell_idx + 1);
    m.identifier = 0x6E2;
    pack(cell_raw(2), cell_raw(3));
    m.data[1] = temp_raw(3);  m.data[2] = temp_raw(4);
    decode_frame(m);

    if (!is_4cell) {
        m = {};  m.data_length_code = 8;  m.data[0] = uint8_t(cell_idx + 1);
        m.identifier = 0x6E3;
        pack(cell_raw(4), cell_raw(5));
        m.data[1] = temp_raw(5);  m.data[2] = temp_raw(6);
        decode_frame(m);

        m = {};  m.data_length_code = 8;  m.data[0] = uint8_t(cell_idx + 1);
        m.identifier = 0x6E4;
        pack(cell_raw(6), cell_raw(7));
        decode_frame(m);
    }
    cell_idx = (cell_idx + 1) % NUM_CMUS;
}

static void test_data_task(void *) {
    while (g_mode == MODE_NONE) delay(50);   // wait for the user to pick a mode

    if (g_mode == MODE_CMU) {
        int cmu = 0, pair = 0;
        while (true) { test_cmu_step(cmu, pair); delay(80); }
    } else {
        int cell_idx = 0;
        uint32_t phase = 0;
        while (true) { test_obd2_step(cell_idx, phase); delay(80); }
    }
}
#endif

/* =====================================================================
 * Mode selection screen
 * ===================================================================== */
static lv_obj_t *ui_select_countdown;
static lv_timer_t *g_select_tick = nullptr;
static int g_select_seconds_left;

// Deferred mode transitions so we don't tear down a screen while one of
// its objects is still mid-event.
static void async_enter_cmu(void *)  { enter_mode(MODE_CMU); }
static void async_enter_obd2(void *) { enter_mode(MODE_OBD2); }

static void select_tick_cb(lv_timer_t *) {
    g_select_seconds_left--;
    if (g_select_seconds_left <= 0) {
        lv_async_call(async_enter_cmu, nullptr);
        return;
    }
    lv_label_set_text_fmt(ui_select_countdown,
        "CMU bus selected in %d s...", g_select_seconds_left);
}

static void on_pick_cmu(lv_event_t *)  { lv_async_call(async_enter_cmu,  nullptr); }
static void on_pick_obd2(lv_event_t *) { lv_async_call(async_enter_obd2, nullptr); }

static void build_mode_select_screen() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "i-MiEV BMS viewer");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_42, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *sub = lv_label_create(scr);
    lv_label_set_text(sub, "Select data source");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x333333), 0);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_28, 0);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 110);

    auto make_pick = [&](const char *label, const char *sub_text,
                         lv_event_cb_t cb, int x_ofs, uint32_t accent) {
        lv_obj_t *btn = lv_button_create(scr);
        lv_obj_set_size(btn, 320, 160);
        lv_obj_align(btn, LV_ALIGN_CENTER, x_ofs, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(accent), 0);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_t *l1 = lv_label_create(btn);
        lv_label_set_text(l1, label);
        lv_obj_set_style_text_color(l1, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(l1, &lv_font_montserrat_42, 0);
        lv_obj_align(l1, LV_ALIGN_CENTER, 0, -20);
        lv_obj_t *l2 = lv_label_create(btn);
        lv_label_set_text(l2, sub_text);
        lv_obj_set_style_text_color(l2, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(l2, LV_ALIGN_CENTER, 0, 30);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        return btn;
    };
    make_pick("CMU Bus", "CMU (default)", on_pick_cmu,  -180, 0x004000);
    make_pick("OBD2",    "pack dashboard",            on_pick_obd2,  180, 0x000080);

    ui_select_countdown = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_select_countdown, lv_color_hex(0x444444), 0);
    lv_obj_set_style_text_font(ui_select_countdown, &lv_font_montserrat_28, 0);
    lv_obj_align(ui_select_countdown, LV_ALIGN_BOTTOM_MID, 0, -40);

    g_select_seconds_left = MODE_SELECT_TIMEOUT_S;
    lv_label_set_text_fmt(ui_select_countdown,
        "CMU bus selected in %d s...", g_select_seconds_left);
    g_select_tick = lv_timer_create(select_tick_cb, 1000, nullptr);
}

/* =====================================================================
 * Mode transition
 * ===================================================================== */
static lv_timer_t *g_refresh_timer = nullptr;

static void enter_mode(BmsMode m) {
    if (g_mode != MODE_NONE) return;       // ignore stray events after the first commit
    g_mode = m;

    if (g_select_tick) { lv_timer_delete(g_select_tick); g_select_tick = nullptr; }
    lv_obj_clean(lv_screen_active());

    if (m == MODE_CMU) {
        Serial.println("Mode: CMU bus  (BMU IDs 0x611-0x6C4)");
        build_cmu_screen();
        g_refresh_timer = lv_timer_create(refresh_cmu_timer, 250, nullptr);
    } else {
        Serial.println("Mode: OBD2  (IDs 0x373, 0x374, 0x6E1-0x6E4)");
        build_obd2_screen();
        g_refresh_timer = lv_timer_create(refresh_obd2_timer, 250, nullptr);
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
}

/* =====================================================================
 * UI: CMU mode
 * ===================================================================== */
static int      g_view_cmu        = 0;
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
                                 lv_align_t align, int x_ofs, lv_event_cb_t cb) {
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

static void build_cmu_screen() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

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

    ui_summary = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_summary, lv_color_hex(0x000000), 0);
    lv_obj_align(ui_summary, LV_ALIGN_TOP_LEFT, 20, 60);
    lv_label_set_text(ui_summary, "Waiting for CAN data...");

    constexpr int ROW_Y0 = 95;
    constexpr int ROW_PITCH = 28;
    for (int i = 0; i < CELLS_PER_CMU; i++) {
        ui_cell_labels[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(ui_cell_labels[i], lv_color_hex(0x000000), 0);
        lv_obj_align(ui_cell_labels[i], LV_ALIGN_TOP_LEFT, 30, ROW_Y0 + i * ROW_PITCH);
        lv_label_set_text_fmt(ui_cell_labels[i], "Cell %d: --", i + 1);
    }
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

    ui_state_label = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_state_label, lv_color_hex(0x666666), 0);
    lv_obj_align(ui_state_label, LV_ALIGN_TOP_LEFT, 20, 336);
    lv_label_set_text(ui_state_label, "no frames");

    make_nav_button(scr, "<", LV_ALIGN_BOTTOM_LEFT,  30, on_prev);
    make_nav_button(scr, ">", LV_ALIGN_BOTTOM_RIGHT, -30, on_next);
}

static void refresh_cmu_timer(lv_timer_t *) {
    const uint32_t now = millis();
    const bool auto_mode = (now - g_last_user_input) > AUTO_FOLLOW_DELAY_MS;
    if (auto_mode) {
        int latest = g_latest_cmu;
        if (latest >= 0 && latest < NUM_CMUS) g_view_cmu = latest;
    }
    lv_label_set_text(ui_mode_badge, auto_mode ? "AUTO" : "MANUAL");
    lv_obj_set_style_bg_color(ui_mode_badge,
                              lv_color_hex(auto_mode ? 0x2E7D32 : 0x1565C0), 0);

    CmuState snap;
    float min_v = 99.f, max_v = 0.f, sum_v = 0.f;
    int   valid_cells = 0;
    int8_t min_t = 127, max_t = -127;
    bool any_temp = false;

    portENTER_CRITICAL(&g_data_mux);
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
    portEXIT_CRITICAL(&g_data_mux);

    lv_label_set_text_fmt(ui_cmu_title, "CMU %d", g_view_cmu + 1);

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

    for (int i = 0; i < CELLS_PER_CMU; i++) {
        if (snap.cell_valid[i]) {
            float v = cell_volts(snap.cell_raw[i]);
            bool balancing = (snap.balance_mask >> i) & 0x01;
            uint32_t color = 0x000000;
            if      (v < 3.30f) color = 0x800000;
            else if (v > 4.15f) color = 0x800000;
            else if (v > 4.05f) color = 0x803000;
            else if (balancing) color = 0x000080;
            else                color = 0x004000;
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
 * UI: OBD2 mode
 * ===================================================================== */
static lv_obj_t *ui_obd2_title;
static lv_obj_t *ui_obd2_fresh;
static lv_obj_t *ui_obd2_soc_num;
static lv_obj_t *ui_obd2_soc_sub;
static lv_obj_t *ui_obd2_kw_num;
static lv_obj_t *ui_obd2_kw_sub;
static lv_obj_t *ui_obd2_pack_v;
static lv_obj_t *ui_obd2_pack_a;
static lv_obj_t *ui_obd2_temp;
static lv_obj_t *ui_obd2_cell_max;
static lv_obj_t *ui_obd2_cell_min;
static lv_obj_t *ui_obd2_delta;
static lv_obj_t *ui_obd2_state;

static void build_obd2_screen() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    ui_obd2_title = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_title, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(ui_obd2_title, &lv_font_montserrat_28, 0);
    lv_obj_align(ui_obd2_title, LV_ALIGN_TOP_LEFT, 20, 14);
#if BMS_TEST_MODE
    lv_label_set_text(ui_obd2_title, "OBD2 Pack  [TEST]");
#else
    lv_label_set_text(ui_obd2_title, "OBD2 Pack");
#endif

    ui_obd2_fresh = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_fresh, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(ui_obd2_fresh, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(ui_obd2_fresh, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ui_obd2_fresh, 6, 0);
    lv_obj_set_style_radius(ui_obd2_fresh, 4, 0);
    lv_obj_align(ui_obd2_fresh, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_label_set_text(ui_obd2_fresh, "NO DATA");

    // SoC hero number (left)
    ui_obd2_soc_num = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_soc_num, lv_color_hex(0x004000), 0);
    lv_obj_set_style_text_font(ui_obd2_soc_num, &lv_font_montserrat_42, 0);
    lv_obj_align(ui_obd2_soc_num, LV_ALIGN_TOP_LEFT, 60, 80);
    lv_label_set_text(ui_obd2_soc_num, "-- %");

    ui_obd2_soc_sub = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_soc_sub, lv_color_hex(0x444444), 0);
    lv_obj_set_style_text_font(ui_obd2_soc_sub, &lv_font_montserrat_20, 0);
    lv_obj_align(ui_obd2_soc_sub, LV_ALIGN_TOP_LEFT, 60, 138);
    lv_label_set_text(ui_obd2_soc_sub, "State of Charge");

    // Power hero number (right)
    ui_obd2_kw_num = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_kw_num, lv_color_hex(0x000080), 0);
    lv_obj_set_style_text_font(ui_obd2_kw_num, &lv_font_montserrat_42, 0);
    lv_obj_align(ui_obd2_kw_num, LV_ALIGN_TOP_LEFT, 440, 80);
    lv_label_set_text(ui_obd2_kw_num, "-- kW");

    ui_obd2_kw_sub = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_kw_sub, lv_color_hex(0x444444), 0);
    lv_obj_set_style_text_font(ui_obd2_kw_sub, &lv_font_montserrat_20, 0);
    lv_obj_align(ui_obd2_kw_sub, LV_ALIGN_TOP_LEFT, 440, 138);
    lv_label_set_text(ui_obd2_kw_sub, "Power (- discharge / + charge)");

    // Detail rows
    constexpr int DET_Y0 = 210;
    constexpr int DET_PITCH = 32;
    constexpr int COL_L = 60, COL_R = 440;

    auto make_row = [&](int x, int y, const char *placeholder) {
        lv_obj_t *l = lv_label_create(scr);
        lv_obj_set_style_text_color(l, lv_color_hex(0x000000), 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, x, y);
        lv_label_set_text(l, placeholder);
        return l;
    };
    ui_obd2_pack_v   = make_row(COL_L, DET_Y0 + 0 * DET_PITCH, "Pack:        -- V");
    ui_obd2_pack_a   = make_row(COL_L, DET_Y0 + 1 * DET_PITCH, "Current:     -- A");
    ui_obd2_temp     = make_row(COL_L, DET_Y0 + 2 * DET_PITCH, "Temp:        -- C");
    ui_obd2_cell_max = make_row(COL_R, DET_Y0 + 0 * DET_PITCH, "Cell max:    -- V");
    ui_obd2_cell_min = make_row(COL_R, DET_Y0 + 1 * DET_PITCH, "Cell min:    -- V");
    ui_obd2_delta    = make_row(COL_R, DET_Y0 + 2 * DET_PITCH, "Delta:       -- mV");

    ui_obd2_state = lv_label_create(scr);
    lv_obj_set_style_text_color(ui_obd2_state, lv_color_hex(0x666666), 0);
    lv_obj_align(ui_obd2_state, LV_ALIGN_BOTTOM_LEFT, 20, -40);
    lv_label_set_text(ui_obd2_state, "no frames");
}

static void refresh_obd2_timer(lv_timer_t *) {
    PackState p;
    uint32_t total;
    int8_t   t_min = 127, t_max = -127;
    bool     have_sensor_temps = false;

    portENTER_CRITICAL(&g_data_mux);
    p = g_pack;
    total = g_total_frames;
    for (int i = 0; i < NUM_CMUS; i++) {
        for (int j = 0; j < TEMPS_PER_CMU; j++) {
            if (g_cmu[i].temp_valid[j]) {
                int8_t t = g_cmu[i].temp_c[j];
                if (t < t_min) t_min = t;
                if (t > t_max) t_max = t;
                have_sensor_temps = true;
            }
        }
    }
    portEXIT_CRITICAL(&g_data_mux);

    const uint32_t now = millis();
    const bool have_any = p.have_373 || p.have_374;
    const uint32_t age = p.last_update_ms ? now - p.last_update_ms : 0xFFFFFFFFu;
    const bool fresh = have_any && age < STALE_DATA_MS;

    // Fresh / stale / no-data badge
    if (!have_any) {
        lv_label_set_text(ui_obd2_fresh, "NO DATA");
        lv_obj_set_style_bg_color(ui_obd2_fresh, lv_color_hex(0x808080), 0);
    } else if (fresh) {
        lv_label_set_text(ui_obd2_fresh, "FRESH");
        lv_obj_set_style_bg_color(ui_obd2_fresh, lv_color_hex(0x2E7D32), 0);
    } else {
        lv_label_set_text(ui_obd2_fresh, "STALE");
        lv_obj_set_style_bg_color(ui_obd2_fresh, lv_color_hex(0xB00000), 0);
    }

    if (p.have_374) {
        float soc = (p.soc1 + p.soc2) * 0.5f;
        lv_label_set_text_fmt(ui_obd2_soc_num, "%.1f %%", soc);
    }

    // Prefer the per-sensor min..max from the 0x6E1..0x6E4 stream.
    // Fall back to the 0x374 byte-4 max if we only have the pack summary.
    if (have_sensor_temps) {
        lv_label_set_text_fmt(ui_obd2_temp, "Temp:       %d..%d C", t_min, t_max);
    } else if (p.have_374) {
        lv_label_set_text_fmt(ui_obd2_temp, "Temp max:   %d C", (int)p.max_temp_c);
    }
    if (p.have_373) {
        // Color the kW value by direction (regen vs draw)
        uint32_t kw_color = (p.power_kw > 0.5f) ? 0x004000      // charging
                          : (p.power_kw < -0.5f) ? 0x000080     // discharging
                          : 0x444444;
        lv_obj_set_style_text_color(ui_obd2_kw_num, lv_color_hex(kw_color), 0);
        lv_label_set_text_fmt(ui_obd2_kw_num,   "%+.2f kW", p.power_kw);
        lv_label_set_text_fmt(ui_obd2_pack_v,   "Pack:       %.1f V", p.pack_v);
        lv_label_set_text_fmt(ui_obd2_pack_a,   "Current:    %+.1f A", p.pack_a);
        lv_label_set_text_fmt(ui_obd2_cell_max, "Cell max:   %.2f V", p.cell_max_v);
        lv_label_set_text_fmt(ui_obd2_cell_min, "Cell min:   %.2f V", p.cell_min_v);
        lv_label_set_text_fmt(ui_obd2_delta,    "Delta:      %.0f mV",
                              (p.cell_max_v - p.cell_min_v) * 1000.f);
    }

    if (!have_any) {
        lv_label_set_text_fmt(ui_obd2_state,
            "Waiting for 0x373/0x374 ...   total frames: %lu",
            (unsigned long)total);
    } else {
        lv_label_set_text_fmt(ui_obd2_state,
            "last frame %lu ms ago (%s)   pack frames: %lu",
            (unsigned long)age, fresh ? "fresh" : "STALE",
            (unsigned long)total);
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

    build_mode_select_screen();    // CAN init + test-task start happen later in enter_mode()
    Serial.println("Ready.");
}

void loop() {
    lv_timer_handler();
    delay(5);
}
