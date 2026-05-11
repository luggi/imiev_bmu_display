/* ---------------------------------------------------------------------
 * LGFX_CrowPanel5.hpp
 *
 * LovyanGFX display + touch configuration for the Elecrow CrowPanel
 * 5-inch ESP32-S3 HMI (basic, 800x480 RGB, GT911 capacitive touch).
 *
 * Pinout is copied from Elecrow's official reference repo:
 *   github.com/Elecrow-RD/CrowPanel-5.0-HMI-ESP32-Display-800x480
 *
 * Include this header in exactly ONE .cpp file (main.cpp).
 * --------------------------------------------------------------------- */
#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB        _bus_instance;
    lgfx::Panel_RGB      _panel_instance;
    lgfx::Light_PWM      _light_instance;
    lgfx::Touch_GT911    _touch_instance;

    LGFX(void) {
        /* ---------------- RGB parallel bus ----------------- */
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;

            // 16-bit parallel RGB565 mapping (B0..B4, G0..G5, R0..R4)
            cfg.pin_d0  = GPIO_NUM_8;   // B0
            cfg.pin_d1  = GPIO_NUM_3;   // B1
            cfg.pin_d2  = GPIO_NUM_46;  // B2
            cfg.pin_d3  = GPIO_NUM_9;   // B3
            cfg.pin_d4  = GPIO_NUM_1;   // B4
            cfg.pin_d5  = GPIO_NUM_5;   // G0
            cfg.pin_d6  = GPIO_NUM_6;   // G1
            cfg.pin_d7  = GPIO_NUM_7;   // G2
            cfg.pin_d8  = GPIO_NUM_15;  // G3
            cfg.pin_d9  = GPIO_NUM_16;  // G4
            cfg.pin_d10 = GPIO_NUM_4;   // G5
            cfg.pin_d11 = GPIO_NUM_45;  // R0
            cfg.pin_d12 = GPIO_NUM_48;  // R1
            cfg.pin_d13 = GPIO_NUM_47;  // R2
            cfg.pin_d14 = GPIO_NUM_21;  // R3
            cfg.pin_d15 = GPIO_NUM_14;  // R4

            cfg.pin_henable = GPIO_NUM_40;
            cfg.pin_vsync   = GPIO_NUM_41;
            cfg.pin_hsync   = GPIO_NUM_39;
            cfg.pin_pclk    = GPIO_NUM_0;
            cfg.freq_write  = 14000000;     // 14 MHz; raise if you see no tearing

            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 8;
            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch  = 43;

            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch  = 12;

            cfg.pclk_active_neg = 1;
            cfg.de_idle_high    = 0;
            cfg.pclk_idle_high  = 0;
            _bus_instance.config(cfg);
        }

        /* ---------------- Panel ---------------------------- */
        {
            auto cfg = _panel_instance.config();
            cfg.memory_width  = 800;
            cfg.memory_height = 480;
            cfg.panel_width   = 800;
            cfg.panel_height  = 480;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            _panel_instance.config(cfg);
        }
        _panel_instance.setBus(&_bus_instance);

        /* ---------------- Backlight (PWM on GPIO 2) -------- */
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl      = GPIO_NUM_2;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        /* ---------------- GT911 touch (I2C) ---------------- */
        {
            auto cfg = _touch_instance.config();
            cfg.x_min       = 0;
            cfg.x_max       = 799;
            cfg.y_min       = 0;
            cfg.y_max       = 479;
            // CrowPanel 5" v3.0: TP_RST and TP_INT are NOT on ESP32 GPIOs.
            // They sit behind a PCA9557 I/O expander (IO0=RST, IO1=INT).
            // We do the GT911 address-latch reset manually in main.cpp before
            // calling lcd.init(); tell LovyanGFX to leave both lines alone.
            cfg.pin_int     = -1;
            cfg.pin_rst     = -1;
            cfg.bus_shared  = false;
            cfg.offset_rotation = 0;

            // I2C — shared bus with the PCA9557.
            cfg.i2c_port    = 1;
            cfg.pin_sda     = GPIO_NUM_19;
            cfg.pin_scl     = GPIO_NUM_20;
            cfg.freq        = 300000;     // 400 kHz is unreliable on this board
            cfg.i2c_addr    = 0x5D;       // INT held LOW during reset -> latches 0x5D
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};
