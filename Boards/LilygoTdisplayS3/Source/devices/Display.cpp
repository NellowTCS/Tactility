#include "Display.h"
#include <Tactility/Log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lvgl_port.h>
#include <driver/gpio.h>
#include <lvgl.h>
#include <array>
#include <cstdint>
#include <cstring>

constexpr auto TAG = "I8080St7789Display";
static I8080St7789Display* g_display_instance = nullptr;
static uint8_t buf1[170 * 320 / 10 * LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565)];


static void draw_test_pattern(esp_lcd_panel_handle_t panel, int w, int h) {
    if (!panel) return;

    const int chunk_rows = 16;
    const size_t buf_pixels = w * chunk_rows;
    uint16_t* buf = (uint16_t*)heap_caps_malloc(buf_pixels * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buf) {
        TT_LOG_E(TAG, "Failed to allocate test buffer");
        return;
    }

    struct { uint16_t color; const char* name; } passes[] = {
        { 0xF800, "RED" },
        { 0x07E0, "GREEN" },
        { 0x001F, "BLUE" }
    };

    for (auto& p : passes) {
        TT_LOG_I(TAG, "Test pass: %s", p.name);
        std::fill_n(buf, buf_pixels, p.color);

        for (int y = 0; y < h; y += chunk_rows) {
            int rows = std::min(chunk_rows, h - y);
            esp_lcd_panel_draw_bitmap(panel, 0, y, w, y + rows, buf);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    heap_caps_free(buf);
}

typedef struct {
    uint8_t addr;
    uint8_t param[14];
    uint8_t len;
} lcd_cmd_t;

static lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};

bool I8080St7789Display::initialize(lv_display_t* lvglDisplayCtx) {
    TT_LOG_I(TAG, "Initializing I8080 ST7789 Display...");

    if (configuration.rdPin != GPIO_NUM_NC) {
        gpio_config_t rd_gpio_config = {
            .pin_bit_mask = (1ULL << static_cast<uint32_t>(configuration.rdPin)),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&rd_gpio_config);
        gpio_set_level(configuration.rdPin, 1);
    }

    size_t max_bytes = configuration.bufferSize * sizeof(uint16_t);
    esp_lcd_i80_bus_config_t bus_cfg = {
        configuration.dcPin,
        configuration.wrPin,
        LCD_CLK_SRC_DEFAULT,
        {
            configuration.dataPins[0], configuration.dataPins[1], configuration.dataPins[2], configuration.dataPins[3],
            configuration.dataPins[4], configuration.dataPins[5], configuration.dataPins[6], configuration.dataPins[7],
        },
        8,
        max_bytes,
        64,
        4
    };
    if (esp_lcd_new_i80_bus(&bus_cfg, &i80BusHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create I80 bus");
        return false;
    }

    esp_lcd_panel_io_i80_config_t io_cfg = {
        configuration.csPin,
        configuration.pixelClockFrequency,
        configuration.transactionQueueDepth,
        nullptr, nullptr,
        8, 8,
        {0, 0, 0, 1},
        {0, 0, 0, 0, 0}
    };
    io_cfg.on_color_trans_done = [](esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void* user_ctx) -> bool {
        auto disp = static_cast<lv_display_t*>(user_ctx);
        if (disp) lv_display_flush_ready(disp);
        return false;
    };
    io_cfg.user_ctx = lvglDisplayCtx;

    if (esp_lcd_new_panel_io_i80(i80BusHandle, &io_cfg, &ioHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel IO");
        return false;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = configuration.resetPin,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .vendor_config = nullptr
    };
    if (esp_lcd_new_panel_st7789(ioHandle, &panel_cfg, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel");
        return false;
    }

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK ||
        esp_lcd_panel_init(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Panel reset/init failed");
        return false;
    }

    for (auto &cmd : lcd_st7789v) {
        esp_lcd_panel_io_tx_param(ioHandle, cmd.addr, cmd.param, cmd.len & 0x7F);
        if (cmd.len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }

    esp_lcd_panel_invert_color(panelHandle, true);
    esp_lcd_panel_swap_xy(panelHandle, true);
    esp_lcd_panel_mirror(panelHandle, false, true);
    esp_lcd_panel_set_gap(panelHandle, 0, 35);
    esp_lcd_panel_disp_on_off(panelHandle, true);

    gpio_config_t bk_gpio_cfg = {
        .pin_bit_mask = 1ULL << static_cast<uint32_t>(configuration.backlightPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bk_gpio_cfg);
    gpio_set_level(configuration.backlightPin, 1);

    TT_LOG_I(TAG, "Filling screen with red for test");

    size_t pixel_count = 170 * 320;
    uint16_t* red_buffer = (uint16_t*)heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!red_buffer) {
        TT_LOG_E(TAG, "Failed to allocate red buffer");
        return false;
    }

    std::fill_n(red_buffer, pixel_count, 0xF800);  // Fill with red
    esp_err_t err = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 170, 320, red_buffer);
    if (err != ESP_OK) {
        TT_LOG_E(TAG, "Draw red failed: %d", err);
    }

    heap_caps_free(red_buffer);

    draw_test_pattern(panelHandle, 170, 320);
    g_display_instance = this;

    TT_LOG_I(TAG, "Display initialized successfully");
    return true;
}

bool I8080St7789Display::startLvgl() {
    TT_LOG_I(TAG, "Creating LVGL ST7789 display");

    lvglDisplay = lv_st7789_create(170, 320, LV_LCD_FLAG_NONE,
        [this](lv_display_t*, const uint8_t* cmd, size_t, const uint8_t* param, size_t param_size) {
            esp_lcd_panel_io_tx_param(ioHandle, *cmd, param, param_size);
            if (param_size & 0x80) vTaskDelay(pdMS_TO_TICKS(120));
        },
        [this](lv_display_t*, const uint8_t* cmd, size_t, uint8_t* param, size_t param_size) {
            esp_lcd_panel_io_tx_color(ioHandle, *cmd, param, param_size);
        }
    );

    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create LVGL ST7789 display");
        return false;
    }

    lv_st7789_set_gap(lvglDisplay, 0, 35);
    lv_st7789_set_invert(lvglDisplay, true);
    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(lvglDisplay, buf1, nullptr, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_rotation(lvglDisplay, LV_DISPLAY_ROTATION_180);

    esp_lcd_panel_disp_on_off(panelHandle, true);

    TT_LOG_I(TAG, "LVGL ST7789 display registered");
    return true;
}


lv_display_t* I8080St7789Display::getLvglDisplay() const {
    return lvglDisplay;
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto display = std::make_shared<I8080St7789Display>(
        I8080St7789Display::Configuration(
            GPIO_NUM_7,   // CS
            GPIO_NUM_8,   // DC
            GPIO_NUM_5,   // WR
            GPIO_NUM_6,   // RD
            { GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42,
              GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_47, GPIO_NUM_48 }, // D0..D7
            GPIO_NUM_9,   // RST
            GPIO_NUM_38   // BL
        )
    );

    // No call to initialize() here — LVGL will trigger it via startLvgl()

    return display;
}
