#include "Display.h"

#include <Tactility/Log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/gpio.h>
#include <lvgl.h>
#include <array>
#include <cstdint>

constexpr auto TAG = "I8080St7789Display";

// LCD initialization command structure
typedef struct {
    uint8_t addr;
    uint8_t param[14];
    uint8_t len;
} lcd_cmd_t;

// ST7789 initialization command sequence
static lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},  // Sleep Out
    {0x3A, {0x05}, 1},      // Pixel Format Set
    {0xB2, {0x0B, 0x0B, 0x00, 0x33, 0x33}, 5},  // Porch Setting
    {0xB7, {0x75}, 1},      // Gate Control
    {0xBB, {0x28}, 1},      // VCOM Setting
    {0xC0, {0x2C}, 1},      // LCM Control
    {0xC2, {0x01}, 1},      // Display Timing
    {0xC3, {0x1F}, 1},      // Power Control
    {0xC6, {0x13}, 1},      // VCOM Offset
    {0xD0, {0xA7}, 1},      // Power Setting
    {0xE0, {0xF0, 0x05, 0x0A, 0x06, 0x06, 0x03, 0x2B, 0x32, 0x43, 0x36, 0x11, 0x10, 0x2B, 0x32}, 14},  // Gamma Positive
    {0xE1, {0xF0, 0x08, 0x0C, 0x0B, 0x09, 0x24, 0x2B, 0x22, 0x43, 0x38, 0x15, 0x16, 0x2F, 0x37}, 14},  // Gamma Negative
};

bool I8080St7789Display::initialize() {
    TT_LOG_I(TAG, "Initializing I8080 ST7789 Display...");

    // I80 bus config - order must match ESP-IDF struct!
    esp_lcd_i80_bus_config_t bus_cfg = {
        configuration->dcPin, // dc_gpio_num
        configuration->wrPin, // wr_gpio_num
        LCD_CLK_SRC_DEFAULT,  // clk_src
        {                     // data_gpio_nums
            configuration->dataPins[0],
            configuration->dataPins[1],
            configuration->dataPins[2],
            configuration->dataPins[3],
            configuration->dataPins[4],
            configuration->dataPins[5],
            configuration->dataPins[6],
            configuration->dataPins[7],
        },
        8, // bus_width
        configuration->bufferSize * sizeof(uint16_t), // max_transfer_bytes
        64, // dma_burst_size (psram_trans_align for older ESP-IDF)
        4   // sram_trans_align
    };

    if (esp_lcd_new_i80_bus(&bus_cfg, &i80BusHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create I8080 bus");
        return false;
    }

    TT_LOG_I(TAG, "I8080 bus initialized");

    // Configure the panel IO
    esp_lcd_panel_io_i80_config_t io_cfg = {
        configuration->csPin, // cs_gpio_num
        configuration->pixelClockFrequency, // pclk_hz
        configuration->transactionQueueDepth, // trans_queue_depth
        nullptr, // on_color_trans_done
        nullptr, // user_ctx
        8, // lcd_cmd_bits
        8, // lcd_param_bits
        { // dc_levels
            0, // dc_idle_level
            0, // dc_cmd_level
            0, // dc_dummy_level
            1  // dc_data_level
        },
        { // flags
            0, // cs_active_high
            0, // reverse_color_bits
            0, // swap_color_bytes
            0, // pclk_active_neg
            0  // pclk_idle_low
        }
    };

    if (esp_lcd_new_panel_io_i80(i80BusHandle, &io_cfg, &ioHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel IO");
        return false;
    }

    TT_LOG_I(TAG, "Panel IO initialized");

    // Panel config (named initializers for clarity)
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = configuration->resetPin,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .vendor_config = nullptr,
    };

    if (esp_lcd_new_panel_st7789(ioHandle, &panel_cfg, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel");
        return false;
    }

    TT_LOG_I(TAG, "Panel initialized");

    // Hard reset
    esp_lcd_panel_reset(panelHandle);

    // Send custom init commands
    for (auto& cmd : lcd_st7789v) {
        esp_lcd_panel_io_tx_param(ioHandle, cmd.addr, cmd.param, cmd.len & 0x7F);
        if (cmd.len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }

    // Turn on the display
    esp_lcd_panel_disp_on_off(panelHandle, true);

    // Enable the backlight
    gpio_config_t bk_gpio_cfg = {
        .pin_bit_mask = 1ULL << configuration->backlightPin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bk_gpio_cfg);
    gpio_set_level(configuration->backlightPin, 1);

    TT_LOG_I(TAG, "I8080 ST7789 Display initialized successfully");
    return true;
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto display = std::make_shared<I8080St7789Display>(std::make_unique<I8080St7789Display::Configuration>(
        GPIO_NUM_6,  // CS
        GPIO_NUM_7,  // DC
        GPIO_NUM_8,  // WR
        GPIO_NUM_9,  // RD
        {GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_47, GPIO_NUM_48},  // Data pins
        GPIO_NUM_5,  // Reset
        GPIO_NUM_38   // Backlight
    ));

    if (!display->initialize()) {
        TT_LOG_E(TAG, "Failed to initialize display");
        return nullptr;
    }

    return display;
}