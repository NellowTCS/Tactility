#include "CustomEspDisplay.h"
#include "Constants.h"
#include <Tactility/Log.h>
#include <Tactility/Check.h>
#include "esp_heap_caps.h"
#include <cstring>

#define TAG "CustomEspDisplay"

// ST7789 Commands (from LovyanGFX Panel_ST7789.hpp)
#define ST7789_CMD_SWRESET   0x01
#define ST7789_CMD_SLPOUT    0x11
#define ST7789_CMD_NORON     0x13
#define ST7789_CMD_IDMOFF    0x38
#define ST7789_CMD_MADCTL    0x36
#define ST7789_CMD_COLMOD    0x3A
#define ST7789_CMD_RAMCTRL   0xB0
#define ST7789_CMD_PORCTRL   0xB2
#define ST7789_CMD_GCTRL     0xB7
#define ST7789_CMD_VCOMS     0xBB
#define ST7789_CMD_LCMCTRL   0xC0
#define ST7789_CMD_VDVVRHEN  0xC2
#define ST7789_CMD_VRHS      0xC3
#define ST7789_CMD_VDVSET    0xC4
#define ST7789_CMD_FRCTR2    0xC6
#define ST7789_CMD_PWCTRL1   0xD0
#define ST7789_CMD_PVGAMCTRL 0xE0
#define ST7789_CMD_NVGAMCTRL 0xE1
#define ST7789_CMD_DISPON    0x29
#define ST7789_CMD_CASET     0x2A
#define ST7789_CMD_RASET     0x2B
#define ST7789_CMD_RAMWR     0x2C

CustomEspDisplay::CustomEspDisplay(std::shared_ptr<tt::Lock> lock) : lock(lock) {
    TT_LOG_I(TAG, "Creating ESP32 native i80 display driver for ST7789");
}

CustomEspDisplay::~CustomEspDisplay() {
    stop();
}

bool CustomEspDisplay::start() {
    TT_LOG_I(TAG, "Starting ESP32 native i80 display");

    // 1. Initialize I80 bus
    if (!initI80Bus()) {
        TT_LOG_E(TAG, "Failed to initialize I80 bus");
        return false;
    }

    // 2. Initialize the panel
    if (!initPanel()) {
        TT_LOG_E(TAG, "Failed to initialize panel");
        cleanupResources();
        return false;
    }

    // 3. Start LVGL with memory-safe buffer sizing
    if (!startLvgl()) {
        TT_LOG_E(TAG, "Failed to start LVGL");
        stop(); // Clean up everything
        return false;
    }

    TT_LOG_I(TAG, "ESP32 native i80 display started successfully");
    return true;
}

bool CustomEspDisplay::stop() {
    TT_LOG_I(TAG, "Stopping ESP32 native display");
    
    if (lvglDisplay) {
        stopLvgl();
    }
    
    cleanupResources();
    return true;
}

bool CustomEspDisplay::initI80Bus() {
    // Configure the I80 bus
    esp_lcd_i80_bus_config_t bus_config = {};
    bus_config.clk_src = LCD_CLK_SRC_DEFAULT;
    bus_config.dc_gpio_num = CYD_2432S022C_LCD_PIN_DC;
    bus_config.wr_gpio_num = CYD_2432S022C_LCD_PIN_WR;
    bus_config.data_gpio_nums[0] = CYD_2432S022C_LCD_PIN_D0;
    bus_config.data_gpio_nums[1] = CYD_2432S022C_LCD_PIN_D1;
    bus_config.data_gpio_nums[2] = CYD_2432S022C_LCD_PIN_D2;
    bus_config.data_gpio_nums[3] = CYD_2432S022C_LCD_PIN_D3;
    bus_config.data_gpio_nums[4] = CYD_2432S022C_LCD_PIN_D4;
    bus_config.data_gpio_nums[5] = CYD_2432S022C_LCD_PIN_D5;
    bus_config.data_gpio_nums[6] = CYD_2432S022C_LCD_PIN_D6;
    bus_config.data_gpio_nums[7] = CYD_2432S022C_LCD_PIN_D7;
    bus_config.bus_width = CYD_2432S022C_LCD_BUS_WIDTH;
    bus_config.max_transfer_bytes = LCD_H_RES * DRAW_BUF_HEIGHT * sizeof(uint16_t);
    bus_config.psram_trans_align = 64;
    bus_config.sram_trans_align = 4;

    esp_err_t ret = esp_lcd_new_i80_bus(&bus_config, &i80_bus);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create I80 bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure the panel IO for i80 interface
    esp_lcd_panel_io_i80_config_t io_config = {};
    io_config.cs_gpio_num = CYD_2432S022C_LCD_PIN_CS;
    io_config.pclk_hz = CYD_2432S022C_LCD_PCLK_HZ;
    io_config.trans_queue_depth = 10;
    io_config.dc_levels = {
        .dc_idle_level = 0,
        .dc_cmd_level = 0,
        .dc_dummy_level = 0,
        .dc_data_level = 1,
    };
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.flags.swap_color_bytes = false; // RGB565 format

    ret = esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return false;
    }

    TT_LOG_I(TAG, "I80 bus initialized successfully");
    return true;
}

bool CustomEspDisplay::initPanel() {
    // For i80 ST7789, we use a custom approach since ESP-IDF doesn't have built-in support
    // We'll create a custom panel that manually handles ST7789 initialization
    
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = CYD_2432S022C_LCD_PIN_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    
    // Create a custom panel for ST7789 i80
    panel_handle = (esp_lcd_panel_handle_t)malloc(128);
    if (!panel_handle) {
        TT_LOG_E(TAG, "Failed to allocate panel handle");
        return false;
    }

    // Initialize the panel structure
    memset(panel_handle, 0, 128);

    // Perform hardware reset if reset pin is available
    if (panel_config.reset_gpio_num != GPIO_NUM_NC) {
        gpio_config_t reset_gpio_config = {
            .pin_bit_mask = 1ULL << panel_config.reset_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&reset_gpio_config);
        
        // Reset sequence
        gpio_set_level((gpio_num_t)panel_config.reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)panel_config.reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // Send ST7789 initialization commands
    if (!sendST7789InitCommands()) {
        TT_LOG_E(TAG, "Failed to send ST7789 init commands");
        free(panel_handle);
        panel_handle = nullptr;
        return false;
    }

    TT_LOG_I(TAG, "ST7789 i80 panel initialized successfully");
    TT_LOG_I(TAG, "Display resolution: %dx%d", LCD_H_RES, LCD_V_RES);
    return true;
}

bool CustomEspDisplay::sendST7789InitCommands() {
    esp_err_t ret;
    
    // Based on LovyanGFX Panel_ST7789.hpp initialization sequence
    TT_LOG_I(TAG, "Sending ST7789 initialization commands (LovyanGFX sequence)");

    // Sleep out - moved to beginning like LovyanGFX does
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_SLPOUT, NULL, 0);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "SLPOUT failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(130)); // 130ms delay as per LovyanGFX

    // Normal display mode on
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_NORON, NULL, 0);
    if (ret != ESP_OK) return false;

    // Porch Setting - exact values from LovyanGFX
    uint8_t porctrl_data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PORCTRL, porctrl_data, sizeof(porctrl_data));
    if (ret != ESP_OK) return false;

    // Gate Control - LovyanGFX value
    uint8_t gctrl_data = 0x35;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_GCTRL, &gctrl_data, 1);
    if (ret != ESP_OK) return false;

    // VCOM Setting - LovyanGFX value (0x28 for JLX240 display datasheet)
    uint8_t vcoms_data = 0x28;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VCOMS, &vcoms_data, 1);
    if (ret != ESP_OK) return false;

    // LCM Control - LovyanGFX value
    uint8_t lcmctrl_data = 0x0C;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_LCMCTRL, &lcmctrl_data, 1);
    if (ret != ESP_OK) return false;

    // VDV and VRH Command Enable - LovyanGFX values
    uint8_t vdvvrhen_data[] = {0x01, 0xFF};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VDVVRHEN, vdvvrhen_data, sizeof(vdvvrhen_data));
    if (ret != ESP_OK) return false;

    // VRH Set - LovyanGFX value (voltage VRHS)
    uint8_t vrhs_data = 0x10;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VRHS, &vrhs_data, 1);
    if (ret != ESP_OK) return false;

    // VDV Set - LovyanGFX value
    uint8_t vdvset_data = 0x20;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VDVSET, &vdvset_data, 1);
    if (ret != ESP_OK) return false;

    // Frame Rate Control 2 - LovyanGFX value (0x0f=60Hz)
    uint8_t frctr2_data = 0x0F;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_FRCTR2, &frctr2_data, 1);
    if (ret != ESP_OK) return false;

    // Power Control 1 - LovyanGFX values
    uint8_t pwctrl1_data[] = {0xA4, 0xA1};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PWCTRL1, pwctrl1_data, sizeof(pwctrl1_data));
    if (ret != ESP_OK) return false;

    // RAM Control - LovyanGFX values
    uint8_t ramctrl_data[] = {0x00, 0xC0};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_RAMCTRL, ramctrl_data, sizeof(ramctrl_data));
    if (ret != ESP_OK) return false;

    // Positive Voltage Gamma Control - exact LovyanGFX values
    uint8_t pvgamctrl_data[] = {
        0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44,
        0x42, 0x06, 0x0E, 0x12, 0x14, 0x17
    };
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PVGAMCTRL, pvgamctrl_data, sizeof(pvgamctrl_data));
    if (ret != ESP_OK) return false;

    // Negative Voltage Gamma Control - exact LovyanGFX values
    uint8_t nvgamctrl_data[] = {
        0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54,
        0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E
    };
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_NVGAMCTRL, nvgamctrl_data, sizeof(nvgamctrl_data));
    if (ret != ESP_OK) return false;

    // Idle Mode OFF - LovyanGFX sequence
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_IDMOFF, NULL, 0);
    if (ret != ESP_OK) return false;

    // Memory Data Access Control - set orientation (0x00 = normal)
    uint8_t madctl_data = 0x00;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_MADCTL, &madctl_data, 1);
    if (ret != ESP_OK) return false;

    // Interface Pixel Format - RGB565 (16-bit)
    uint8_t colmod_data = 0x55;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_COLMOD, &colmod_data, 1);
    if (ret != ESP_OK) return false;

    // Display on - final step
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_DISPON, NULL, 0);
    if (ret != ESP_OK) return false;

    TT_LOG_I(TAG, "ST7789 initialization commands sent successfully");
    return true;
}

bool CustomEspDisplay::startLvgl() {
    if (lvglDisplay) return true;

    const size_t buf_pixel_count = LCD_H_RES * DRAW_BUF_HEIGHT;
    const size_t buf_bytes = buf_pixel_count * sizeof(lv_color_t);

    // Allocate DMA-capable internal memory
    buf1_memory = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    buf2_memory = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    if (!buf1_memory || !buf2_memory) {
        TT_LOG_E(TAG, "Failed to allocate LVGL buffers");
        if (buf1_memory) heap_caps_free(buf1_memory);
        if (buf2_memory) heap_caps_free(buf2_memory);
        buf1_memory = buf2_memory = nullptr;
        return false;
    }

    // Create LVGL draw buffers
    draw_buf1 = lv_draw_buf_create((lv_color_t*)buf1_memory, buf_pixel_count);
    draw_buf2 = lv_draw_buf_create((lv_color_t*)buf2_memory, buf_pixel_count);

    if (!draw_buf1 || !draw_buf2) {
        TT_LOG_E(TAG, "Failed to create LVGL draw buffers");
        stopLvgl();
        return false;
    }

    // Create LVGL display
    lvglDisplay = lv_display_create(LCD_H_RES, LCD_V_RES);
    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create LVGL display");
        stopLvgl();
        return false;
    }

    lv_display_set_user_data(lvglDisplay, this);
    lv_display_set_draw_buffers(lvglDisplay, draw_buf1, draw_buf2);
    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(lvglDisplay, lvgl_flush_cb);

    TT_LOG_I(TAG, "LVGL started successfully with buffer height %d", DRAW_BUF_HEIGHT);
    return true;
}

bool CustomEspDisplay::stopLvgl() {
    TT_LOG_I(TAG, "Stopping LVGL");

    if (lvglDisplay) {
        lv_display_delete(lvglDisplay);
        lvglDisplay = nullptr;
    }

    if (draw_buf1) {
        lv_draw_buf_destroy(draw_buf1);
        draw_buf1 = nullptr;
    }

    if (draw_buf2) {
        lv_draw_buf_destroy(draw_buf2);
        draw_buf2 = nullptr;
    }

    if (buf1_memory) {
        heap_caps_free(buf1_memory);
        buf1_memory = nullptr;
    }

    if (buf2_memory) {
        heap_caps_free(buf2_memory);
        buf2_memory = nullptr;
    }

    return true;
}

void CustomEspDisplay::lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* display = static_cast<CustomEspDisplay*>(lv_display_get_user_data(disp));
    
    // Calculate coordinates and dimensions
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;
    
    // Send draw bitmap command manually since we're using custom panel
    // Set column address (CASET)
    uint8_t col_data[] = {
        (uint8_t)((x1 >> 8) & 0xFF), (uint8_t)(x1 & 0xFF),
        (uint8_t)((x2 >> 8) & 0xFF), (uint8_t)(x2 & 0xFF)
    };
    esp_lcd_panel_io_tx_param(display->io_handle, ST7789_CMD_CASET, col_data, 4);
    
    // Set page address (RASET)
    uint8_t page_data[] = {
        (uint8_t)((y1 >> 8) & 0xFF), (uint8_t)(y1 & 0xFF),
        (uint8_t)((y2 >> 8) & 0xFF), (uint8_t)(y2 & 0xFF)
    };
    esp_lcd_panel_io_tx_param(display->io_handle, ST7789_CMD_RASET, page_data, 4);
    
    // Send pixel data (RAMWR)
    size_t pixel_count = (x2 - x1 + 1) * (y2 - y1 + 1);
    esp_lcd_panel_io_tx_color(display->io_handle, ST7789_CMD_RAMWR, px_map, pixel_count * 2);
    
    // Notify LVGL that flush is complete
    lv_display_flush_ready(disp);
}

void CustomEspDisplay::cleanupResources() {
    if (panel_handle) {
        free(panel_handle);
        panel_handle = nullptr;
    }

    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
        io_handle = nullptr;
    }

    if (i80_bus) {
        esp_lcd_del_i80_bus(i80_bus);
        i80_bus = nullptr;
    }
}
