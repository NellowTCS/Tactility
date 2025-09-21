#include "CustomEspDisplay.h"
#include "Constants.h"
#include <Tactility/Log.h>
#include <Tactility/Check.h>
#include "esp_heap_caps.h"
#include <cstring>

#define TAG "CustomEspDisplay"

// ST7789 Commands for manual initialization
#define ST7789_CMD_SWRESET 0x01
#define ST7789_CMD_SLPOUT  0x11
#define ST7789_CMD_NORON   0x13
#define ST7789_CMD_MADCTL  0x36
#define ST7789_CMD_COLMOD  0x3A
#define ST7789_CMD_PORCTRL 0xB2
#define ST7789_CMD_GCTRL   0xB7
#define ST7789_CMD_VCOMS   0xBB
#define ST7789_CMD_LCMCTRL 0xC0
#define ST7789_CMD_VDVVRHEN 0xC2
#define ST7789_CMD_VRHS    0xC3
#define ST7789_CMD_VDVS    0xC4
#define ST7789_CMD_FRCTRL2 0xC6
#define ST7789_CMD_PWCTRL1 0xD0
#define ST7789_CMD_PVGAMCTRL 0xE0
#define ST7789_CMD_NVGAMCTRL 0xE1
#define ST7789_CMD_DISPON  0x29

CustomEspDisplay::CustomEspDisplay(std::shared_ptr<tt::Lock> lock) : lock(lock) {
    TT_LOG_I(TAG, "Creating ST7789 i80 display driver for ST7789");
}

CustomEspDisplay::~CustomEspDisplay() {
    stop();
}

bool CustomEspDisplay::start() {
    TT_LOG_I(TAG, "Starting ST7789 i80 display");
    
    if (!initI80Bus()) {
        TT_LOG_E(TAG, "Failed to initialize I80 bus");
        return false;
    }

    if (!initPanel()) {
        TT_LOG_E(TAG, "Failed to initialize panel");
        cleanupResources();
        return false;
    }

    TT_LOG_I(TAG, "ST7789 i80 display started successfully");
    return true;
}

bool CustomEspDisplay::stop() {
    TT_LOG_I(TAG, "Stopping ST7789 i80 display");
    
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
    panel_handle = (esp_lcd_panel_handle_t)malloc(sizeof(esp_lcd_panel_t));
    if (!panel_handle) {
        TT_LOG_E(TAG, "Failed to allocate panel handle");
        return false;
    }

    // Initialize the panel structure
    memset(panel_handle, 0, sizeof(esp_lcd_panel_t));

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
        gpio_set_level(panel_config.reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(panel_config.reset_gpio_num, 1);
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
    
    // Software reset
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_SWRESET, NULL, 0);
    if (ret != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(120));

    // Sleep out
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_SLPOUT, NULL, 0);
    if (ret != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(120));

    // Normal display mode on
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_NORON, NULL, 0);
    if (ret != ESP_OK) return false;

    // Memory Data Access Control
    uint8_t madctl_data = 0x00; // Normal orientation
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_MADCTL, &madctl_data, 1);
    if (ret != ESP_OK) return false;

    // Interface Pixel Format - RGB565
    uint8_t colmod_data = 0x55; // 16-bit RGB565
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_COLMOD, &colmod_data, 1);
    if (ret != ESP_OK) return false;

    // Porch Setting
    uint8_t porctrl_data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PORCTRL, porctrl_data, sizeof(porctrl_data));
    if (ret != ESP_OK) return false;

    // Gate Control
    uint8_t gctrl_data = 0x35;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_GCTRL, &gctrl_data, 1);
    if (ret != ESP_OK) return false;

    // VCOM Setting
    uint8_t vcoms_data = 0x19;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VCOMS, &vcoms_data, 1);
    if (ret != ESP_OK) return false;

    // LCM Control
    uint8_t lcmctrl_data = 0x2C;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_LCMCTRL, &lcmctrl_data, 1);
    if (ret != ESP_OK) return false;

    // VDV and VRH Command Enable
    uint8_t vdvvrhen_data = 0x01;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VDVVRHEN, &vdvvrhen_data, 1);
    if (ret != ESP_OK) return false;

    // VRH Set
    uint8_t vrhs_data = 0x12;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VRHS, &vrhs_data, 1);
    if (ret != ESP_OK) return false;

    // VDV Set
    uint8_t vdvs_data = 0x20;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_VDVS, &vdvs_data, 1);
    if (ret != ESP_OK) return false;

    // Frame Rate Control 2
    uint8_t frctrl2_data = 0x0F;
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_FRCTRL2, &frctrl2_data, 1);
    if (ret != ESP_OK) return false;

    // Power Control 1
    uint8_t pwctrl1_data[] = {0xA4, 0xA1};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PWCTRL1, pwctrl1_data, sizeof(pwctrl1_data));
    if (ret != ESP_OK) return false;

    // Positive Voltage Gamma Control
    uint8_t pvgamctrl_data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_PVGAMCTRL, pvgamctrl_data, sizeof(pvgamctrl_data));
    if (ret != ESP_OK) return false;

    // Negative Voltage Gamma Control
    uint8_t nvgamctrl_data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_NVGAMCTRL, nvgamctrl_data, sizeof(nvgamctrl_data));
    if (ret != ESP_OK) return false;

    // Display on
    ret = esp_lcd_panel_io_tx_param(io_handle, ST7789_CMD_DISPON, NULL, 0);
    if (ret != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(120));

    return true;
}

bool CustomEspDisplay::startLvgl() {
    if (lvglDisplay != nullptr) {
        TT_LOG_W(TAG, "LVGL already started");
        return true;
    }

    // Calculate buffer size
    size_t buf_size = LCD_H_RES * DRAW_BUF_HEIGHT;
    size_t buf_bytes = buf_size * sizeof(lv_color_t);

    TT_LOG_I(TAG, "Allocating LVGL buffers: %d bytes each (%dx%d pixels)", 
             buf_bytes, LCD_H_RES, DRAW_BUF_HEIGHT);

    // Allocate buffer memory - use DMA capable internal memory
    buf1_memory = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    buf2_memory = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    if (!buf1_memory || !buf2_memory) {
        TT_LOG_E(TAG, "Failed to allocate LVGL buffers");
        if (buf1_memory) { heap_caps_free(buf1_memory); buf1_memory = nullptr; }
        if (buf2_memory) { heap_caps_free(buf2_memory); buf2_memory = nullptr; }
        return false;
    }

    // Create LVGL draw buffers
    draw_buf1 = lv_draw_buf_create(LCD_H_RES, DRAW_BUF_HEIGHT, LV_COLOR_FORMAT_RGB565, 0);
    draw_buf2 = lv_draw_buf_create(LCD_H_RES, DRAW_BUF_HEIGHT, LV_COLOR_FORMAT_RGB565, 0);

    if (!draw_buf1 || !draw_buf2) {
        TT_LOG_E(TAG, "Failed to create LVGL draw buffers");
        if (draw_buf1) { lv_draw_buf_destroy(draw_buf1); draw_buf1 = nullptr; }
        if (draw_buf2) { lv_draw_buf_destroy(draw_buf2); draw_buf2 = nullptr; }
        heap_caps_free(buf1_memory); buf1_memory = nullptr;
        heap_caps_free(buf2_memory); buf2_memory = nullptr;
        return false;
    }

    // Initialize draw buffers with our allocated memory
    lv_draw_buf_init(draw_buf1, LCD_H_RES, DRAW_BUF_HEIGHT, LV_COLOR_FORMAT_RGB565, 
                     LCD_H_RES * sizeof(lv_color_t), buf1_memory, buf_bytes);
    lv_draw_buf_init(draw_buf2, LCD_H_RES, DRAW_BUF_HEIGHT, LV_COLOR_FORMAT_RGB565, 
                     LCD_H_RES * sizeof(lv_color_t), buf2_memory, buf_bytes);

    // Create LVGL display
    lvglDisplay = lv_display_create(LCD_H_RES, LCD_V_RES);
    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create LVGL display");
        stopLvgl();
        return false;
    }

    // Configure LVGL display
    lv_display_set_user_data(lvglDisplay, this);
    lv_display_set_draw_buffers(lvglDisplay, draw_buf1, draw_buf2);
    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(lvglDisplay, lvgl_flush_cb);

    TT_LOG_I(TAG, "LVGL started successfully");
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
    // Set column address
    uint8_t col_data[] = {
        (x1 >> 8) & 0xFF, x1 & 0xFF,
        (x2 >> 8) & 0xFF, x2 & 0xFF
    };
    esp_lcd_panel_io_tx_param(display->io_handle, 0x2A, col_data, 4);
    
    // Set page address
    uint8_t page_data[] = {
        (y1 >> 8) & 0xFF, y1 & 0xFF,
        (y2 >> 8) & 0xFF, y2 & 0xFF
    };
    esp_lcd_panel_io_tx_param(display->io_handle, 0x2B, page_data, 4);
    
    // Send pixel data
    size_t pixel_count = (x2 - x1 + 1) * (y2 - y1 + 1);
    esp_lcd_panel_io_tx_color(display->io_handle, 0x2C, px_map, pixel_count * 2);
    
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