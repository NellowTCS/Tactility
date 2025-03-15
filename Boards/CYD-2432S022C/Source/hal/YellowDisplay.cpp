#include "YellowDisplay.h"
#include "Cst816Touch.h"
#include "YellowDisplayConstants.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_log.h>

static const char* TAG = "DISPLAY";

// Touch creation function (unchanged for brevity)
static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto configuration = std::make_unique<Cst816sTouch::Configuration>(I2C_NUM_0, 240, 320);
    return std::make_shared<Cst816sTouch>(std::move(configuration));
}

class CustomI80Display : public tt::hal::display::DisplayDevice {
private:
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_handle_t panel_handle = nullptr;
    lv_display_t* display_handle = nullptr;
    std::shared_ptr<tt::hal::touch::TouchDevice> touch;

public:
    CustomI80Display(std::shared_ptr<tt::hal::touch::TouchDevice> touch_dev) : touch(touch_dev) {
        // Initialize I80 bus
        esp_lcd_i80_bus_handle_t i80_bus = nullptr;
        esp_lcd_i80_bus_config_t bus_config = {
            .dc_gpio_num = 16,  // DC pin
            .wr_gpio_num = 4,   // WR pin
            .data_gpio_nums = {15, 13, 12, 14, 27, 25, 33, 32},  // 8 data lines
            .bus_width = 8,
            .max_transfer_bytes = 240 * 320 * sizeof(lv_color_t),  // Full screen buffer
            .clk_src = LCD_CLK_SRC_PLL160M,
            .psram_trans_align = 64,
            .sram_trans_align = 4
        };
        ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

        // Configure I80 panel I/O
        esp_lcd_panel_io_i80_config_t io_config = {
            .cs_gpio_num = 17,  // CS pin
            .pclk_hz = 12000000,  // Pixel clock (12 MHz)
            .trans_queue_depth = 10,
            .dc_levels = {0, 1, 0, 0},  // DC idle, cmd, dummy, data levels
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = { .cs_active_high = 0 }
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

        // Initialize ST7789 panel
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = GPIO_NUM_NC,  // No reset pin
            .color_space = ESP_LCD_COLOR_SPACE_RGB,
            .bits_per_pixel = 16,  // RGB565
            .data_endian = ESP_LCD_DATA_ENDIAN_BIG,
            .flags = { .reset_active_high = 0 },
            .vendor_config = nullptr
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));  // Adjust as needed

        // LVGL display configuration
        lvgl_port_display_cfg_t lvgl_cfg = {
            .io_handle = io_handle,
            .panel_handle = panel_handle,
            .buffer_size = 240 * 320,  // Full screen buffer
            .double_buffer = false,
            .trans_size = 0,  // No SRAM transfer buffer
            .hres = 240,  // Horizontal resolution
            .vres = 320,  // Vertical resolution
            .monochrome = false,
            .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },  // HW mirroring
#if LVGL_VERSION_MAJOR >= 9
            .color_format = LV_COLOR_FORMAT_NATIVE,
#endif
            .flags = {
                .buff_dma = 0,
                .buff_spiram = 0,
                .sw_rotate = 0,
#if LVGL_VERSION_MAJOR >= 9
                .swap_bytes = 0,
#endif
                .full_refresh = 1,  // Redraw full screen
                .direct_mode = 0
            }
        };
        display_handle = lvgl_port_add_disp(&lvgl_cfg);
        if (!display_handle) {
            ESP_LOGE(TAG, "LVGL display initialization failed");
        }
    }

    ~CustomI80Display() {
        if (display_handle) lvgl_port_remove_disp(display_handle);
        if (panel_handle) esp_lcd_panel_del(panel_handle);
        if (io_handle) esp_lcd_panel_io_del(io_handle);
    }

    std::string getName() const override { return "CustomI80ST7789"; }
    std::string getDescription() const override { return "Custom I80 ST7789 for CYD-2432S022C"; }
    bool start() override { return display_handle != nullptr; }
    bool stop() override { return true; }
    std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() override { return touch; }
    void setBacklightDuty(uint8_t duty) override { /* Add PWM backlight code if needed */ }
    bool supportsBacklightDuty() const override { return true; }
    void setGammaCurve(uint8_t) override {}
    uint8_t getGammaCurveCount() const override { return 1; }
    lv_display_t* getLvglDisplay() const override { return display_handle; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch = createTouch();
    auto display = std::make_shared<CustomI80Display>(touch);
    ESP_LOGI(TAG, "Initialized I80 ST7789: DC=16, WR=4, CS=17");
    return display;
}
