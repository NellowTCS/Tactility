#include "hal/YellowDisplay.h"
#include "Cst816Touch.h"
#include "hal/YellowDisplayConstants.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_log.h>

static const char* TAG = "DISPLAY";

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
        esp_lcd_i80_bus_handle_t i80_bus = nullptr;
        esp_lcd_i80_bus_config_t bus_config = {
            .dc_gpio_num = GPIO_NUM_16,               // 1st
            .wr_gpio_num = GPIO_NUM_4,                // 2nd
            .clk_src = LCD_CLK_SRC_DEFAULT,           // 3rd
            .data_gpio_nums = {GPIO_NUM_15, GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32}, // 4th
            .bus_width = 8,                           // 5th
            .max_transfer_bytes = 240 * 320 * 2,      // 6th
            .psram_trans_align = 64,                  // 7th
            .dma_burst_size = 64,                     // 8th (new field, reasonable default)
            .sram_trans_align = 4                     // 9th
        };
        ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

        esp_lcd_panel_io_i80_config_t io_config = {
            .cs_gpio_num = GPIO_NUM_17,               // 1st
            .pclk_hz = 12000000,                      // 2nd
            .trans_queue_depth = 10,                  // 3rd
            .on_color_trans_done = NULL,              // 4th
            .user_ctx = NULL,                         // 5th
            .lcd_cmd_bits = 8,                        // 6th
            .lcd_param_bits = 8,                      // 7th
            .dc_levels = {                            // 8th
                .dc_idle_level = 0,
                .dc_cmd_level = 1,
                .dc_dummy_level = 0,
                .dc_data_level = 0
            },
            .flags = {                                // 9th
                .cs_active_high = 0,
                .reverse_color_bits = 0,
                .swap_color_bytes = 0,
                .pclk_active_neg = 0,
                .pclk_idle_low = 0
            }
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = GPIO_NUM_NC,            // 1st
            .rgb_ele_order = ESP_LCD_COLOR_SPACE_RGB, // 2nd
            .bits_per_pixel = 16,                     // 3rd
            .flags = {                                // 4th
                .reset_active_high = 0
            },
            .data_endian = ESP_LCD_DATA_ENDIAN_LITTLE,// 5th
            .vendor_config = NULL                     // 6th
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

        lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,
            .panel_handle = panel_handle,
            .control_handle = NULL,
            .buffer_size = 240 * 320,
            .double_buffer = false,
            .trans_size = 0,
            .hres = 240,
            .vres = 320,
            .monochrome = false,
            .rotation = {
                .swap_xy = false,
                .mirror_x = false,
                .mirror_y = false
            },
            .color_format = LV_COLOR_FORMAT_RGB565,
            .flags = {
                .buff_dma = 0,
                .buff_spiram = 0,
                .sw_rotate = 0,
                .swap_bytes = 0,
                .full_refresh = 0,
                .direct_mode = 0
            }
        };
        display_handle = lvgl_port_add_disp(&disp_cfg);
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
