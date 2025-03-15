#include "YellowDisplay.h"
#include "Cst816Touch.h"
#include "YellowDisplayConstants.h"
#include "sdkconfig.h"
#include <PwmBacklight.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto configuration = std::make_unique<Cst816sTouch::Configuration>(
        I2C_NUM_0,
        240,
        320
    );
    return std::make_shared<Cst816sTouch>(std::move(configuration));
}

// Custom I80 ST7789 implementation
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
            .dc_gpio_num = CONFIG_LCD_I80_BUS_CONFIG_DC,  // 16
            .wr_gpio_num = CONFIG_LCD_I80_BUS_CONFIG_WR,  // 4
            .data_gpio_nums = {
                GPIO_NUM_15, GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14,
                GPIO_NUM_27, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32
            },
            .bus_width = CONFIG_LCD_I80_BUS_WIDTH,  // 8
            .max_transfer_bytes = LCD_I80_BUS_CONFIG_MAX_TRANSFER_BYTES
        };
        ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

        // Initialize panel I/O
        esp_lcd_panel_io_i80_config_t io_config = {
            .cs_gpio_num = LCD_I80_BUS_CONFIG_CS_GPIO_NUM,  // 17
            .pclk_hz = LCD_I80_BUS_CONFIG_PCLK_HZ,         // 12000000
            .trans_queue_depth = 10,
            .dc_levels = { .dc_idle_level = 0, .dc_cmd_level = 1, .dc_dummy_level = 0, .dc_data_level = 0 },
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

        // Initialize ST7789 panel
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = GPIO_NUM_NC,  // No reset pin specified
            .color_space = ESP_LCD_COLOR_SPACE_RGB,
            .bits_per_pixel = 16
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));  // Mirror X

        // Integrate with LVGL
        lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        display_handle = lvgl_port_add_disp(&lvgl_cfg, panel_handle, io_handle);
        if (!display_handle) {
            ESP_LOGE("DISPLAY", "LVGL display init failed");
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
    void setBacklightDuty(uint8_t duty) override { driver::pwmbacklight::setBacklightDuty(duty); }
    bool supportsBacklightDuty() const override { return true; }
    void setGammaCurve(uint8_t) override {}
    uint8_t getGammaCurveCount() const override { return 1; }
    lv_display_t* getLvglDisplay() const override { return display_handle; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch = createTouch();
    auto display = std::make_shared<CustomI80Display>(touch);
    ESP_LOGI("DISPLAY", "Initialized I80 ST7789: DC=%d, WR=%d, CS=%d", 16, 4, 17);
    return display;
}
