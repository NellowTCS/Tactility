#include "YellowDisplay.h"
#include "Cst816Touch.h"
#include "YellowDisplayConstants.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
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
            .clk_src = LCD_CLK_SRC_DEFAULT,      // Clock source
            .dc_gpio_num = 16,                   // Data/Command GPIO
            .wr_gpio_num = 4,                    // Write signal GPIO
            .data_gpio_nums = {15, 13, 12, 14, 27, 25, 33, 32}, // 8-bit data lines
            .bus_width = 8,                      // 8-bit parallel bus
            .max_transfer_bytes = 240 * 320 * 2, // Buffer size (assuming 16-bit color)
            .psram_trans_align = 64,             // PSRAM alignment
            .sram_trans_align = 4                // SRAM alignment
        };
        ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

        // Configure I80 panel I/O
        esp_lcd_panel_io_i80_config_t io_config = {
            .cs_gpio_num = 17,                   // Chip select GPIO
            .pclk_hz = 12000000,                 // Pixel clock frequency (12 MHz)
            .trans_queue_depth = 10,             // Transaction queue depth
            .dc_levels = {                       // DC signal levels
                .dc_idle_level = 0,
                .dc_cmd_level = 1,
                .dc_dummy_level = 0,
                .dc_data_level = 0
            },
            .on_color_trans_done = NULL,         // Callback (none)
            .user_ctx = NULL,                    // User context (none)
            .lcd_cmd_bits = 8,                   // Command bits
            .lcd_param_bits = 8                  // Parameter bits
            // .flags defaults to 0 if not specified
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

        // Initialize ST7789 panel
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = -1,                // No reset GPIO
            .rgb_ele_order = ESP_LCD_COLOR_SPACE_RGB, // Color space
            .bits_per_pixel = 16,                // 16-bit color depth
            .vendor_config = NULL                // No vendor-specific config
            // Note: data_endian is not a field here; handled elsewhere if needed
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));  // Adjust as needed

        // LVGL display configuration
        lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,              // From esp_lcd_panel_io_init
            .panel_handle = panel_handle,        // From esp_lcd_new_panel
            .buffer_size = 240 * 320,            // Single buffer size
            .double_buffer = false,              // No double buffering
            .trans_size = 0,                     // Default transfer size
            .hres = 240,                         // Horizontal resolution
            .vres = 320,                         // Vertical resolution
            .monochrome = false,                 // Color display
            .rotation = {0, 0, 0, 0},            // No rotation
            .color_format = LV_COLOR_FORMAT_RGB565, // 16-bit RGB565
            .flags = 0                           // No special flags
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

static std::vector<tt::hal::spi::Configuration> make_spi_configurations() {
    return {
        {
            .host = SPI3_HOST,
            .miso_pin = 19,  // Example GPIO
            .mosi_pin = 23,
            .sclk_pin = 18,
            .cs_pin = 5,
            .clock_speed_hz = 4000000  // 4 MHz
        }
    };
}
