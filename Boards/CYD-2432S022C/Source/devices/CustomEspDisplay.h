#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <Tactility/Lock.h>
#include <memory>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

class CustomEspDisplay : public tt::hal::display::DisplayDevice {
private:
    std::shared_ptr<tt::hal::touch::TouchDevice> touchDevice;
    std::shared_ptr<tt::Lock> lock;

    // ESP-IDF LCD handles
    esp_lcd_i80_bus_handle_t i80_bus = nullptr;
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_handle_t panel_handle = nullptr;
    
    // LVGL objects
    lv_display_t* lvglDisplay = nullptr;
    lv_draw_buf_t* draw_buf1 = nullptr;
    lv_draw_buf_t* draw_buf2 = nullptr;
    void* buf1_memory = nullptr;
    void* buf2_memory = nullptr;

    // Display dimensions
    static constexpr uint16_t LCD_H_RES = 240;
    static constexpr uint16_t LCD_V_RES = 320;
    static constexpr uint16_t DRAW_BUF_HEIGHT = 32; // 32 rows buffer

    // Helper methods
    bool initI80Bus();
    bool initPanel();
    bool sendST7789InitCommands();
    void cleanupResources();

public:
    explicit CustomEspDisplay(std::shared_ptr<tt::Lock> lock);
    ~CustomEspDisplay() override;

    // Device interface
    std::string getName() const override { return "CYD-2432S022C Display"; }
    std::string getDescription() const override { return "320x240 ST7789 i8080 LCD"; }
    
    // DisplayDevice interface
    std::shared_ptr<tt::hal::touch::TouchDevice> getTouchDevice() override { return touchDevice; }
    void setTouchDevice(std::shared_ptr<tt::hal::touch::TouchDevice> device) { touchDevice = device; }
    std::shared_ptr<tt::Lock> getLock() const { return lock; }

    // Start/stop
    bool start() override;
    bool stop() override;

    // LVGL integration
    bool supportsLvgl() const override { return true; }
    bool startLvgl() override;
    bool stopLvgl() override;
    lv_display_t* getLvglDisplay() const override { return lvglDisplay; }

    // DisplayDriver integration (optional - can be implemented later)
    bool supportsDisplayDriver() const override { return false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> getDisplayDriver() override { return nullptr; }

private:
    // LVGL flush callback
    static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
};