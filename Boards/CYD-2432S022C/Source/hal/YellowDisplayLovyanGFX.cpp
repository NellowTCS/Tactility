#include "YellowDisplayLovyanGFX.h"
#include "CYD2432S022CConstants.h"
#include <LovyanGFX.hpp>
#include <Tactility/Log.h>
#include <memory>

#define TAG "YellowDisplayLovyanGFX"

bool YellowDisplayLovyanGFX::createPanel(std::shared_ptr<lgfx::Panel_LCD>& outPanel) {
    // Create ST7789 panel
    auto panel = std::make_shared<lgfx::Panel_ST7789>();
    
    // Configure panel
    lgfx::Panel_ST7789::config_t panelCfg{};
    
    // Interface config (pins)
    panelCfg.pin_cs = CYD_2432S022C_LCD_PIN_CS;
    panelCfg.pin_rst = CYD_2432S022C_LCD_PIN_RST;
    panelCfg.pin_busy = -1;  // Not used
    panelCfg.bus_shared = true;
    
    // Panel config
    panelCfg.panel_width = CYD_2432S022C_LCD_HORIZONTAL_RESOLUTION;
    panelCfg.panel_height = CYD_2432S022C_LCD_VERTICAL_RESOLUTION;
    panelCfg.offset_x = 0;
    panelCfg.offset_y = 0;
    panelCfg.offset_rotation = 0;
    panelCfg.dummy_read_pixel = 8;
    panelCfg.dummy_read_bits = 1;
    panelCfg.readable = true;
    panelCfg.invert = true;  // IPS ST7789
    panelCfg.rgb_order = false;
    panelCfg.dlen_16bit = false;
    panelCfg.bus_shared = true;

    // Create and configure SPI bus
    auto bus = new lgfx::Bus_SPI();
    lgfx::Bus_SPI::config_t busCfg = {};
    busCfg.spi_mode = 0;
    busCfg.spi_3wire = false;
    bus->config(busCfg);

    // Apply configuration
    panel->setBus(bus);
    panel->config(panelCfg);

    // Log configuration
    TT_LOG_I(TAG, "=== ST7789 Display Configuration ===");
    TT_LOG_I(TAG, "Resolution: %dx%d pixels", 
             panelCfg.panel_width,
             panelCfg.panel_height);
    TT_LOG_I(TAG, "Control Pins - CS:%d, RST:%d",
             panelCfg.pin_cs,
             panelCfg.pin_rst);
    TT_LOG_I(TAG, "Invert Color: %s", panelCfg.invert ? "Enabled" : "Disabled");
    TT_LOG_I(TAG, "=================================");

    outPanel = panel;
    return true;
}