#include "YellowDisplayLovyanGFX.h"
#include "CYD2432S022CConstants.h"
#include <LovyanGFX.hpp>
#include <Tactility/Log.h>
#include <memory>

#define TAG "YellowDisplayLovyanGFX"

bool YellowDisplayLovyanGFX::createPanel(std::shared_ptr<lgfx::Panel_LCD>& outPanel) {
    // Create ST7789 panel
    auto panel = std::make_shared<lgfx::Panel_ST7789>();
    
    // Create and configure i80 (8-bit parallel) bus first
    auto bus = new lgfx::Bus_Parallel8();
    lgfx::Bus_Parallel8::config_t busCfg = {};
    
    // Configure i80 bus pins
    busCfg.freq_write = 16000000;  // 16MHz clock rate (based on successful openHASP implementation)
    busCfg.pin_wr = CYD_2432S022C_LCD_PIN_WR;
    busCfg.pin_rd = CYD_2432S022C_LCD_PIN_RD;
    busCfg.pin_rs = CYD_2432S022C_LCD_PIN_DC;  // RS = DC (data/command) pin
    
    // Configure data pins
    busCfg.pin_d0 = CYD_2432S022C_LCD_PIN_D0;
    busCfg.pin_d1 = CYD_2432S022C_LCD_PIN_D1;
    busCfg.pin_d2 = CYD_2432S022C_LCD_PIN_D2;
    busCfg.pin_d3 = CYD_2432S022C_LCD_PIN_D3;
    busCfg.pin_d4 = CYD_2432S022C_LCD_PIN_D4;
    busCfg.pin_d5 = CYD_2432S022C_LCD_PIN_D5;
    busCfg.pin_d6 = CYD_2432S022C_LCD_PIN_D6;
    busCfg.pin_d7 = CYD_2432S022C_LCD_PIN_D7;
    
    // Apply bus configuration
    bus->config(busCfg);
    panel->setBus(bus);
    
    // Configure panel
    lgfx::Panel_ST7789::config_t panelCfg{};
    
    // Panel config for the ST7789 240x320 display with correct settings
    // Following the successful openHASP implementation
    panelCfg.pin_cs = CYD_2432S022C_LCD_PIN_CS;
    panelCfg.pin_rst = CYD_2432S022C_LCD_PIN_RST; 
    panelCfg.pin_busy = -1;
    
    panelCfg.memory_width = CYD_2432S022C_LCD_HORIZONTAL_RESOLUTION;  // 240
    panelCfg.memory_height = CYD_2432S022C_LCD_VERTICAL_RESOLUTION;   // 320
    panelCfg.panel_width = CYD_2432S022C_LCD_HORIZONTAL_RESOLUTION;   // 240
    panelCfg.panel_height = CYD_2432S022C_LCD_VERTICAL_RESOLUTION;    // 320
    panelCfg.offset_x = 0;
    panelCfg.offset_y = 0;
    panelCfg.offset_rotation = 0;
    panelCfg.dummy_read_pixel = 8;
    panelCfg.dummy_read_bits = 1;
    panelCfg.readable = true;
    panelCfg.invert = false;       // Changed to false based on openHASP implementation
    panelCfg.rgb_order = false;    // RGB color order (false = BGR)
    panelCfg.dlen_16bit = false;   // 8-bit parallel interface
    panelCfg.bus_shared = false;   // Changed to false based on openHASP implementation
    
    // Apply panel configuration
    panel->config(panelCfg);

    // Log configuration
    TT_LOG_I(TAG, "=== ST7789 Display Configuration ===");
    TT_LOG_I(TAG, "Resolution: %dx%d pixels", 
             panelCfg.panel_width,
             panelCfg.panel_height);
    TT_LOG_I(TAG, "Control Pins - CS:%d, RST:%d, DC:%d",
             panelCfg.pin_cs,
             panelCfg.pin_rst,
             busCfg.pin_rs);
    TT_LOG_I(TAG, "Clock Frequency: %d MHz", busCfg.freq_write / 1000000);
    TT_LOG_I(TAG, "Invert Color: %s", panelCfg.invert ? "Enabled" : "Disabled");
    TT_LOG_I(TAG, "Bus Shared: %s", panelCfg.bus_shared ? "Yes" : "No");
    TT_LOG_I(TAG, "=================================");

    outPanel = panel;
    return true;
}