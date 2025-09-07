#include "YellowDisplay.h"
#include "YellowTouch.h"
#include "CYD2432S022CConstants.h"
#include "St7789-i8080Display.h"
#include <Tactility/Log.h>
#include <esp_log.h>
#include <inttypes.h>

#define TAG "YellowDisplay"

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    TT_LOG_I(TAG, "Creating touch device");
    auto touch = createYellowTouch();
    if (!touch) {
        TT_LOG_E(TAG, "Failed to create touch device");
    }
    return touch;
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    TT_LOG_I(TAG, "Creating ST7789 I8080 display");

    // Create touch device
    auto touch = createTouch();
    if (!touch) {
        TT_LOG_E(TAG, "Touch device creation failed, proceeding without touch");
    }

    // Create configuration
    auto configuration = std::make_unique<St7789I8080Display::Configuration>(
        CYD_2432S022C_LCD_PIN_WR,        // WR pin
        CYD_2432S022C_LCD_PIN_DC,        // DC pin
        CYD_2432S022C_LCD_PIN_CS,        // CS pin
        CYD_2432S022C_LCD_PIN_RST,       // RST pin
        CYD_2432S022C_LCD_PIN_BACKLIGHT, // Backlight pin
        CYD_2432S022C_LCD_HORIZONTAL_RESOLUTION, // Horizontal resolution
        CYD_2432S022C_LCD_VERTICAL_RESOLUTION,   // Vertical resolution
        touch,                           // Touch device
        CYD_2432S022C_LCD_PCLK_HZ,       // Pixel clock
        false,                           // swapXY
        false,                           // mirrorX
        false,                           // mirrorY
        true,                            // invertColor (for IPS ST7789)
        0,                               // bufferSize (default: 1/10 screen size)
        CYD_2432S022C_LCD_BACKLIGHT_ON_LEVEL == 1 // backlightOnLevel
    );

    // Configure 8-bit data bus
    configuration->setDataPins8Bit(
        CYD_2432S022C_LCD_PIN_D0,
        CYD_2432S022C_LCD_PIN_D1,
        CYD_2432S022C_LCD_PIN_D2,
        CYD_2432S022C_LCD_PIN_D3,
        CYD_2432S022C_LCD_PIN_D4,
        CYD_2432S022C_LCD_PIN_D5,
        CYD_2432S022C_LCD_PIN_D6,
        CYD_2432S022C_LCD_PIN_D7
    );

    // Create and return display
    auto display = std::make_shared<St7789I8080Display>(std::move(configuration));
    
    // Use the fancy getConfiguration() method for comprehensive logging
    const auto* config = display->getConfiguration();
    
    ESP_LOGI(TAG, "=== ST7789 Display Configuration ===");
    ESP_LOGI(TAG, "Resolution: %dx%d pixels", config->horizontalResolution, config->verticalResolution);
    ESP_LOGI(TAG, "Pixel Clock: %" PRIu32 " Hz (%.1f MHz)", config->pixelClockHz, config->pixelClockHz / 1000000.0f);
    ESP_LOGI(TAG, "Bus Width: %" PRIu8 "-bit parallel", config->busWidth);
    
    // Control pins
    ESP_LOGI(TAG, "Control Pins - WR:%d, DC:%d, CS:%d, RST:%d, BL:%d", 
             config->pin_wr, config->pin_dc, config->pin_cs, config->pin_rst, config->pin_backlight);
    
    // Data pins (only log the ones being used based on bus width)
    ESP_LOGI(TAG, "Data Pins - D0:%d, D1:%d, D2:%d, D3:%d, D4:%d, D5:%d, D6:%d, D7:%d",
             config->dataPins[0], config->dataPins[1], config->dataPins[2], config->dataPins[3],
             config->dataPins[4], config->dataPins[5], config->dataPins[6], config->dataPins[7]);
    
    // Display settings
    ESP_LOGI(TAG, "Display Settings:");
    ESP_LOGI(TAG, "  Color Inversion: %s (for %s displays)", 
             config->invertColor ? "Enabled" : "Disabled",
             config->invertColor ? "IPS" : "TN");
    ESP_LOGI(TAG, "  Orientation - SwapXY:%s, MirrorX:%s, MirrorY:%s",
             config->swapXY ? "Yes" : "No",
             config->mirrorX ? "Yes" : "No", 
             config->mirrorY ? "Yes" : "No");
    ESP_LOGI(TAG, "  Backlight - GPIO:%d, ActiveLevel:%s", 
             config->pin_backlight,
             config->backlightOnLevel ? "HIGH" : "LOW");
    
    // Buffer configuration
    uint32_t actual_buffer_size = config->bufferSize == 0 ? 
        (config->horizontalResolution * config->verticalResolution / 10) : 
        config->bufferSize;
    ESP_LOGI(TAG, "Buffer Size: %" PRIu32 " pixels (%.1f KB)", 
             actual_buffer_size, 
             (actual_buffer_size * 2) / 1024.0f); // 2 bytes per pixel for RGB565
    
    // Touch integration
    ESP_LOGI(TAG, "Touch Device: %s", config->touch ? "Enabled" : "Disabled");
    if (config->touch) {
        ESP_LOGI(TAG, "  Touch Type: %s", config->touch->getName().c_str());
    }
    
    // PWM backlight
    ESP_LOGI(TAG, "PWM Backlight Control: %s", 
             config->backlightDutyFunction ? "Available" : "Not Available");
    
    ESP_LOGI(TAG, "=====================================");
    
    TT_LOG_I(TAG, "Display created successfully with comprehensive configuration");
    return display;
}
