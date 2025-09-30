#pragma once

#include "Tactility/hal/display/DisplayDevice.h"
#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <functional>
#include <lvgl.h>

class St7789I8080Display : public tt::hal::display::DisplayDevice {
public:
    class Configuration {
    public:
        Configuration(
            gpio_num_t wr,
            gpio_num_t dc,
            gpio_num_t cs,
            gpio_num_t rst,
            gpio_num_t backlight,
            uint16_t horizontalResolution,
            uint16_t verticalResolution,
            std::shared_ptr<tt::hal::touch::TouchDevice> touch,
            uint32_t pixelClockHz = 20'000'000,
            bool swapXY = false,
            bool mirrorX = false,
            bool mirrorY = false,
            bool invertColor = true,        // Default to true (most ST7789 are IPS)
            uint32_t bufferSize = 0,
            bool backlightOnLevel = true
        ) : pin_wr(wr),
            pin_dc(dc),
            pin_cs(cs),
            pin_rst(rst),
            pin_backlight(backlight),
            dataPins{},
            busWidth(8),
            horizontalResolution(horizontalResolution),
            verticalResolution(verticalResolution),
            pixelClockHz(pixelClockHz),
            swapXY(swapXY),
            mirrorX(mirrorX),
            mirrorY(mirrorY),
            invertColor(invertColor),
            bufferSize(bufferSize),
            backlightOnLevel(backlightOnLevel),
            touch(std::move(touch)),
            backlightDutyFunction(nullptr)
        {
            for (int i = 0; i < 16; i++) {
                dataPins[i] = GPIO_NUM_NC;
            }
        }

        // I8080 Bus configuration
        gpio_num_t pin_wr;          // Write strobe pin
        gpio_num_t pin_dc;          // Data/Command pin
        gpio_num_t pin_cs;          // Chip select pin
        gpio_num_t pin_rst;         // Reset pin
        gpio_num_t pin_backlight;   // Backlight control pin
        
        // Data pins (8-bit or 16-bit parallel interface)
        gpio_num_t dataPins[16];    // D0-D15 data pins
        uint8_t busWidth;           // 8 or 16 bit bus width
        
        // Display properties
        uint16_t horizontalResolution;  // Display width in pixels
        uint16_t verticalResolution;    // Display height in pixels
        uint32_t pixelClockHz;          // Pixel clock frequency
        
        // Display orientation and color settings
        bool swapXY;                // Swap X and Y coordinates (rotate 90°)
        bool mirrorX;               // Mirror X axis (horizontal flip)
        bool mirrorY;               // Mirror Y axis (vertical flip)
        bool invertColor;           // Enable color inversion (required for IPS, disable for TN)
        
        // Buffer configuration
        uint32_t bufferSize;        // Buffer size in pixels (0 = auto: screen_size/10)
        
        // Backlight configuration
        bool backlightOnLevel;      // true = active high, false = active low
        
        // Touch device integration
        std::shared_ptr<tt::hal::touch::TouchDevice> touch;
        
        // Optional PWM backlight control function
        std::function<void(uint8_t)> _Nullable backlightDutyFunction;

        // Helper methods to configure data pins
        void setDataPins8Bit(gpio_num_t d0, gpio_num_t d1, gpio_num_t d2, gpio_num_t d3,
                             gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7) {
            busWidth = 8;
            dataPins[0] = d0; dataPins[1] = d1; dataPins[2] = d2; dataPins[3] = d3;
            dataPins[4] = d4; dataPins[5] = d5; dataPins[6] = d6; dataPins[7] = d7;
            // Clear unused pins
            for (int i = 8; i < 16; i++) {
                dataPins[i] = GPIO_NUM_NC;
            }
        }

        void setDataPins16Bit(gpio_num_t d0, gpio_num_t d1, gpio_num_t d2, gpio_num_t d3,
                              gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7,
                              gpio_num_t d8, gpio_num_t d9, gpio_num_t d10, gpio_num_t d11,
                              gpio_num_t d12, gpio_num_t d13, gpio_num_t d14, gpio_num_t d15) {
            busWidth = 16;
            dataPins[0] = d0;   dataPins[1] = d1;   dataPins[2] = d2;   dataPins[3] = d3;
            dataPins[4] = d4;   dataPins[5] = d5;   dataPins[6] = d6;   dataPins[7] = d7;
            dataPins[8] = d8;   dataPins[9] = d9;   dataPins[10] = d10; dataPins[11] = d11;
            dataPins[12] = d12; dataPins[13] = d13; dataPins[14] = d14; dataPins[15] = d15;
        }

        // Helper methods to create common configurations
        static std::unique_ptr<Configuration> createForIPS(
            gpio_num_t wr, gpio_num_t dc, gpio_num_t cs, gpio_num_t rst, gpio_num_t backlight,
            uint16_t width, uint16_t height, std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr) {
            return std::make_unique<Configuration>(wr, dc, cs, rst, backlight, width, height, 
                                                   std::move(touch), 20'000'000, false, false, false, true);
        }

        static std::unique_ptr<Configuration> createForTN(
            gpio_num_t wr, gpio_num_t dc, gpio_num_t cs, gpio_num_t rst, gpio_num_t backlight,
            uint16_t width, uint16_t height, std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr) {
            return std::make_unique<Configuration>(wr, dc, cs, rst, backlight, width, height, 
                                                   std::move(touch), 20'000'000, false, false, false, false);
        }
    };

private:
    std::unique_ptr<Configuration> configuration;
    esp_lcd_panel_handle_t panelHandle = nullptr;
    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_i80_bus_handle_t i80Bus = nullptr;
    lv_display_t* displayHandle = nullptr;

public:
    explicit St7789I8080Display(std::unique_ptr<Configuration> inConfiguration) 
        : configuration(std::move(inConfiguration)) {
        assert(configuration != nullptr);
    }

    // Tactility DisplayDevice interface implementation
    std::string getName() const final { return "ST7789-I8080"; }
    std::string getDescription() const final { return "ST7789 display controller via Intel 8080 parallel interface"; }
    
    // Core lifecycle methods
    bool start() final;             // Initialize hardware (Step 1-5 from ST7789 reference)
    bool stop() final;              // Cleanup hardware resources
    
    // Touch device integration
    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override { 
        return configuration->touch; 
    }
    
    // Backlight control
    void setBacklightDuty(uint8_t backlightDuty) final {
        if (configuration->backlightDutyFunction != nullptr) {
            configuration->backlightDutyFunction(backlightDuty);
        } else {
            // Simple on/off backlight control (threshold at 50%)
            setBacklight(backlightDuty > 128);
        }
    }
    
    bool supportsBacklightDuty() const final { 
        return configuration->backlightDutyFunction != nullptr; 
    }
    
    // Gamma correction support
    void setGammaCurve(uint8_t index) final;    // 0-3: different gamma curves
    uint8_t getGammaCurveCount() const final { return 4; }
    
    // LVGL integration
    lv_display_t* _Nullable getLvglDisplay() const final { return displayHandle; }
    bool supportsLvgl() const override { return true; }
    bool startLvgl() override;      // Initialize LVGL integration
    bool stopLvgl() override;       // Stop LVGL integration

    // DisplayDevice pure virtual methods that need implementation
    bool supportsDisplayDriver() const final { return false; }  // This display doesn't use DisplayDriver pattern
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() final { return nullptr; }
    
    // Additional utility methods
    uint16_t getWidth() const { return configuration->horizontalResolution; }
    uint16_t getHeight() const { return configuration->verticalResolution; }
    uint32_t getPixelClockHz() const { return configuration->pixelClockHz; }
    uint8_t getBusWidth() const { return configuration->busWidth; }
    bool isColorInverted() const { return configuration->invertColor; }
    
    // Simple backlight control (on/off)
    void setBacklight(bool on);

    // Configuration access for debugging
    const Configuration* getConfiguration() const { return configuration.get(); }
};

// Factory function for creating display instances
std::shared_ptr<tt::hal::display::DisplayDevice> createI8080Display();