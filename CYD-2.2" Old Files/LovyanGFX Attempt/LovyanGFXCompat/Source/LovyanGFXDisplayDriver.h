#pragma once

#include <Tactility/hal/display/DisplayDriver.h>
#include <Tactility/MutexLock.h>
#include "lgfx/v1/panel/Panel_LCD.hpp"
#include <lgfx/v1/misc/pixelcopy.hpp>
#include <memory>

class LovyanGFXDisplayDriver : public tt::hal::display::DisplayDriver {
private:
    std::shared_ptr<lgfx::Panel_LCD> panel;
    std::shared_ptr<tt::Lock> lock;
    tt::hal::display::ColorFormat colorFormat;

public:
    explicit LovyanGFXDisplayDriver(std::shared_ptr<lgfx::Panel_LCD> panel) 
        : panel(panel)
        , lock(std::make_shared<tt::MutexLock>())
        , colorFormat(tt::hal::display::ColorFormat::RGB565) {} // LovyanGFX uses RGB565 by default

    tt::hal::display::ColorFormat getColorFormat() const override {
        return colorFormat;
    }

    uint16_t getPixelWidth() const override {
        return panel->width();
    }

    uint16_t getPixelHeight() const override {
        return panel->height();
    }

    bool drawBitmap(int xStart, int yStart, int xEnd, int yEnd, const void* pixelData) override {
        // Setup the pixel copy parameters
        lgfx::pixelcopy_t pixelcopy;
        pixelcopy.src_data = pixelData;
        pixelcopy.src_width = xEnd - xStart + 1;
        pixelcopy.src_height = yEnd - yStart + 1;
        pixelcopy.src_bits = 16; // RGB565
        pixelcopy.src_x = 0;
        pixelcopy.src_y = 0;
        
        // Write the image to the panel
        panel->writeImage(
            xStart, yStart, 
            pixelcopy.src_width, 
            pixelcopy.src_height,
            &pixelcopy,
            true // Use DMA for faster transfers
        );
        return true;
    }

    std::shared_ptr<tt::Lock> getLock() const override {
        return lock;
    }

    // Additional LovyanGFX specific methods
    bool setRotation(uint8_t rotation) {
        panel->setRotation(rotation);
        return true;
    }

    bool setInvert(bool invert) {
        panel->setInvert(invert);
        return true;
    }
};