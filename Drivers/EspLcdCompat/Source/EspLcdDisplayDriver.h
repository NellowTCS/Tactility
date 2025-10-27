#pragma once

#include <Tactility/Mutex.h>
#include <Tactility/hal/display/DisplayDriver.h>

#include <esp_lcd_panel_ops.h>

class EspLcdDisplayDriver : public tt::hal::display::DisplayDriver {

    esp_lcd_panel_handle_t panelHandle;
    std::shared_ptr<tt::Lock> lock;
    uint16_t hRes;
    uint16_t vRes;
    tt::hal::display::ColorFormat colorFormat;
    int columnOffset = 0; // column offset applied to x coordinates

public:

    EspLcdDisplayDriver(
        esp_lcd_panel_handle_t panelHandle,
        std::shared_ptr<tt::Lock> lock,
        uint16_t hRes,
        uint16_t vRes,
        tt::hal::display::ColorFormat colorFormat,
        int columnOffset = 0
    ) : panelHandle(panelHandle), lock(lock), hRes(hRes), vRes(vRes), colorFormat(colorFormat), columnOffset(columnOffset) {}

    tt::hal::display::ColorFormat getColorFormat() const override {
        return colorFormat;
    }

    bool drawBitmap(int xStart, int yStart, int xEnd, int yEnd, const void* pixelData) override {
        // Apply column offset to both start and end X coordinates before issuing the draw
        int xs = xStart + columnOffset;
        int xe = xEnd + columnOffset;
        bool result = esp_lcd_panel_draw_bitmap(panelHandle, xs, yStart, xe, yEnd, pixelData) == ESP_OK;
        return result;
    }

    uint16_t getPixelWidth() const override { return hRes; }

    uint16_t getPixelHeight() const override { return vRes; }

    std::shared_ptr<tt::Lock> getLock() const override { return lock; }
};
