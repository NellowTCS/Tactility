#pragma once

#include <EspLcdDisplay.h>
#include <Tactility/hal/display/DisplayDevice.h>
#include <Tactility/hal/spi/Spi.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>
#include <esp_lcd_panel_ssd1681.h>

class Ssd1681Display final : public EspLcdDisplay {

public:

    class Configuration {

    public:

        Configuration(
            spi_host_device_t spiHost,
            gpio_num_t csPin,
            gpio_num_t dcPin,
            gpio_num_t resetPin,
            gpio_num_t busyPin,
            unsigned int width,
            unsigned int height,
            uint8_t rotation = 0,
            std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr
        ) : spiHost(spiHost),
            csPin(csPin),
            dcPin(dcPin),
            resetPin(resetPin),
            busyPin(busyPin),
            width(width),
            height(height),
            rotation(rotation),
            touch(std::move(touch)),
            bufferSize(0),
            gapX(0),
            gapY(0)
        {
            vendorConfig.busy_gpio_num = busyPin;
            vendorConfig.non_copy_mode = false;
        }

        spi_host_device_t spiHost;
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t resetPin;
        gpio_num_t busyPin;
        unsigned int width;
        unsigned int height;
        uint8_t rotation;
        std::shared_ptr<tt::hal::touch::TouchDevice> touch;
        uint32_t bufferSize;
        int gapX;
        int gapY;
        esp_lcd_ssd1681_config_t vendorConfig;
    };

private:

    std::unique_ptr<Configuration> configuration;

    bool createIoHandle(esp_lcd_panel_io_handle_t& ioHandle) override;

    bool createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) override;

    lvgl_port_display_cfg_t getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) override;

public:

    explicit Ssd1681Display(std::unique_ptr<Configuration> inConfiguration) :
        EspLcdDisplay(tt::hal::spi::getLock(inConfiguration->spiHost)),
        configuration(std::move(inConfiguration))
    {
        assert(configuration != nullptr);
        if (configuration->bufferSize == 0) {
            configuration->bufferSize = configuration->width * configuration->height;
        }
    }

    std::string getName() const override { return "SSD1681"; }

    std::string getDescription() const override { return "SSD1681/SSD1685 e-paper display"; }

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override { return configuration->touch; }

    void setBacklightDuty(uint8_t backlightDuty) override {
        // E-paper does not have backlight control
    }

    bool supportsBacklightDuty() const override { return false; }

    void setGammaCurve(uint8_t index) override {
        // E-paper does not support gamma curves
    }

    uint8_t getGammaCurveCount() const override { return 0; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();
