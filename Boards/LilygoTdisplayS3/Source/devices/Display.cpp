#include "TouchDisplay.h"

#include "freertos/FreeRTOS.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_dma_utils.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include <Gt911Touch.h>
#include <PwmBacklight.h>
#include <St7789Display.h>

#define TOUCH_LCD_SPI_HOST SPI2_HOST
#define TOUCH_LCD_PIN_CS GPIO_NUM_6
#define TOUCH_LCD_PIN_DC GPIO_NUM_7
#define TOUCH_LCD_PIN_RST GPIO_NUM_5
#define TOUCH_LCD_PIN_BL GPIO_NUM_38
#define TOUCH_LCD_HORIZONTAL_RESOLUTION 320
#define TOUCH_LCD_VERTICAL_RESOLUTION 170

static const char *TAG = "TouchDisplay";

static lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto configuration = std::make_unique<Gt911Touch::Configuration>(
        I2C_NUM_0,
        TOUCH_LCD_HORIZONTAL_RESOLUTION,
        TOUCH_LCD_VERTICAL_RESOLUTION,
        true,
        true,
        false
    );

    return std::make_shared<Gt911Touch>(std::move(configuration));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    ESP_LOGI(TAG, "Initializing Touch Display...");

    auto touch = createTouch();

    auto configuration = std::make_unique<St7789Display::Configuration>(
        TOUCH_LCD_SPI_HOST,
        TOUCH_LCD_PIN_CS,
        TOUCH_LCD_PIN_DC,
        TOUCH_LCD_HORIZONTAL_RESOLUTION,
        TOUCH_LCD_VERTICAL_RESOLUTION,
        touch,
        true,
        true,
        false,
        true
    );

    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    // Custom initialization commands
    auto display = std::make_shared<St7789Display>(std::move(configuration));
    auto panel_handle = display->getPanelHandle();
    auto io_handle = display->getIoHandle();

    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
        esp_lcd_panel_io_tx_param(io_handle, lcd_st7789v[i].addr, lcd_st7789v[i].param, lcd_st7789v[i].len & 0x7f);
        if (lcd_st7789v[i].len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }

    ESP_LOGI(TAG, "Touch Display initialized successfully.");
    return std::reinterpret_pointer_cast<tt::hal::display::DisplayDevice>(display);
}