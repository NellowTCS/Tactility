#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>

#include "HeltecV3.h"
#include "devices/Display.h"
#include "devices/Power.h"
#include "devices/Constants.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void i2c_scan(i2c_port_t port) {
    ESP_LOGI("I2C_SCAN", "Scanning I2C port %d for devices...", port);
    bool found_any = false;
    for (int addr = 1; addr < 127; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
            found_any = true;
        }
    }
    if (!found_any) {
        ESP_LOGW("I2C_SCAN", "No I2C devices found on port %d (try different address/pins/pull-ups)", port);
    } else {
        ESP_LOGI("I2C_SCAN", "I2C scan complete");
    }
}

static bool initBoot() {
    // Ensure the OLED module gets a proper reset pulse on boot.
    // Some Heltec variants require toggling the reset line to wake the display.
    if (HELTEC_LCD_PIN_RST != GPIO_NUM_NC) {
        gpio_set_direction(HELTEC_LCD_PIN_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(HELTEC_LCD_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10)); // hold low
        gpio_set_level(HELTEC_LCD_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // let it finish booting
    }

    // Run a quick I2C scan on the configured I2C port.
    // This helps quickly surface wiring/address issues for the OLED/touch devices.
    // Note: if the I2C driver hasn't been initialized yet by the HAL, this will
    // return no devices — that's okay. The log will indicate the result.
    i2c_scan(HELTEC_LCD_I2C_PORT);

    return true;
}

using namespace tt::hal;

static std::vector<std::shared_ptr<Device>> createDevices() {
    return {
        createPower(),
        createDisplay()
    };
}

extern const Configuration heltec_v3 = {
    .initBoot = initBoot,
    .createDevices = createDevices,
    .i2c = {
        tt::hal::i2c::Configuration {
            .name = "Touch",
            .port = HELTEC_LCD_I2C_PORT,
            .initMode = tt::hal::i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = HELTEC_LCD_PIN_SDA,
                .scl_io_num = HELTEC_LCD_PIN_SCL,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master = {
                    .clk_speed = HELTEC_LCD_I2C_SPEED
                },
                .clk_flags = 0
            }
        }
    },
    .spi {},
};
