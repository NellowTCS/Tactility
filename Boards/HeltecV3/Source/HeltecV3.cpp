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

static void enableOledPower() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << HELTEC_LCD_PIN_POWER);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // The board has an external pull-up
    gpio_config(&io_conf);
    gpio_set_level(HELTEC_LCD_PIN_POWER, 0); // Set LOW to enable power

    vTaskDelay(pdMS_TO_TICKS(100)); // Add a small delay for power to stabilize
    ESP_LOGI("OLED_POWER", "OLED Vext power enabled on GPIO %d", HELTEC_LCD_PIN_POWER);
}

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
    // Enable power to the OLED before doing anything else
    enableOledPower();

    // Handle the OLED reset sequence
    if (HELTEC_LCD_PIN_RST != GPIO_NUM_NC) {
        gpio_set_direction(HELTEC_LCD_PIN_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(HELTEC_LCD_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10)); // hold low
        gpio_set_level(HELTEC_LCD_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // let it finish booting
    }

    // Run the I2C scan to see if we can find the display
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
