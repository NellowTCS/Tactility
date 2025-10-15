#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>

#include "HeltecV3.h"
#include "devices/Display.h"
#include "devices/Power.h"
#include "devices/Constants.h"

static bool initBoot() {
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
