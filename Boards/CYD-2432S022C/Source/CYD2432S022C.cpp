#include "CYD2432S022C.h"
#include "hal/YellowDisplay.h"
#include "hal/YellowDisplayConstants.h"
#include "hal/YellowSdCard.h"
#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>
#include <vector>
#include <esp_log.h>

#define TOUCH_I2C_SDA_GPIO_NUM GPIO_NUM_22
#define TOUCH_I2C_SCL_GPIO_NUM GPIO_NUM_21
#define TOUCH_I2C_CLK_SPEED 400000

bool initBoot() {
    return driver::pwmbacklight::init(TWODOTFOUR_LCD_PIN_BACKLIGHT);
}

static std::vector<tt::hal::i2c::Configuration> make_i2c_configurations() {
    std::vector<tt::hal::i2c::Configuration> configs;

    tt::hal::i2c::Configuration cfg1;
    cfg1.name = "First";
    cfg1.port = I2C_NUM_0;
    cfg1.initMode = tt::hal::i2c::InitMode::ByTactility;
    cfg1.isMutable = true;
    cfg1.config.mode = I2C_MODE_MASTER;
    cfg1.config.sda_io_num = TOUCH_I2C_SDA_GPIO_NUM;
    cfg1.config.scl_io_num = TOUCH_I2C_SCL_GPIO_NUM;
    cfg1.config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg1.config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg1.config.master.clk_speed = TOUCH_I2C_CLK_SPEED;
    cfg1.config.clk_flags = 0;
    configs.push_back(cfg1);

    tt::hal::i2c::Configuration cfg2;
    cfg2.name = "Second";
    cfg2.port = I2C_NUM_1;
    cfg2.initMode = tt::hal::i2c::InitMode::Disabled;
    cfg2.isMutable = true;
    cfg2.config.mode = I2C_MODE_MASTER;
    cfg2.config.sda_io_num = GPIO_NUM_NC;
    cfg2.config.scl_io_num = GPIO_NUM_NC;
    cfg2.config.sda_pullup_en = false;
    cfg2.config.scl_pullup_en = false;
    cfg2.config.master.clk_speed = 400000;
    cfg2.config.clk_flags = 0;
    configs.push_back(cfg2);

    return configs;
}

static std::vector<tt::hal::spi::Configuration> make_spi_configurations() {
    std::vector<tt::hal::spi::Configuration> configs;

    tt::hal::spi::Configuration cfg;
    cfg.mosi_io_num = 23;           // Adjusted to common Tactility naming
    cfg.miso_io_num = 19;
    cfg.sclk_io_num = 18;
    cfg.cs_pin = 5;
    cfg.clock_speed_hz = 1000000;
    configs.push_back(cfg);

    return configs;
}

const tt::hal::Configuration cyd_2432s022C_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay,
    .sdcard = createYellowSdCard(),
    .power = nullptr,
    .i2c = make_i2c_configurations(),
    .spi = make_spi_configurations()
};
