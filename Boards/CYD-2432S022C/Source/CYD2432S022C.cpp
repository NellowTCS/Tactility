#include "CYD2432S022C.h"
#include "hal/YellowDisplay.h"
#include "hal/YellowDisplayConstants.h"
#include "hal/YellowSdCard.h"
#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>
#include <vector>
#include <esp_log.h>
#include <driver/spi_master.h> // For SPI_HOST definitions

#define TOUCH_I2C_SDA_GPIO_NUM GPIO_NUM_22
#define TOUCH_I2C_SCL_GPIO_NUM GPIO_NUM_21
#define TOUCH_I2C_CLK_SPEED 400000

bool initBoot() {
    return driver::pwmbacklight::init(TWODOTFOUR_LCD_PIN_BACKLIGHT);
}

static std::vector<tt::hal::i2c::Configuration> make_i2c_configurations() {
    std::vector<tt::hal::i2c::Configuration> configs;

    tt::hal::i2c::Configuration cfg1 = {
        .name = "First",
        .port = I2C_NUM_0,
        .initMode = tt::hal::i2c::InitMode::ByTactility,
        .isMutable = true,
        .config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = TOUCH_I2C_SDA_GPIO_NUM,
            .scl_io_num = TOUCH_I2C_SCL_GPIO_NUM,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = TOUCH_I2C_CLK_SPEED
            },
            .clk_flags = 0
        }
    };
    configs.push_back(cfg1);

    tt::hal::i2c::Configuration cfg2 = {
        .name = "Second",
        .port = I2C_NUM_1,
        .initMode = tt::hal::i2c::InitMode::Disabled,
        .isMutable = true,
        .config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_NC,
            .scl_io_num = GPIO_NUM_NC,
            .sda_pullup_en = false,
            .scl_pullup_en = false,
            .master = {
                .clk_speed = 400000
            },
            .clk_flags = 0
        }
    };
    configs.push_back(cfg2);

    return configs;
}

static std::vector<tt::hal::spi::Configuration> make_spi_configurations() {
    std::vector<tt::hal::spi::Configuration> configs;

    // SPI config for SD card (no display SPI needed for i80 ST7789)
    tt::hal::spi::Configuration cfg = {
        .device = SPI3_HOST, // SD card typically uses SPI3 on ESP32
        .dma = SPI_DMA_CH_AUTO,
        .config = {
            .mosi_io_num = GPIO_NUM_23,
            .miso_io_num = GPIO_NUM_19,
            .sclk_io_num = GPIO_NUM_18,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .data4_io_num = GPIO_NUM_0,
            .data5_io_num = GPIO_NUM_0,
            .data6_io_num = GPIO_NUM_0,
            .data7_io_num = GPIO_NUM_0,
            .data_io_default_level = false,
            .max_transfer_sz = 8192, // Reasonable for SD card
            .flags = 0,
            .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags = 0
        },
        .initMode = tt::hal::spi::InitMode::ByTactility,
        .isMutable = false,
        .lock = nullptr // SD card doesn’t need LVGL lock
    };
    configs.push_back(cfg);

    return configs;
}

const tt::hal::Configuration cyd_2432s022C_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay, // ST7789 i80 from YellowDisplay.cpp
    .sdcard = createYellowSdCard(),
    .power = nullptr,
    .i2c = make_i2c_configurations(),
    .spi = make_spi_configurations()
};
