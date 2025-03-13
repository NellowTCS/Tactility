#include "sdkconfig.h"
#include "CYD2432S022C.h"
#include "hal/YellowDisplay.h"
#include "hal/YellowDisplayConstants.h"
#include "hal/YellowSdCard.h"

#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>
#include <vector>

#define CYD_SPI_TRANSFER_SIZE_LIMIT (TWODOTFOUR_LCD_DRAW_BUFFER_SIZE * LV_COLOR_DEPTH / 8)

// LVGL Settings
#define LVGL_FONT_MONTSERRAT_14_ENABLED CONFIG_LV_FONT_MONTSERRAT_14
#define LVGL_FONT_MONTSERRAT_18_ENABLED CONFIG_LV_FONT_MONTSERRAT_18
#define LVGL_USE_USER_DATA_ENABLED CONFIG_LV_USE_USER_DATA

// I2C Configuration for Touchscreen (CST816S)
#define TOUCH_I2C_SDA_GPIO_NUM TOUCH_I2C_SDA_GPIO_NUM
#define TOUCH_I2C_SCL_GPIO_NUM TOUCH_I2C_SCL_GPIO_NUM
#define TOUCH_I2C_CLK_SPEED    TOUCH_I2C_CLK_SPEED

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

    tt::hal::spi::Configuration spi_cfg1;
    spi_cfg1.device = SPI2_HOST;
    spi_cfg1.dma = SPI_DMA_CH_AUTO;
    spi_cfg1.config.mosi_io_num = GPIO_NUM_13;
    spi_cfg1.config.miso_io_num = GPIO_NUM_NC;
    spi_cfg1.config.sclk_io_num = GPIO_NUM_14;
    spi_cfg1.config.quadwp_io_num = -1;
    spi_cfg1.config.quadhd_io_num = -1;
    spi_cfg1.config.data4_io_num = 0;
    spi_cfg1.config.data5_io_num = 0;
    spi_cfg1.config.data6_io_num = 0;
    spi_cfg1.config.data7_io_num = 0;
    spi_cfg1.config.data_io_default_level = false;
    spi_cfg1.config.max_transfer_sz = CYD_SPI_TRANSFER_SIZE_LIMIT;
    spi_cfg1.config.flags = 0;
    spi_cfg1.config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    spi_cfg1.config.intr_flags = 0;
    spi_cfg1.initMode = tt::hal::spi::InitMode::ByTactility;
    spi_cfg1.isMutable = false;
    spi_cfg1.lock = tt::lvgl::getSyncLock();
    configs.push_back(spi_cfg1);

    tt::hal::spi::Configuration spi_cfg2;
    spi_cfg2.device = SPI3_HOST;
    spi_cfg2.dma = SPI_DMA_CH_AUTO;
    spi_cfg2.config.mosi_io_num = GPIO_NUM_23;
    spi_cfg2.config.miso_io_num = GPIO_NUM_19;
    spi_cfg2.config.sclk_io_num = GPIO_NUM_18;
    spi_cfg2.config.quadwp_io_num = -1;
    spi_cfg2.config.quadhd_io_num = -1;
    spi_cfg2.config.data4_io_num = 0;
    spi_cfg2.config.data5_io_num = 0;
    spi_cfg2.config.data6_io_num = 0;
    spi_cfg2.config.data7_io_num = 0;
    spi_cfg2.config.data_io_default_level = false;
    spi_cfg2.config.max_transfer_sz = 8192;
    spi_cfg2.config.flags = 0;
    spi_cfg2.config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    spi_cfg2.config.intr_flags = 0;
    spi_cfg2.initMode = tt::hal::spi::InitMode::ByTactility;
    spi_cfg2.isMutable = false;
    spi_cfg2.lock = nullptr;
    configs.push_back(spi_cfg2);

    return configs;
}

const tt::hal::Configuration cyd_2432S022c_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay,
    .sdcard = createYellowSdCard(),
    .power = nullptr,
    .i2c = make_i2c_configurations(),
    .spi = make_spi_configurations()
};
