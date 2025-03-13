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
#define TOUCH_I2C_SDA_GPIO_NUM CONFIG_CST816S_I2C_CONFIG_SDA_IO_NUM
#define TOUCH_I2C_SCL_GPIO_NUM CONFIG_CST816S_I2C_CONFIG_SCL_IO_NUM
#define TOUCH_I2C_CLK_SPEED    CONFIG_CST816S_I2C_CONFIG_MASTER_CLK_SPEED

bool initBoot() {
    return driver::pwmbacklight::init(TWODOTFOUR_LCD_PIN_BACKLIGHT);
}

const tt::hal::Configuration cyd_2432S022c_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay,
    .sdcard = createYellowSdCard(),
    .power = nullptr,
    .i2c = std::vector<tt::hal::i2c::Configuration>{
        tt::hal::i2c::Configuration {
            .name = "First",
            .port = I2C_NUM_0,
            .initMode = tt::hal::i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
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
        },
        tt::hal::i2c::Configuration {
            .name = "Second",
            .port = I2C_NUM_1,
            .initMode = tt::hal::i2c::InitMode::Disabled,
            .isMutable = true,
            .config = (i2c_config_t) {
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
        }
    },
    .spi = std::vector<tt::hal::spi::Configuration>{
        tt::hal::spi::Configuration {
            .device = SPI2_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_13,
                .miso_io_num = GPIO_NUM_NC,
                .sclk_io_num = GPIO_NUM_14,
                .quadwp_io_num = -1, 
                .quadhd_io_num = -1,
                .data4_io_num = 0,
                .data5_io_num = 0,
                .data6_io_num = 0,
                .data7_io_num = 0,
                .data_io_default_level = false,
                .max_transfer_sz = CYD_SPI_TRANSFER_SIZE_LIMIT,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = tt::hal::spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = tt::lvgl::getSyncLock()
        },
        tt::hal::spi::Configuration {
            .device = SPI3_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_23,
                .miso_io_num = GPIO_NUM_19,
                .sclk_io_num = GPIO_NUM_18,
                .quadwp_io_num = -1, 
                .quadhd_io_num = -1, 
                .data4_io_num = 0,
                .data5_io_num = 0,
                .data6_io_num = 0,
                .data7_io_num = 0,
                .data_io_default_level = false,
                .max_transfer_sz = 8192,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = tt::hal::spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = nullptr
        }
    }
};
