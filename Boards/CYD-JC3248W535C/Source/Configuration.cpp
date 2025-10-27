#include "devices/Display.h"
#include "devices/SdCard.h"

#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>

using namespace tt::hal;

#define CYD_SPI_TRANSFER_SIZE_LIMIT ((320 * (480 / 10)) * LV_COLOR_DEPTH / 8)

static DeviceVector createDevices() {
    return {
        createDisplay(),
        createSdCard()
    };
}

static bool initBoot() {
    return driver::pwmbacklight::init(GPIO_NUM_1);
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices,
    .i2c = {
        //Touch
        i2c::Configuration {
            .name = "Internal",
            .port = I2C_NUM_0,
            .initMode = i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = GPIO_NUM_4,
                .scl_io_num = GPIO_NUM_8,
                .sda_pullup_en = true,
                .scl_pullup_en = true,
                .master = {
                    .clk_speed = 400000
                },
                .clk_flags = 0
            }
        },
        //some header
        i2c::Configuration {
            .name = "External",
            .port = I2C_NUM_1,
            .initMode = i2c::InitMode::Disabled,
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
    .spi {
        //Display
        spi::Configuration {
            .device = SPI2_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .data0_io_num = GPIO_NUM_21,
                .data1_io_num = GPIO_NUM_48,
                .sclk_io_num = GPIO_NUM_47,
                .data2_io_num = GPIO_NUM_40,
                .data3_io_num = GPIO_NUM_39,
                .data4_io_num = -1,
                .data5_io_num = -1,
                .data6_io_num = -1,
                .data7_io_num = -1,
                .data_io_default_level = false,
                .max_transfer_sz = CYD_SPI_TRANSFER_SIZE_LIMIT,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = tt::lvgl::getSyncLock()
        },
        //SD Card
        spi::Configuration {
            .device = SPI3_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_11,
                .miso_io_num = GPIO_NUM_13,
                .sclk_io_num = GPIO_NUM_12,
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
            .initMode = spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = nullptr
        },
    }
};
