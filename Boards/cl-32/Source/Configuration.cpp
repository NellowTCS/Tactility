#include "devices/Display.h"
#include "devices/SdCard.h"
#include "devices/Keyboard.h"

#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>
#include <driver/gpio.h>

using namespace tt::hal;

static bool initBoot() {
    // E-paper displays don't have backlights, but we might want to init other things here
    // For now, just return true
    return true;
}

static DeviceVector createDevices() {
    // create TCA wrapper and keyboard for CL-32
    auto tca = std::make_shared<Tca8418>(I2C_NUM_0);
    auto keyboard = std::make_shared<CL32Keyboard>(tca);

    return {
        createDisplay(),
        createSdCard(),
        tca,
        keyboard
    };
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices,
    .i2c = {
        i2c::Configuration {
            .name = "Keyboard",
            .port = I2C_NUM_0,
            .initMode = i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = GPIO_NUM_1,
                .scl_io_num = GPIO_NUM_2,
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
        // E-Paper Display
        spi::Configuration {
            .device = SPI2_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_10,
                .miso_io_num = GPIO_NUM_11,
                .sclk_io_num = GPIO_NUM_9,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC,
                .data7_io_num = GPIO_NUM_NC,
                .data_io_default_level = false,
                .max_transfer_sz = LCD_SPI_TRANSFER_SIZE_LIMIT,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = tt::lvgl::getSyncLock() // LVGL owns the lock
        }
    }
};
