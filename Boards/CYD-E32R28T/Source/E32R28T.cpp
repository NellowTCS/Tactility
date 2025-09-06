#include "E32R28T.h"
#include "devices/SdCard.h"
#include "devices/Display.h"
#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>

#define CYD_SPI_TRANSFER_SIZE_LIMIT (240 * 320 / 4 * 2)

/**
 * @brief Initialize board-specific boot peripherals.
 *
 * Initializes the PWM backlight driver for the board using the CYD_BACKLIGHT_PIN.
 *
 * @return true if the backlight driver initialized successfully; false on failure.
 */
static bool initBoot() {
    return driver::pwmbacklight::init(CYD_BACKLIGHT_PIN);
}

/**
 * @brief Construct the board's runtime device list.
 *
 * Returns a vector containing the display device and the SD card device
 * used by the CYD-E32R28T board.
 *
 * @return tt::hal::DeviceVector Vector with the display device first and the SD card device second.
 */
static tt::hal::DeviceVector createDevices() {
    return {
        createDisplay(),
        createSdCard()
    };
}

const tt::hal::Configuration cyd_e32r28t_config = {
    .initBoot = initBoot,
    .createDevices = createDevices,
    .i2c = {},
    .spi = {
        tt::hal::spi::Configuration {
            .device = SPI2_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_13,
                .miso_io_num = GPIO_NUM_12,
                .sclk_io_num = GPIO_NUM_14,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC,
                .data7_io_num = GPIO_NUM_NC,
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
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC,
                .data7_io_num = GPIO_NUM_NC,
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

    }
};
