#include "devices/Display.h"
#include "devices/SdCard.h"
#include "devices/Keyboard.h"

#include <Tactility/Log.h>
#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>

using namespace tt::hal;

static bool initBoot() {
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        TT_LOG_E("CL32", "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return false;
    }
    
    return true;
}

static DeviceVector createDevices() {
    auto tca = std::make_shared<Tca8418>(I2C_NUM_0);
    auto keyboard = std::make_shared<CL32Keyboard>(tca);

    // Add SPI device for display
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000,
        .input_delay_ns = 0,
        .spics_io_num = EPD_PIN_CS,
        .flags = 0,
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr,
    };

    spi_device_handle_t display_spi = nullptr;
    esp_err_t ret = spi_bus_add_device(EPD_SPI_HOST, &dev_cfg, &display_spi);
    if (ret != ESP_OK) {
        TT_LOG_E("CL32", "Failed to add display SPI device: %s", esp_err_to_name(ret));
        return {
            createSdCard(),
            tca,
            keyboard
        };
    }

    return {
        createDisplay(display_spi),
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
                .sda_pullup_en = true,
                .scl_pullup_en = true,
                .master = {
                    .clk_speed = 100000
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
            .lock = tt::lvgl::getSyncLock()
        }
    }
};
