#include "devices/Display.h"
#include "devices/SdCard.h"
#include "devices/Constants.h"

#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>
#include <PwmBacklight.h>

#define TAG "CYD2432S022C"

static bool initBoot() {
    // Initialize PWM backlight before creating display
    driver::pwmbacklight::init(DISPLAY_BL, 40000);
    driver::pwmbacklight::setBacklightDuty(255);
    return true;
}

static tt::hal::DeviceVector createDevices() {
    return {
        createDisplay(),
        createSdCard()
    };
}

extern const tt::hal::Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices,
    .i2c = {
        tt::hal::i2c::Configuration {
            .name = "Touch",
            .port = TOUCH_PORT,
            .initMode = tt::hal::i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = TOUCH_SDA,
                .scl_io_num = TOUCH_SCL,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master = {
                    .clk_speed = TOUCH_SPEED
                },
                .clk_flags = 0
            }
        }
    },
    .spi = {
        tt::hal::spi::Configuration {
            .device = SDCARD_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = SDCARD_MOSI,
                .miso_io_num = SDCARD_MISO,
                .sclk_io_num = SDCARD_SCLK,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC,
                .data7_io_num = GPIO_NUM_NC,
                .data_io_default_level = false,
                .max_transfer_sz = SDCARD_MAX_TRANSFER_SIZE,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = tt::hal::spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = nullptr
        }
    },
    .uart = {
        tt::hal::uart::Configuration {
            .name = "UART0",
            .port = UART_NUM_0,
            .rxPin = GPIO_NUM_3,
            .txPin = GPIO_NUM_1,
            .rtsPin = GPIO_NUM_NC,
            .ctsPin = GPIO_NUM_NC,
            .rxBufferSize = 1024,
            .txBufferSize = 1024,
            .config = {
                .baud_rate = 115200,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh = 0,
                .source_clk = UART_SCLK_DEFAULT,
                .flags = {
                    .allow_pd = 0,
                    .backup_before_sleep = 0,
                }
            }
        }
    }
};
