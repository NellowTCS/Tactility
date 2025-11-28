#include "SdCard.h"
#include <Tactility/hal/sdcard/SpiSdCardDevice.h>

using tt::hal::sdcard::SpiSdCardDevice;

std::shared_ptr<SdCardDevice> createSdCard() {
    auto config = std::make_unique<SpiSdCardDevice::Config>(
        SDCARD_PIN_CS,
        GPIO_NUM_NC, // Card detect
        GPIO_NUM_NC, // Write protect
        GPIO_NUM_NC, // Power control
        SdCardDevice::MountBehaviour::AtBoot,
        std::make_shared<tt::Mutex>(tt::Mutex::Type::Recursive),
        std::vector<gpio_num_t>(), // No additional pins
        SDCARD_SPI_HOST
    );

    return std::make_shared<SpiSdCardDevice>(std::move(config));
}
