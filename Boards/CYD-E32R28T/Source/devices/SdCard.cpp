#include "SdCard.h"
#include <Tactility/hal/sdcard/SpiSdCardDevice.h>

using tt::hal::sdcard::SpiSdCardDevice;

/**
 * @brief Create and return a shared SD card device using the SPI driver.
 *
 * Constructs a SpiSdCardDevice configured for this board and returns it
 * upcast to std::shared_ptr<SdCardDevice>. The created configuration uses
 * GPIO_NUM_5 as CS, three GPIO_NUM_NC placeholders for other pins, a
 * MountBehaviour::AtBoot mount policy, a newly allocated mutex, an empty
 * extra-pin list, and SPI3_HOST as the SPI bus.
 *
 * @return std::shared_ptr<SdCardDevice> A shared pointer to the configured SD card device.
 */
std::shared_ptr<SdCardDevice> createSdCard() {
    auto configuration = std::make_unique<SpiSdCardDevice::Config>(
        GPIO_NUM_5,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        SdCardDevice::MountBehaviour::AtBoot,
        std::make_shared<tt::Mutex>(),
        std::vector<gpio_num_t>(),
        SPI3_HOST
    );

    return std::make_shared<SpiSdCardDevice>(
        std::move(configuration)
    );
}
