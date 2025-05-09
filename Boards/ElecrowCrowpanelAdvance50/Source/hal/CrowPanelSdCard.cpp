#include "CrowPanelSdCard.h"

#include <Tactility/hal/sdcard/SpiSdCardDevice.h>
#include <Tactility/lvgl/LvglSync.h>

#include <esp_vfs_fat.h>

using tt::hal::sdcard::SpiSdCardDevice;

std::shared_ptr<SdCardDevice> createSdCard() {
    auto* configuration = new SpiSdCardDevice::Config(
        // See https://github.com/Elecrow-RD/CrowPanel-Advance-HMI-ESP32-AI-Display/blob/master/5.0/factory_code/factory_code.ino
        GPIO_NUM_0, // It's actually not connected, but in the demo pin 0 is used
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        SdCardDevice::MountBehaviour::AtBoot
    );

    auto* sdcard = (SdCardDevice*) new SpiSdCardDevice(
        std::unique_ptr<SpiSdCardDevice::Config>(configuration)
    );

    return std::shared_ptr<SdCardDevice>(sdcard);
}
