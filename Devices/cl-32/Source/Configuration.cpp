#include "devices/Display.h"
#include "devices/SdCard.h"
#include "devices/Keyboard.h"

#include <Tactility/hal/Configuration.h>
#include <Tactility/lvgl/LvglSync.h>
#include <driver/gpio.h>
#include <esp_log.h>

using namespace tt::hal;

static bool initBoot() {
    // Install GPIO ISR service (required for e-paper BUSY pin interrupt)
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("CL32", "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return false;
    }
    
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
    .createDevices = createDevices
};
