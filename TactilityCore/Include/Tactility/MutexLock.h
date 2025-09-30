#pragma once

#include "Lock.h"
#include "RtosCompat.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace tt {

class MutexLock : public Lock {
public:
    MutexLock() {
        mutex = xSemaphoreCreateMutex();
    }

    ~MutexLock() override {
        if (mutex) {
            vSemaphoreDelete(mutex);
        }
    }

    bool lock(TickType_t timeout) const override {
        return xSemaphoreTake(mutex, timeout) == pdTRUE;
    }

    bool unlock() const override {
        return xSemaphoreGive(mutex) == pdTRUE;
    }

private:
    SemaphoreHandle_t mutex;
};

} // namespace tt