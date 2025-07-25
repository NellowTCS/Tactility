#pragma once

#include "Tactility/hal/power/PowerDevice.h"
#include <Bq27220.h>
#include <memory>

using tt::hal::power::PowerDevice;

class TpagerPower : public PowerDevice {
    std::shared_ptr<Bq27220> gauge;

public:

    TpagerPower(std::shared_ptr<Bq27220> bq) : gauge(std::move(bq)) {}
    ~TpagerPower();

    std::string getName() const final { return "T-LoRa Pager Power measument"; }
    std::string getDescription() const final { return "Power measurement interface via I2C fuel gauge"; }

    bool supportsMetric(MetricType type) const override;
    bool getMetric(MetricType type, MetricData& data) override;

private:
};

std::shared_ptr<PowerDevice> tpager_get_power();
