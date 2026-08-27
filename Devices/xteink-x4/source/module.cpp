// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver xteink_x4_power_driver;
extern Driver xteink_x4_power_supply_driver;
extern Driver gdeq0426t82_driver;

static Driver* const xteink_x4_drivers[] = {
    &xteink_x4_power_driver,
    &xteink_x4_power_supply_driver,
    &gdeq0426t82_driver,
    nullptr
};

Module xteink_x4_module = {
    .name = "xteink-x4",
    .drivers = xteink_x4_drivers
};

}
