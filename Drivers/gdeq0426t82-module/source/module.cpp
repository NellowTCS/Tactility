// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern Driver gdeq0426t82_driver;

extern "C" {

static Driver* const gdeq0426t82_drivers[] = {
    &gdeq0426t82_driver,
    nullptr
};

Module gdeq0426t82_module = {
    .name = "gdeq0426t82",
    .drivers = gdeq0426t82_drivers
};

} // extern "C"
