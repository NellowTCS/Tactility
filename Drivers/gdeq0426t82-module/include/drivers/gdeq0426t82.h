// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <tactility/drivers/gpio.h>

struct Gdeq0426t82Config {
    struct GpioPinSpec pin_dc;
    struct GpioPinSpec pin_reset;
    struct GpioPinSpec pin_busy;
    int clock_speed_hz;
};

#ifdef __cplusplus
}
#endif
