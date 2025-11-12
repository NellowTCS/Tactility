/**
 * @file ssd1685.h
 * SSD1685 e-paper controller driver for GDEY029T71H panel
 * Ported to ESP-IDF v5 + LVGL v9
 * Based on SSD1680 driver by Aram Vartanyan
 */

#ifndef SSD1685_H
#define SSD1685_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "driver/gpio.h"

#define EPD_PANEL_WIDTH          168
#define EPD_PANEL_HEIGHT         384

#define SSD1685_COLUMNS          (EPD_PANEL_WIDTH / 8)

#define SSD1685_DC_PIN           GPIO_NUM_13
#define SSD1685_RST_PIN          GPIO_NUM_12
#define SSD1685_USE_RST          1
#define SSD1685_BUSY_PIN         GPIO_NUM_14
#define SSD1685_BUSY_LEVEL       1

/* SSD1685 commands */
#define SSD1685_CMD_DRIVER_OUTPUT_CTRL     0x01
#define SSD1685_CMD_BOOSTER_SOFT_START     0x0C
#define SSD1685_CMD_GATE_SCANNING_START    0x0F
#define SSD1685_CMD_DEEP_SLEEP_MODE        0x10
#define SSD1685_CMD_DATA_ENTRY_MODE        0x11
#define SSD1685_CMD_SW_RESET               0x12
#define SSD1685_CMD_TEMPERATURE_SENSOR     0x18
#define SSD1685_CMD_MASTER_ACTIVATION      0x20
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL1   0x21
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL2   0x22
#define SSD1685_CMD_WRITE_BLACK_VRAM       0x24
#define SSD1685_CMD_WRITE_RED_VRAM         0x26
#define SSD1685_CMD_READ_VRAM              0x25
#define SSD1685_CMD_VCOM_SENSE             0x28
#define SSD1685_CMD_VCOM_SENSE_DURATION    0x29
#define SSD1685_CMD_PROGRAM_VCOM_OTP       0x2A
#define SSD1685_CMD_VCOM_VOLTAGE           0x2C
#define SSD1685_CMD_PROGRAM_WS_OTP         0x30
#define SSD1685_CMD_UPDATE_LUT              0x32
#define SSD1685_CMD_PROGRAM_OTP_SELECTION  0x36
#define SSD1685_CMD_WRITE_DISPLAY_OPTION   0x37
#define SSD1685_CMD_BORDER_WAVEFORM_CTRL   0x3C
#define SSD1685_CMD_SET_RAM_X_ADDR_START_END  0x44
#define SSD1685_CMD_SET_RAM_Y_ADDR_START_END  0x45
#define SSD1685_CMD_SET_RAM_X_ADDR_COUNTER    0x4E
#define SSD1685_CMD_SET_RAM_Y_ADDR_COUNTER    0x4F

/* Data entry sequence modes */
#define SSD1685_DATA_ENTRY_XDYDX           0x00
#define SSD1685_DATA_ENTRY_XIYDX           0x01
#define SSD1685_DATA_ENTRY_XDYIX           0x02
#define SSD1685_DATA_ENTRY_XIYIX           0x03
#define SSD1685_DATA_ENTRY_XDYDY           0x04
#define SSD1685_DATA_ENTRY_XIYDY           0x05
#define SSD1685_DATA_ENTRY_XDYIY           0x06
#define SSD1685_DATA_ENTRY_XIYIY           0x07

/* Display update control modes */
#define SSD1685_CTRL2_ENABLE_CLK           0x80
#define SSD1685_CTRL2_ENABLE_ANALOG        0x40
#define SSD1685_CTRL2_TO_INITIAL           0x08
#define SSD1685_CTRL2_TO_PATTERN           0x04
#define SSD1685_CTRL2_DISABLE_ANALOG       0x02
#define SSD1685_CTRL2_DISABLE_CLK          0x01

/* Border waveform control */
#define SSD1685_BORDER_WAVEFORM_INIT       0x05
#define SSD1685_BORDER_WAVEFORM_PARTIAL    0x80

/* Sleep modes */
#define SSD1685_SLEEP_MODE_DSM             0x01
#define SSD1685_SLEEP_MODE_PON             0x00

/* Timing constants (ms) */
#define SSD1685_RESET_DELAY                20
#define SSD1685_BUSY_DELAY                 1
#define SSD1685_WAIT                       20

/* GDEY029T71H specific */
#define SSD1685_SOURCE_SHIFT               8

void ssd1685_init(void);
void ssd1685_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void ssd1685_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area);
void ssd1685_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, 
                       lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);
void ssd1685_deep_sleep(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SSD1680_REGS_H__ */
