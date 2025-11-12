/**
 * @file ssd1685.c
 * SSD1685 e-paper controller driver for GDEY029T71H panel
 * Ported to ESP-IDF v5 + LVGL v9
 * Based on SSD1680 driver by Aram Vartanyan
 */

#include "ssd1685.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "SSD1685"

#define BIT_SET(a,b)                    ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b)                  ((a) &= ~(1U<<(b)))

#define SSD1685_PIXEL                   (EPD_PANEL_WIDTH * EPD_PANEL_HEIGHT)
#define EPD_PANEL_NUMOF_COLUMS          EPD_PANEL_WIDTH
#define EPD_PANEL_NUMOF_ROWS_PER_PAGE   8
#define EPD_PANEL_NUMOF_PAGES           (EPD_PANEL_HEIGHT / EPD_PANEL_NUMOF_ROWS_PER_PAGE)

#define SSD1685_PANEL_FIRST_PAGE        0
#define SSD1685_PANEL_LAST_PAGE         (EPD_PANEL_NUMOF_PAGES - 1)
#define SSD1685_PANEL_FIRST_GATE        0
#define SSD1685_PANEL_LAST_GATE         (EPD_PANEL_NUMOF_COLUMS - 1)

#define SSD1685_PIXELS_PER_BYTE         8
#define EPD_PARTIAL_CNT                 5

static uint8_t ssd1685_scan_mode = SSD1685_DATA_ENTRY_XIYIY;
static uint8_t partial_counter = 0;

static uint8_t ssd1685_border_init[] = {SSD1685_BORDER_WAVEFORM_INIT};
static uint8_t ssd1685_border_part[] = {SSD1685_BORDER_WAVEFORM_PARTIAL};

static void ssd1685_update_display(bool isPartial);
static inline void ssd1685_set_window(uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye);
static inline void ssd1685_set_cursor(uint16_t sx, uint16_t ys);
static inline void ssd1685_waitbusy(int wait_ms);
static inline void ssd1685_hw_reset(void);
static inline void ssd1685_command_mode(void);
static inline void ssd1685_data_mode(void);
static inline void ssd1685_write_cmd(uint8_t cmd, uint8_t *data, size_t len);
static inline void ssd1685_send_cmd(uint8_t cmd);
static inline void ssd1685_send_data(uint8_t *data, uint16_t length);

void ssd1685_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    size_t linelen = EPD_PANEL_WIDTH / 8;
    uint8_t *buffer = (uint8_t *) color_map;
    uint16_t x_addr_counter = 0;
    uint16_t y_addr_counter = EPD_PANEL_HEIGHT - 1;

    ssd1685_init();

    if (!partial_counter) {
        ESP_LOGD(TAG, "Refreshing in FULL");
        
        ssd1685_send_cmd(SSD1685_CMD_WRITE_BLACK_VRAM);
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(buffer, linelen);
            buffer += SSD1685_COLUMNS;
        }

        buffer = (uint8_t *) color_map;
        
        ssd1685_send_cmd(SSD1685_CMD_WRITE_RED_VRAM);
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(buffer, linelen);
            buffer += SSD1685_COLUMNS;
        }

        ssd1685_update_display(false);
        partial_counter = EPD_PARTIAL_CNT;
    } else {
        ESP_LOGD(TAG, "Refreshing in PARTIAL");
        
        ssd1685_hw_reset();
        ssd1685_write_cmd(SSD1685_CMD_BORDER_WAVEFORM_CTRL, ssd1685_border_part, 1);
        ssd1685_set_window(area->x1, area->x2, area->y1, area->y2);
        ssd1685_set_cursor(x_addr_counter, y_addr_counter);

        ssd1685_send_cmd(SSD1685_CMD_WRITE_BLACK_VRAM);
        buffer = (uint8_t *) color_map;
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(buffer, linelen);
            buffer += SSD1685_COLUMNS;
        }

        ssd1685_update_display(true);
        partial_counter--;
    }

    ssd1685_deep_sleep();
    lv_disp_flush_ready(drv);
}

static inline void ssd1685_set_window(uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye)
{
    uint8_t tmp[4] = {0};

    tmp[0] = (sx + SSD1685_SOURCE_SHIFT) / 8;
    tmp[1] = (ex + SSD1685_SOURCE_SHIFT) / 8;

    ssd1685_write_cmd(SSD1685_CMD_SET_RAM_X_ADDR_START_END, tmp, 2);

    tmp[0] = ys & 0xFF;
    tmp[1] = (ys >> 8) & 0xFF;
    tmp[2] = ye & 0xFF;
    tmp[3] = (ye >> 8) & 0xFF;

    ssd1685_write_cmd(SSD1685_CMD_SET_RAM_Y_ADDR_START_END, tmp, 4);
}

static inline void ssd1685_set_cursor(uint16_t sx, uint16_t ys)
{
    uint8_t tmp[2] = {0};

    tmp[0] = (sx + SSD1685_SOURCE_SHIFT) / 8;
    ssd1685_write_cmd(SSD1685_CMD_SET_RAM_X_ADDR_COUNTER, tmp, 1);

    tmp[0] = ys & 0xFF;
    tmp[1] = (ys >> 8) & 0xFF;
    ssd1685_write_cmd(SSD1685_CMD_SET_RAM_Y_ADDR_COUNTER, tmp, 2);
}

static void ssd1685_update_display(bool isPartial)
{
    uint8_t tmp = 0;

    if (isPartial) {
        tmp = 0xDF;
    } else {
        tmp = 0xF7;
    }

    ssd1685_write_cmd(SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &tmp, 1);
    ssd1685_write_cmd(SSD1685_CMD_MASTER_ACTIVATION, NULL, 0);
    ssd1685_waitbusy(SSD1685_WAIT);
}

void ssd1685_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf,
                       lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                       lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = 0;
    uint8_t bit_index = 0;

#if defined(CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
    byte_index = x + ((y >> 3) * EPD_PANEL_HEIGHT);
    bit_index = y & 0x7;

    if (color.full) {
        BIT_SET(buf[byte_index], 7 - bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], 7 - bit_index);
    }
#elif defined(CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE)
    uint16_t mirrored_idx = (EPD_PANEL_HEIGHT - x) + ((y >> 3) * EPD_PANEL_HEIGHT);
    byte_index = x + ((y >> 3) * EPD_PANEL_HEIGHT);
    bit_index = y & 0x7;

    if (color.full == 0) {
        BIT_SET(buf[mirrored_idx - 1], 7 - bit_index);
    } else {
        BIT_CLEAR(buf[mirrored_idx - 1], 7 - bit_index);
    }
#else
#error "Unsupported orientation used"
#endif
}

void ssd1685_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->x1 = area->x1 & ~(0x7);
    area->x2 = area->x2 | (0x7);
}

void ssd1685_init(void)
{
    uint8_t tmp[3] = {0};

    gpio_pad_select_gpio(SSD1685_DC_PIN);
    gpio_set_direction(SSD1685_DC_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(SSD1685_BUSY_PIN);
    gpio_set_direction(SSD1685_BUSY_PIN, GPIO_MODE_INPUT);

#if SSD1685_USE_RST
    gpio_pad_select_gpio(SSD1685_RST_PIN);
    gpio_set_direction(SSD1685_RST_PIN, GPIO_MODE_OUTPUT);

    ssd1685_hw_reset();
#endif

    ssd1685_waitbusy(SSD1685_WAIT);

    ssd1685_write_cmd(SSD1685_CMD_SW_RESET, NULL, 0);
    ssd1685_waitbusy(SSD1685_WAIT);

    tmp[0] = (EPD_PANEL_HEIGHT - 1) & 0xFF;
    tmp[1] = ((EPD_PANEL_HEIGHT - 1) >> 8) & 0xFF;
    tmp[2] = 0x00;
    ssd1685_write_cmd(SSD1685_CMD_DRIVER_OUTPUT_CTRL, tmp, 3);

    ssd1685_write_cmd(SSD1685_CMD_DATA_ENTRY_MODE, &ssd1685_scan_mode, 1);

    ssd1685_set_window(0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);

    ssd1685_write_cmd(SSD1685_CMD_BORDER_WAVEFORM_CTRL, ssd1685_border_init, 1);

    tmp[0] = 0x00;
    tmp[1] = 0x80;
    ssd1685_write_cmd(SSD1685_CMD_DISPLAY_UPDATE_CTRL1, tmp, 2);

    tmp[0] = 0x80;
    ssd1685_write_cmd(SSD1685_CMD_TEMPERATURE_SENSOR, tmp, 1);

    ssd1685_set_cursor(0, EPD_PANEL_HEIGHT - 1);
    ssd1685_waitbusy(SSD1685_WAIT);
}

void ssd1685_deep_sleep(void)
{
    uint8_t data[] = {0x01};

    ssd1685_waitbusy(SSD1685_WAIT);
    ssd1685_write_cmd(SSD1685_CMD_DEEP_SLEEP_MODE, data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static inline void ssd1685_waitbusy(int wait_ms)
{
    int i = 0;

    vTaskDelay(pdMS_TO_TICKS(10));

    for (i = 0; i < (wait_ms * 10); i++) {
        if (gpio_get_level(SSD1685_BUSY_PIN) != SSD1685_BUSY_LEVEL) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "busy exceeded %dms", i * 10);
}

static inline void ssd1685_hw_reset(void)
{
#if SSD1685_USE_RST
    gpio_set_level(SSD1685_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(SSD1685_RESET_DELAY));
    gpio_set_level(SSD1685_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(SSD1685_RESET_DELAY));
#endif
}

static inline void ssd1685_command_mode(void)
{
    gpio_set_level(SSD1685_DC_PIN, 0);
}

static inline void ssd1685_data_mode(void)
{
    gpio_set_level(SSD1685_DC_PIN, 1);
}

static inline void ssd1685_write_cmd(uint8_t cmd, uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    ssd1685_command_mode();
    disp_spi_send_data(&cmd, 1);

    if (data != NULL) {
        ssd1685_data_mode();
        disp_spi_send_data(data, len);
    }
}

static inline void ssd1685_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    ssd1685_command_mode();
    disp_spi_send_data(&cmd, 1);
}

static inline void ssd1685_send_data(uint8_t *data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    ssd1685_data_mode();
    disp_spi_send_colors(data, length);
}
