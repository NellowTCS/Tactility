/**
 * @file esp_lcd_ssd1685.c
 * @brief ESP LCD driver implementation for SSD1685 e-paper display
 * 
 * Display: GDEY029T71H (168x384, monochrome)
 * Controller: SSD1685
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_ssd1685.h"

static const char *TAG = "lcd_ssd1685";

// Display specifications for GDEY029T71H
#define SSD1685_WIDTH           168
#define SSD1685_HEIGHT          384
#define SSD1685_SOURCE_SHIFT    8    // S8..S175 connected to display

// SSD1685 Commands
#define SSD1685_CMD_DRIVER_OUTPUT           0x01
#define SSD1685_CMD_GATE_DRIVING_VOLTAGE    0x03
#define SSD1685_CMD_SOURCE_DRIVING_VOLTAGE  0x04
#define SSD1685_CMD_DEEP_SLEEP              0x10
#define SSD1685_CMD_DATA_ENTRY_MODE         0x11
#define SSD1685_CMD_SW_RESET                0x12
#define SSD1685_CMD_TEMP_SENSOR_SELECT      0x18
#define SSD1685_CMD_TEMP_SENSOR_WRITE       0x1A
#define SSD1685_CMD_MASTER_ACTIVATE         0x20
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL1    0x21
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL2    0x22
#define SSD1685_CMD_WRITE_RAM_BW            0x24
#define SSD1685_CMD_WRITE_RAM_RED           0x26
#define SSD1685_CMD_VCOM_SENSE              0x28
#define SSD1685_CMD_VCOM_DURATION           0x29
#define SSD1685_CMD_WRITE_VCOM              0x2C
#define SSD1685_CMD_WRITE_LUT               0x32
#define SSD1685_CMD_BORDER_WAVEFORM         0x3C
#define SSD1685_CMD_SET_RAM_X_START_END     0x44
#define SSD1685_CMD_SET_RAM_Y_START_END     0x45
#define SSD1685_CMD_SET_RAM_X_COUNTER       0x4E
#define SSD1685_CMD_SET_RAM_Y_COUNTER       0x4F
#define SSD1685_CMD_NOP                     0x7F

// Data Entry Modes
#define DATA_ENTRY_X_INC_Y_INC              0x03
#define DATA_ENTRY_X_DEC_Y_DEC              0x00

// Border Waveform
#define BORDER_WAVEFORM_LUT                 0x05
#define BORDER_WAVEFORM_PARTIAL             0x80

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio;
    int busy_gpio;
    bool reset_level;
    
    uint16_t width;
    uint16_t height;
    uint8_t *framebuffer;
    uint8_t *previous_buffer;
    
    ssd1685_refresh_mode_t refresh_mode;
    uint8_t partial_counter;
    bool first_update;
    
    bool swap_axes;
    bool mirror_x;
    bool mirror_y;
} ssd1685_panel_t;

static esp_err_t panel_ssd1685_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, 
                                            int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1685_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1685_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1685_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1685_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1685_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

static void ssd1685_wait_busy(ssd1685_panel_t *ssd1685);
static esp_err_t ssd1685_write_cmd(ssd1685_panel_t *ssd1685, uint8_t cmd, const uint8_t *data, size_t len);
static esp_err_t ssd1685_set_window(ssd1685_panel_t *ssd1685, uint16_t x_start, uint16_t y_start, 
                                        uint16_t x_end, uint16_t y_end);
static esp_err_t ssd1685_set_cursor(ssd1685_panel_t *ssd1685, uint16_t x, uint16_t y);
static esp_err_t ssd1685_update_display(ssd1685_panel_t *ssd1685);

esp_err_t esp_lcd_new_panel_ssd1685(const esp_lcd_panel_io_handle_t io,
                                     const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     const esp_lcd_ssd1685_config_t *ssd1685_config,
                                     esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ssd1685_panel_t *ssd1685 = NULL;

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    
    ssd1685 = calloc(1, sizeof(ssd1685_panel_t));
    ESP_GOTO_ON_FALSE(ssd1685, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1685 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST failed");
    }

    if (ssd1685_config && ssd1685_config->busy_gpio >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << ssd1685_config->busy_gpio,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for BUSY failed");
    }

    ssd1685->io = io;
    ssd1685->reset_gpio = panel_dev_config->reset_gpio_num;
    ssd1685->reset_level = panel_dev_config->flags.reset_active_high;
    ssd1685->width = SSD1685_WIDTH;
    ssd1685->height = SSD1685_HEIGHT;
    ssd1685->first_update = true;
    ssd1685->partial_counter = 0;

    if (ssd1685_config) {
        ssd1685->busy_gpio = ssd1685_config->busy_gpio;
        ssd1685->refresh_mode = ssd1685_config->refresh_mode;
        ssd1685->swap_axes = ssd1685_config->swap_axes;
        ssd1685->mirror_x = ssd1685_config->mirror_x;
        ssd1685->mirror_y = ssd1685_config->mirror_y;
    } else {
        ssd1685->busy_gpio = -1;
        ssd1685->refresh_mode = SSD1685_REFRESH_FULL;
    }

    // Allocate framebuffers
    size_t fb_size = (SSD1685_WIDTH * SSD1685_HEIGHT) / 8;
    ssd1685->framebuffer = heap_caps_calloc(1, fb_size, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(ssd1685->framebuffer, ESP_ERR_NO_MEM, err, TAG, "no mem for framebuffer");
    
    ssd1685->previous_buffer = heap_caps_calloc(1, fb_size, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(ssd1685->previous_buffer, ESP_ERR_NO_MEM, err, TAG, "no mem for previous buffer");

    // Initialize all white
    memset(ssd1685->framebuffer, 0xFF, fb_size);
    memset(ssd1685->previous_buffer, 0xFF, fb_size);

    ssd1685->base.del = panel_ssd1685_del;
    ssd1685->base.reset = panel_ssd1685_reset;
    ssd1685->base.init = panel_ssd1685_init;
    ssd1685->base.draw_bitmap = panel_ssd1685_draw_bitmap;
    ssd1685->base.invert_color = panel_ssd1685_invert_color;
    ssd1685->base.set_gap = panel_ssd1685_set_gap;
    ssd1685->base.mirror = panel_ssd1685_mirror;
    ssd1685->base.swap_xy = panel_ssd1685_swap_xy;
    ssd1685->base.disp_on_off = panel_ssd1685_disp_on_off;

    *ret_panel = &(ssd1685->base);
    ESP_LOGD(TAG, "new ssd1685 panel @%p", ssd1685);

    return ESP_OK;

err:
    if (ssd1685) {
        if (ssd1685->framebuffer) {
            free(ssd1685->framebuffer);
        }
        if (ssd1685->previous_buffer) {
            free(ssd1685->previous_buffer);
        }
        free(ssd1685);
    }
    return ret;
}

static esp_err_t panel_ssd1685_del(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);

    if (ssd1685->reset_gpio >= 0) {
        gpio_reset_pin(ssd1685->reset_gpio);
    }
    if (ssd1685->busy_gpio >= 0) {
        gpio_reset_pin(ssd1685->busy_gpio);
    }

    free(ssd1685->framebuffer);
    free(ssd1685->previous_buffer);
    free(ssd1685);
    ESP_LOGD(TAG, "del ssd1685 panel @%p", ssd1685);
    return ESP_OK;
}

static esp_err_t panel_ssd1685_reset(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    if (ssd1685->reset_gpio >= 0) {
        gpio_set_level(ssd1685->reset_gpio, ssd1685->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ssd1685->reset_gpio, !ssd1685->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Software reset
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SW_RESET, NULL, 0), TAG, "send command failed");
    ssd1685_wait_busy(ssd1685);

    return ESP_OK;
}

static esp_err_t panel_ssd1685_init(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_RETURN_ON_ERROR(panel_ssd1685_reset(panel), TAG, "reset failed");

    // Driver output control
    uint8_t data[3];
    data[0] = (SSD1685_HEIGHT - 1) & 0xFF;
    data[1] = (SSD1685_HEIGHT - 1) >> 8;
    data[2] = 0x00; // GD=0, SM=0, TB=0
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DRIVER_OUTPUT, data, 3), TAG, "send command failed");

    // Data entry mode - increment X, increment Y
    data[0] = DATA_ENTRY_X_INC_Y_INC;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DATA_ENTRY_MODE, data, 1), TAG, "send command failed");

    // Border waveform control
    data[0] = BORDER_WAVEFORM_LUT;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_BORDER_WAVEFORM, data, 1), TAG, "send command failed");

    // Temperature sensor - internal
    data[0] = 0x80;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_TEMP_SENSOR_SELECT, data, 1), TAG, "send command failed");

    // Display update control 1
    data[0] = 0x00; // Normal RAM content
    data[1] = 0x80; // Source output S8-S175 (important for GDEY029T71H!)
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, data, 2), TAG, "send command failed");

    // Set full window
    ESP_RETURN_ON_ERROR(ssd1685_set_window(ssd1685, 0, 0, SSD1685_WIDTH - 1, SSD1685_HEIGHT - 1), TAG, "set window failed");
    ESP_RETURN_ON_ERROR(ssd1685_set_cursor(ssd1685, 0, 0), TAG, "set cursor failed");

    ssd1685_wait_busy(ssd1685);

    ESP_LOGI(TAG, "SSD1685 panel initialized");
    return ESP_OK;
}

static esp_err_t panel_ssd1685_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
                                            int x_end, int y_end, const void *color_data)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "color_data is NULL");

    // Copy to framebuffer
    const uint8_t *data = (const uint8_t *)color_data;
    size_t fb_size = (SSD1685_WIDTH * SSD1685_HEIGHT) / 8;
    memcpy(ssd1685->framebuffer, data, fb_size);

    // Determine refresh mode
    bool use_partial = false;
    if (!ssd1685->first_update && ssd1685->refresh_mode == SSD1685_REFRESH_PARTIAL) {
        use_partial = true;
    }

    if (use_partial && ssd1685->partial_counter == 0) {
        // Time for a full refresh to clear ghosting
        use_partial = false;
        ssd1685->partial_counter = 5; // Do 5 partial updates before next full
    }

    ESP_RETURN_ON_ERROR(ssd1685_set_window(ssd1685, 0, 0, SSD1685_WIDTH - 1, SSD1685_HEIGHT - 1), TAG, "set window failed");
    ESP_RETURN_ON_ERROR(ssd1685_set_cursor(ssd1685, 0, 0), TAG, "set cursor failed");

    if (use_partial) {
        // Partial update: write previous to RED RAM, current to BW RAM
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_RED,
                                                        ssd1685->previous_buffer, fb_size), TAG, "send command failed");
        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_BW,
                                                        ssd1685->framebuffer, fb_size), TAG, "send command failed");
        
        ssd1685->partial_counter--;
    } else {
        // Full update: write same data to both RAMs
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_BW, 
                                                        ssd1685->framebuffer, fb_size), TAG, "send command failed");

        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_RED,
                                                        ssd1685->framebuffer, fb_size), TAG, "send command failed");
    }

    // Save current as previous for next partial update
    memcpy(ssd1685->previous_buffer, ssd1685->framebuffer, fb_size);

    // Update display
    ESP_RETURN_ON_ERROR(ssd1685_update_display(ssd1685), TAG, "update display failed");

    ssd1685->first_update = false;

    return ESP_OK;
}

static esp_err_t ssd1685_update_display(ssd1685_panel_t *ssd1685)
{
    uint8_t data;
    
    // Determine update sequence
    if (ssd1685->refresh_mode == SSD1685_REFRESH_FAST && ssd1685->first_update) {
        // Fast refresh trick: set high temperature
        data = 0x6E; // 110°C
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_TEMP_SENSOR_WRITE, &data, 1), TAG, "write temp failed");
        
        data = 0x91;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1), TAG, "update ctrl2 failed");
        
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0), TAG, "activate failed");
        ssd1685_wait_busy(ssd1685);
        
        data = 0xC7; // Fast refresh without temp load
    } else if (!ssd1685->first_update && ssd1685->refresh_mode == SSD1685_REFRESH_PARTIAL && ssd1685->partial_counter > 0) {
        // Partial update
        data = 0xFF; // Display mode 2
    } else {
        // Full normal refresh
        data = 0xF7; // Display mode 1
    }

    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1), TAG, "update ctrl2 failed");
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0), TAG, "activate failed");
    
    ssd1685_wait_busy(ssd1685);

    return ESP_OK;
}

static void ssd1685_wait_busy(ssd1685_panel_t *ssd1685)
{
    if (ssd1685->busy_gpio < 0) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Conservative wait
        return;
    }

    int retry = 0;
    while (gpio_get_level(ssd1685->busy_gpio) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        retry++;
        if (retry > 500) { // 5 second timeout
            ESP_LOGW(TAG, "Busy timeout!");
            break;
        }
    }
}

static esp_err_t ssd1685_write_cmd(ssd1685_panel_t *ssd1685, uint8_t cmd, const uint8_t *data, size_t len)
{
    if (len > 0 && data != NULL) {
        return esp_lcd_panel_io_tx_param(ssd1685->io, cmd, data, len);
    } else {
        return esp_lcd_panel_io_tx_param(ssd1685->io, cmd, NULL, 0);
    }
}

static esp_err_t ssd1685_set_window(ssd1685_panel_t *ssd1685, uint16_t x_start, uint16_t y_start,
                                        uint16_t x_end, uint16_t y_end)
{
    // Account for source shift
    x_start += SSD1685_SOURCE_SHIFT;
    x_end += SSD1685_SOURCE_SHIFT;

    uint8_t data[4];
    
    // Set X address range
    data[0] = x_start / 8;
    data[1] = x_end / 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_X_START_END, data, 2), TAG, "set X range failed");

    // Set Y address range
    data[0] = y_start & 0xFF;
    data[1] = y_start >> 8;
    data[2] = y_end & 0xFF;
    data[3] = y_end >> 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_Y_START_END, data, 4), TAG, "set Y range failed");

    return ESP_OK;
}

static esp_err_t ssd1685_set_cursor(ssd1685_panel_t *ssd1685, uint16_t x, uint16_t y)
{
    // Account for source shift
    x += SSD1685_SOURCE_SHIFT;

    uint8_t data[2];
    
    data[0] = x / 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_X_COUNTER, data, 1), TAG, "set X counter failed");

    data[0] = y & 0xFF;
    data[1] = y >> 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_Y_COUNTER, data, 2), TAG, "set Y counter failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1685_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    if (!on_off) {
        // Enter deep sleep
        uint8_t data = 0x01;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DEEP_SLEEP, &data, 1), TAG, "deep sleep failed");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return ESP_OK;
}

static esp_err_t panel_ssd1685_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    // E-paper doesn't support color inversion in the traditional sense
    return ESP_OK;
}

static esp_err_t panel_ssd1685_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    ssd1685->mirror_x = mirror_x;
    ssd1685->mirror_y = mirror_y;
    return ESP_OK;
}

static esp_err_t panel_ssd1685_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    ssd1685->swap_axes = swap_axes;
    return ESP_OK;
}

static esp_err_t panel_ssd1685_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    // Not applicable for e-paper
    return ESP_OK;
}

esp_err_t esp_lcd_ssd1685_set_refresh_mode(esp_lcd_panel_handle_t panel, ssd1685_refresh_mode_t mode)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    ssd1685->refresh_mode = mode;
    return ESP_OK;
}