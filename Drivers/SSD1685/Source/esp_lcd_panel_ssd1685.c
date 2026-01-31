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
#include "esp_lcd_panel_ssd1685.h"

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
#define DATA_ENTRY_X_INC_Y_DEC              0x01
#define DATA_ENTRY_X_DEC_Y_INC              0x02

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
static esp_err_t ssd1685_update_display(ssd1685_panel_t *ssd1685, bool use_partial);
static void ssd1685_transform_bitmap(const uint8_t *src, uint8_t *dst, int width, int height,
                                      bool swap_axes, bool mirror_x, bool mirror_y);

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
        ssd1685->swap_axes = false;
        ssd1685->mirror_x = false;
        ssd1685->mirror_y = false;
    }

    // Allocate framebuffers with DMA-capable memory
    size_t fb_size = (SSD1685_WIDTH * SSD1685_HEIGHT) / 8;
    ssd1685->framebuffer = heap_caps_calloc(1, fb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(ssd1685->framebuffer, ESP_ERR_NO_MEM, err, TAG, "no mem for framebuffer");
    
    ssd1685->previous_buffer = heap_caps_calloc(1, fb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
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
    ESP_LOGI(TAG, "SSD1685 panel created @%p (rot: swap=%d mx=%d my=%d)", 
                ssd1685, ssd1685->swap_axes, ssd1685->mirror_x, ssd1685->mirror_y);

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
        gpio_set_level(ssd1685->reset_gpio, ssd1685->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}

static esp_err_t panel_ssd1685_init(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    uint8_t data[3];

    // Software reset
    ESP_LOGI(TAG, "Initializing SSD1685...");
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SW_RESET, NULL, 0), TAG, "sw reset failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Driver output control
    data[0] = (SSD1685_HEIGHT - 1) & 0xFF;
    data[1] = ((SSD1685_HEIGHT - 1) >> 8) & 0xFF;
    data[2] = 0x00;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DRIVER_OUTPUT, data, 3), TAG, "driver output failed");

    // Border waveform control
    data[0] = 0x05; // Follow LUT
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_BORDER_WAVEFORM, data, 1), TAG, "border waveform failed");

    // Temperature sensor selection
    data[0] = 0x80; // Internal temperature sensor
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_TEMP_SENSOR_SELECT, data, 1), TAG, "temp sensor select failed");

    // Data entry mode: adjust based on mirror settings
    uint8_t entry_mode = DATA_ENTRY_X_INC_Y_INC;
    
    if (ssd1685->mirror_x && ssd1685->mirror_y) {
        entry_mode = DATA_ENTRY_X_DEC_Y_DEC;
    } else if (ssd1685->mirror_x) {
        entry_mode = DATA_ENTRY_X_DEC_Y_INC;
    } else if (ssd1685->mirror_y) {
        entry_mode = DATA_ENTRY_X_INC_Y_DEC;
    }
    
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DATA_ENTRY_MODE, &entry_mode, 1), TAG, "data entry mode failed");

    // Display Update Control 1
    data[0] = 0x00; // RED mode normal
    data[1] = 0x00; // Source output mode
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, data, 2), TAG, "display update ctrl1 failed");

    // Set full window
    ESP_RETURN_ON_ERROR(ssd1685_set_window(ssd1685, 0, 0, SSD1685_WIDTH - 1, SSD1685_HEIGHT - 1), TAG, "set window failed");
    ESP_RETURN_ON_ERROR(ssd1685_set_cursor(ssd1685, 0, 0), TAG, "set cursor failed");

    ssd1685_wait_busy(ssd1685);

    ESP_LOGI(TAG, "SSD1685 panel initialized (entry_mode=0x%02X)", entry_mode);
    return ESP_OK;
}

static esp_err_t panel_ssd1685_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
                                            int x_end, int y_end, const void *color_data)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "color_data is NULL");

    // Calculate dimensions from coordinates (x_end/y_end are inclusive)
    int width = x_end - x_start + 1;
    int height = y_end - y_start + 1;
    
    ESP_LOGD(TAG, "draw_bitmap: x=%d..%d y=%d..%d (w=%d h=%d)", 
                x_start, x_end, y_start, y_end, width, height);

    const uint8_t *src_data = (const uint8_t *)color_data;
    size_t fb_size = (SSD1685_WIDTH * SSD1685_HEIGHT) / 8;

    // Handle full-screen updates (most common case)
    if (x_start == 0 && y_start == 0 && width == SSD1685_WIDTH && height == SSD1685_HEIGHT) {
        ESP_LOGD(TAG, "Full screen update");
        
        // Apply rotation/mirroring transformation if needed
        if (ssd1685->swap_axes || ssd1685->mirror_x || ssd1685->mirror_y) {
            ESP_LOGD(TAG, "Applying transformation: swap=%d mx=%d my=%d", 
                    ssd1685->swap_axes, ssd1685->mirror_x, ssd1685->mirror_y);
            ssd1685_transform_bitmap(src_data, ssd1685->framebuffer, width, height,
                                    ssd1685->swap_axes, ssd1685->mirror_x, ssd1685->mirror_y);
        } else {
            memcpy(ssd1685->framebuffer, src_data, fb_size);
        }
    } else {
        // Partial screen update
        ESP_LOGD(TAG, "Partial screen update");
        
        if (x_start % 8 != 0) {
            ESP_LOGW(TAG, "Partial updates must be byte-aligned (x_start=%d)", x_start);
            x_start = (x_start / 8) * 8;
            width = (width + 7) & ~7;
        }
        
        int bytes_per_row = width / 8;
        int fb_bytes_per_row = SSD1685_WIDTH / 8;
        
        for (int y = 0; y < height && (y_start + y) < SSD1685_HEIGHT; y++) {
            int dst_offset = ((y_start + y) * fb_bytes_per_row) + (x_start / 8);
            int src_offset = y * bytes_per_row;
            memcpy(&ssd1685->framebuffer[dst_offset], &src_data[src_offset], bytes_per_row);
        }
    }

    // Determine refresh mode
    bool use_partial = false;
    if (!ssd1685->first_update && ssd1685->refresh_mode == SSD1685_REFRESH_PARTIAL) {
        use_partial = true;
        ssd1685->partial_counter++;
        
        if (ssd1685->partial_counter >= 5) {
            ESP_LOGI(TAG, "Periodic full refresh (counter=%d)", ssd1685->partial_counter);
            use_partial = false;
            ssd1685->partial_counter = 0;
        }
    } else {
        ssd1685->partial_counter = 0;
    }

    ESP_LOGD(TAG, "Update mode: %s (first=%d counter=%d)", 
            use_partial ? "PARTIAL" : "FULL", ssd1685->first_update, ssd1685->partial_counter);

    ESP_RETURN_ON_ERROR(ssd1685_set_window(ssd1685, 0, 0, SSD1685_WIDTH - 1, SSD1685_HEIGHT - 1), 
                        TAG, "set window failed");
    ESP_RETURN_ON_ERROR(ssd1685_set_cursor(ssd1685, 0, 0), TAG, "set cursor failed");

    if (use_partial) {
        ESP_LOGD(TAG, "Writing partial: previous->RED, current->BW");
        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_RED,
                                                        ssd1685->previous_buffer, fb_size), 
                            TAG, "write RED RAM failed");
        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_BW,
                                                        ssd1685->framebuffer, fb_size), 
                            TAG, "write BW RAM failed");
    } else {
        ESP_LOGD(TAG, "Writing full: current->BW and current->RED");
        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_BW, 
                                                        ssd1685->framebuffer, fb_size), 
                            TAG, "write BW RAM failed");
        
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(ssd1685->io, SSD1685_CMD_WRITE_RAM_RED,
                                                        ssd1685->framebuffer, fb_size), 
                            TAG, "write RED RAM failed");
    }

    memcpy(ssd1685->previous_buffer, ssd1685->framebuffer, fb_size);

    ESP_RETURN_ON_ERROR(ssd1685_update_display(ssd1685, use_partial), TAG, "update display failed");

    ssd1685->first_update = false;

    return ESP_OK;
}

static esp_err_t ssd1685_update_display(ssd1685_panel_t *ssd1685, bool use_partial)
{
    uint8_t data;
    
    ESP_LOGD(TAG, "Display update: mode=%s refresh_mode=%d", 
              use_partial ? "PARTIAL" : "FULL", ssd1685->refresh_mode);

    if (use_partial) {
        ESP_LOGD(TAG, "Executing partial refresh sequence");
        
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                            TAG, "update ctrl1[0] failed");
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                            TAG, "update ctrl1[1] failed");
        
        data = 0xC0;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_BORDER_WAVEFORM, &data, 1),
                           TAG, "border waveform failed");
        
        data = 0xDF; // CORRECT partial update command
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1),
                           TAG, "update ctrl2 failed");
        
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0),
                           TAG, "activate failed");
        
        ssd1685_wait_busy(ssd1685);
        ESP_LOGD(TAG, "Partial refresh complete");
        
    } else if (ssd1685->refresh_mode == SSD1685_REFRESH_FAST) {
        ESP_LOGD(TAG, "Executing fast refresh sequence");
        
        data = 0x40;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                           TAG, "update ctrl1[0] failed");
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                           TAG, "update ctrl1[1] failed");
        
        data = 0x6E;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_TEMP_SENSOR_WRITE, &data, 1),
                           TAG, "write temp failed");
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_TEMP_SENSOR_WRITE, &data, 1),
                           TAG, "write temp failed");
        
        data = 0x91;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1),
                           TAG, "load temp failed");
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0),
                           TAG, "activate failed");
        
        vTaskDelay(pdMS_TO_TICKS(2));
        
        data = 0xC7;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1),
                           TAG, "fast refresh failed");
        
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0),
                           TAG, "activate failed");
        
        ssd1685_wait_busy(ssd1685);
        ESP_LOGD(TAG, "Fast refresh complete");
        
    } else {
        ESP_LOGD(TAG, "Executing normal full refresh sequence");
        
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                           TAG, "update ctrl1[0] failed");
        data = 0x00;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, &data, 1), 
                           TAG, "update ctrl1[1] failed");
        
        data = 0xF7;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &data, 1),
                           TAG, "update ctrl2 failed");
        
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_MASTER_ACTIVATE, NULL, 0),
                           TAG, "activate failed");
        
        ssd1685_wait_busy(ssd1685);
        ESP_LOGD(TAG, "Normal refresh complete");
    }

    return ESP_OK;
}

static void ssd1685_transform_bitmap(const uint8_t *src, uint8_t *dst, int width, int height,
                                      bool swap_axes, bool mirror_x, bool mirror_y)
{
    ESP_LOGD(TAG, "Transforming bitmap: %dx%d swap=%d mx=%d my=%d", 
             width, height, swap_axes, mirror_x, mirror_y);
    
    size_t dst_size = (width * height) / 8;
    memset(dst, 0xFF, dst_size);
    
    int dst_width = swap_axes ? height : width;
    int dst_height = swap_axes ? width : height;
    
    for (int src_y = 0; src_y < height; src_y++) {
        for (int src_x = 0; src_x < width; src_x++) {
            int src_bit_index = src_y * width + src_x;
            int src_byte_index = src_bit_index / 8;
            int src_bit_offset = 7 - (src_bit_index % 8);
            bool pixel = (src[src_byte_index] >> src_bit_offset) & 0x01;
            
            int dst_x = src_x;
            int dst_y = src_y;
            
            if (swap_axes) {
                int tmp = dst_x;
                dst_x = dst_y;
                dst_y = tmp;
            }
            
            if (mirror_x) {
                dst_x = dst_width - 1 - dst_x;
            }
            
            if (mirror_y) {
                dst_y = dst_height - 1 - dst_y;
            }
            
            int dst_bit_index = dst_y * dst_width + dst_x;
            int dst_byte_index = dst_bit_index / 8;
            int dst_bit_offset = 7 - (dst_bit_index % 8);
            
            if (pixel) {
                dst[dst_byte_index] |= (1 << dst_bit_offset);
            } else {
                dst[dst_byte_index] &= ~(1 << dst_bit_offset);
            }
        }
    }
}

static void ssd1685_wait_busy(ssd1685_panel_t *ssd1685)
{
    if (ssd1685->busy_gpio < 0) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        return;
    }

    int retry = 0;
    const int max_retries = 600;
    
    ESP_LOGD(TAG, "Waiting for BUSY pin...");
    uint32_t start = xTaskGetTickCount();
    
    while (gpio_get_level(ssd1685->busy_gpio) == 1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        retry++;
        if (retry > max_retries) {
            ESP_LOGW(TAG, "BUSY timeout after %d retries!", retry);
            break;
        }
    }
    
    uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - start);
    ESP_LOGD(TAG, "BUSY wait complete (%lu ms, %d checks)", elapsed, retry);
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
    x_start += SSD1685_SOURCE_SHIFT;
    x_end += SSD1685_SOURCE_SHIFT;

    uint8_t data[4];
    
    data[0] = x_start / 8;
    data[1] = x_end / 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_X_START_END, data, 2), 
                       TAG, "set X range failed");

    data[0] = y_start & 0xFF;
    data[1] = (y_start >> 8) & 0xFF;
    data[2] = y_end & 0xFF;
    data[3] = (y_end >> 8) & 0xFF;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_Y_START_END, data, 4), 
                       TAG, "set Y range failed");

    return ESP_OK;
}

static esp_err_t ssd1685_set_cursor(ssd1685_panel_t *ssd1685, uint16_t x, uint16_t y)
{
    x += SSD1685_SOURCE_SHIFT;

    uint8_t data[2];
    
    data[0] = x / 8;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_X_COUNTER, data, 1), 
                       TAG, "set X counter failed");

    data[0] = y & 0xFF;
    data[1] = (y >> 8) & 0xFF;
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_SET_RAM_Y_COUNTER, data, 2), 
                       TAG, "set Y counter failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1685_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    if (!on_off) {
        ESP_LOGI(TAG, "Entering deep sleep");
        uint8_t data = 0x01;
        ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DEEP_SLEEP, &data, 1), 
                           TAG, "deep sleep failed");
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        ESP_LOGI(TAG, "Waking from sleep");
        ESP_RETURN_ON_ERROR(panel_ssd1685_reset(panel), TAG, "reset failed");
        ESP_RETURN_ON_ERROR(panel_ssd1685_init(panel), TAG, "reinit failed");
    }
    
    return ESP_OK;
}

static esp_err_t panel_ssd1685_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ESP_LOGW(TAG, "Color inversion not implemented for e-paper");
    return ESP_OK;
}

static esp_err_t panel_ssd1685_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_LOGI(TAG, "Setting mirror: x=%d y=%d", mirror_x, mirror_y);
    
    ssd1685->mirror_x = mirror_x;
    ssd1685->mirror_y = mirror_y;
    
    uint8_t entry_mode = DATA_ENTRY_X_INC_Y_INC;
    
    if (mirror_x && mirror_y) {
        entry_mode = DATA_ENTRY_X_DEC_Y_DEC;
    } else if (mirror_x) {
        entry_mode = DATA_ENTRY_X_DEC_Y_INC;
    } else if (mirror_y) {
        entry_mode = DATA_ENTRY_X_INC_Y_DEC;
    }
    
    ESP_RETURN_ON_ERROR(ssd1685_write_cmd(ssd1685, SSD1685_CMD_DATA_ENTRY_MODE, &entry_mode, 1), 
                       TAG, "data entry mode failed");
    
    return ESP_OK;
}

static esp_err_t panel_ssd1685_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_LOGI(TAG, "Setting swap_axes: %d", swap_axes);
    ssd1685->swap_axes = swap_axes;
    
    return ESP_OK;
}

static esp_err_t panel_ssd1685_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    return ESP_OK;
}

esp_err_t esp_lcd_ssd1685_set_refresh_mode(esp_lcd_panel_handle_t panel, ssd1685_refresh_mode_t mode)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handle is NULL");
    
    ssd1685_panel_t *ssd1685 = __containerof(panel, ssd1685_panel_t, base);
    
    ESP_LOGI(TAG, "Setting refresh mode: %d", mode);
    ssd1685->refresh_mode = mode;
    ssd1685->partial_counter = 0;
    
    return ESP_OK;
}
