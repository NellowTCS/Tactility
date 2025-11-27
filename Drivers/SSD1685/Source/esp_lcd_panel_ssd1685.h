/**
 * @file esp_lcd_ssd1685.h
 * @brief ESP LCD driver for SSD1685 e-paper display controller
 * 
 * Specifically designed for GDEY029T71H (168x384 monochrome e-paper)
 */

#pragma once

#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SSD1685 refresh modes
 */
typedef enum {
    SSD1685_REFRESH_FULL = 0,    /*!< Full refresh - slow but complete update */
    SSD1685_REFRESH_FAST,        /*!< Fast full refresh - uses temperature trick */
    SSD1685_REFRESH_PARTIAL,     /*!< Partial refresh - fast but may ghost */
} ssd1685_refresh_mode_t;

/**
 * @brief SSD1685 panel configuration
 */
typedef struct {
    int busy_gpio;                      /*!< BUSY signal GPIO, -1 if not used */
    ssd1685_refresh_mode_t refresh_mode; /*!< Default refresh mode */
    bool swap_axes;                     /*!< Swap X and Y axes */
    bool mirror_x;                      /*!< Mirror X axis */
    bool mirror_y;                      /*!< Mirror Y axis */
} esp_lcd_ssd1685_config_t;

/**
 * @brief Create LCD panel for SSD1685 (GDEY029T71H)
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config General panel device configuration
 * @param[in] ssd1685_config SSD1685 specific configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameter is invalid
 *      - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t esp_lcd_new_panel_ssd1685(const esp_lcd_panel_io_handle_t io,
                                     const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     const esp_lcd_ssd1685_config_t *ssd1685_config,
                                     esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Set refresh mode for SSD1685 panel
 *
 * @param[in] panel LCD panel handle
 * @param[in] mode Refresh mode to set
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameter is invalid
 */
esp_err_t esp_lcd_ssd1685_set_refresh_mode(esp_lcd_panel_handle_t panel, 
                                            ssd1685_refresh_mode_t mode);

#ifdef __cplusplus
}
#endif