#pragma once

#include "Tactility/Lock.h"

#include <Tactility/hal/display/DisplayDevice.h>

#include <esp_lcd_types.h>
#include <esp_lvgl_port_disp.h>
#include <Tactility/Check.h>

/**
 * Get the underlying panel IO handle used for sending commands to the panel.
 *
 * @return The current esp_lcd_panel_io_handle_t, or nullptr if not created.
 */
/**
 * Create and initialize the panel IO handle required by the concrete display.
 *
 * Implementations must allocate/configure an IO handle and assign it to outHandle.
 *
 * @param[out] outHandle Set to the created esp_lcd_panel_io_handle_t on success.
 * @return true if the IO handle was created successfully; false otherwise.
 */
/**
 * Create and initialize the panel handle using a previously created IO handle.
 *
 * Implementations must create the panel handle (panel driver object) tied to the provided IO handle.
 *
 * @param ioHandle The IO handle returned from createIoHandle.
 * @param[out] panelHandle Set to the created esp_lcd_panel_handle_t on success.
 * @return true if the panel handle was created successfully; false otherwise.
 */
/**
 * Provide the LVGL port display configuration for this panel.
 *
 * Implementations must return a fully populated lvgl_port_display_cfg_t that will be used to register
 * and configure the display with the LVGL port layer.
 *
 * @param ioHandle The panel IO handle to use when building the LVGL config.
 * @param panelHandle The panel handle to use when building the LVGL config.
 * @return A configured lvgl_port_display_cfg_t for this panel.
 */
/**
 * Indicates whether the panel is an RGB parallel panel.
 *
 * Override to return true for RGB-parallel panels; default is false.
 *
 * @return true if the panel is RGB-parallel; otherwise false.
 */
/**
 * Provide the LVGL RGB-specific port configuration.
 *
 * Only required for RGB-parallel panels. Default implementation aborts (not supported).
 *
 * @param ioHandle The panel IO handle to use when building the RGB LVGL config.
 * @param panelHandle The panel handle to use when building the RGB LVGL config.
 * @return A configured lvgl_port_display_rgb_cfg_t for RGB panels.
 */
/**
 * Construct an EspLcdDisplay.
 *
 * The provided lock is stored for use by the display instance.
 */
/**
 * Destroy the EspLcdDisplay and release resources.
 */
/**
 * Return the lock associated with this display.
 *
 * @return Shared pointer to the stored tt::Lock.
 */
/**
 * Start the display device and initialize hardware resources.
 *
 * This performs the device's startup sequence; returns true on successful start.
 *
 * @return true if the device started successfully; false otherwise.
 */
/**
 * Stop the display device and release hardware resources.
 *
 * This performs the device's shutdown sequence; returns true on successful stop.
 *
 * @return true if the device stopped successfully; false otherwise.
 */
/**
 * Report LVGL support availability for this device.
 *
 * @return true — LVGL integration is supported.
 */
/**
 * Start LVGL integration for this display.
 *
 * Performs LVGL registration and any required runtime setup.
 *
 * @return true if LVGL was started successfully; false otherwise.
 */
/**
 * Stop LVGL integration for this display.
 *
 * Unregisters the display from LVGL and undoes LVGL-specific state.
 *
 * @return true if LVGL was stopped successfully; false otherwise.
 */
/**
 * Get the LVGL display object associated with this device.
 *
 * @return Pointer to the lv_display_t instance, or nullptr if LVGL is not started.
 */
/**
 * Report DisplayDriver support availability for this device.
 *
 * @return true — DisplayDriver integration is supported.
 */
/**
 * Return a DisplayDriver instance for use by higher-level code.
 *
 * @return Shared pointer to a DisplayDriver if available; otherwise nullptr.
 */
class EspLcdDisplay : public tt::hal::display::DisplayDevice {

    esp_lcd_panel_io_handle_t _Nullable ioHandle = nullptr;
    esp_lcd_panel_handle_t _Nullable panelHandle = nullptr;
    lv_display_t* _Nullable lvglDisplay = nullptr;
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable displayDriver;
    std::shared_ptr<tt::Lock> lock;
    lcd_rgb_element_order_t rgbElementOrder;

protected:

    // Used for sending commands such as setting curves
    esp_lcd_panel_io_handle_t getIoHandle() const { return ioHandle; }

    virtual bool createIoHandle(esp_lcd_panel_io_handle_t& outHandle) = 0;

    virtual bool createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) = 0;

    virtual lvgl_port_display_cfg_t getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) = 0;

    virtual bool isRgbPanel() const { return false; }

    virtual lvgl_port_display_rgb_cfg_t getLvglPortDisplayRgbConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) { tt_crash("Not supported"); }

public:

    EspLcdDisplay(std::shared_ptr<tt::Lock> lock) : lock(lock) {}

    ~EspLcdDisplay() override;

    std::shared_ptr<tt::Lock> getLock() const { return lock; }

    bool start() final;

    bool stop() final;

    // region LVGL

    bool supportsLvgl() const final { return true; }

    bool startLvgl() final;

    bool stopLvgl() final;

    lv_display_t* _Nullable getLvglDisplay() const final { return lvglDisplay; }

    // endregion

    // region DisplayDriver

    bool supportsDisplayDriver() const override { return true; }

    /** @return a NativeDisplay instance if this device supports it */
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() final;

    // endregion
};
