#include "tt_hal.h"

#include <Tactility/Tactility.h>
#include <Tactility/hal/Configuration.h>

extern "C" {

UiScale tt_hal_configuration_get_ui_scale() {
    auto scale = tt::hal::getConfiguration()->uiScale;
    return static_cast<UiScale>(scale);
}

int tt_hal_configuration_get_toolbar_height() {
    return tt::hal::getConfiguration()->uiMetrics.toolbarHeight;
}

int tt_hal_configuration_get_toolbar_button_inset() {
    return tt::hal::getConfiguration()->uiMetrics.toolbarButtonInset;
}

int tt_hal_configuration_get_toolbar_title_padding() {
    return tt::hal::getConfiguration()->uiMetrics.toolbarTitlePadding;
}

int tt_hal_configuration_get_button_padding() {
    return tt::hal::getConfiguration()->uiMetrics.buttonPadding;
}

int tt_hal_configuration_get_button_radius() {
    return tt::hal::getConfiguration()->uiMetrics.buttonRadius;
}

int tt_hal_configuration_get_object_padding() {
    return tt::hal::getConfiguration()->uiMetrics.objectPadding;
}

int tt_hal_configuration_get_object_gap() {
    return tt::hal::getConfiguration()->uiMetrics.objectGap;
}

int tt_hal_configuration_get_object_radius() {
    return tt::hal::getConfiguration()->uiMetrics.objectRadius;
}

int tt_hal_configuration_get_object_border_width() {
    return tt::hal::getConfiguration()->uiMetrics.objectBorderWidth;
}

int tt_hal_configuration_get_list_padding() {
    return tt::hal::getConfiguration()->uiMetrics.listPadding;
}

int tt_hal_configuration_get_list_button_vert_padding() {
    return tt::hal::getConfiguration()->uiMetrics.listButtonVertPadding;
}

int tt_hal_configuration_get_dropdown_height() {
    return tt::hal::getConfiguration()->uiMetrics.dropdownHeight;
}

int tt_hal_configuration_get_switch_width() {
    return tt::hal::getConfiguration()->uiMetrics.switchWidth;
}

int tt_hal_configuration_get_switch_height() {
    return tt::hal::getConfiguration()->uiMetrics.switchHeight;
}

int tt_hal_configuration_get_textarea_padding() {
    return tt::hal::getConfiguration()->uiMetrics.textareaPadding;
}

int tt_hal_configuration_get_launcher_button_size() {
    return tt::hal::getConfiguration()->uiMetrics.launcherButtonSize;
}

int tt_hal_configuration_get_general_vertical_padding() {
    return tt::hal::getConfiguration()->uiMetrics.generalVerticalPadding;
}

int tt_hal_configuration_get_locale_settings_offset() {
    return tt::hal::getConfiguration()->uiMetrics.localeSettingsOffset;
}

int tt_hal_configuration_get_system_info_padding() {
    return tt::hal::getConfiguration()->uiMetrics.systemInfoPadding;
}

int tt_hal_configuration_get_wifi_manage_scrollbar_width() {
    return tt::hal::getConfiguration()->uiMetrics.wifiManageScrollbarWidth;
}

}
