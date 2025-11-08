#include "Tactility/service/wifi/WifiBootSplashInit.h"
#include "Tactility/file/PropertiesFile.h"

#include <Tactility/file/File.h>
#include <Tactility/MountPoints.h>
#include <Tactility/Log.h>
#include <Tactility/service/wifi/WifiApSettings.h>

#include <dirent.h>
#include <format>
#include <map>
#include <string>
#include <vector>
#include <Tactility/Tactility.h>
#include <Tactility/hal/sdcard/SdCardDevice.h>

namespace tt::service::wifi {

constexpr auto* TAG = "WifiBootSplashInit";

constexpr auto* AP_PROPERTIES_KEY_SSID = "ssid";
constexpr auto* AP_PROPERTIES_KEY_PASSWORD = "password";
constexpr auto* AP_PROPERTIES_KEY_AUTO_CONNECT = "autoConnect";
constexpr auto* AP_PROPERTIES_KEY_CHANNEL = "channel";
constexpr auto* AP_PROPERTIES_KEY_AUTO_REMOVE = "autoRemovePropertiesFile";

struct ApProperties {
    std::string ssid;
    std::string password;
    bool autoConnect;
    int32_t channel;
    bool autoRemovePropertiesFile;
};

static void importWifiAp(const std::string& filePath) {
    std::map<std::string, std::string> map;
    if (!file::loadPropertiesFile(filePath, map)) {
        TT_LOG_E(TAG, "Failed to load AP properties at %s", filePath.c_str());
        return;
    }

    const auto ssid_iterator = map.find(AP_PROPERTIES_KEY_SSID);
    if (ssid_iterator == map.end()) {
        TT_LOG_E(TAG, "%s is missing ssid", filePath.c_str());
        return;
    }
    const auto ssid = ssid_iterator->second;

    if (!settings::contains(ssid)) {

        const auto password_iterator = map.find(AP_PROPERTIES_KEY_PASSWORD);
        const auto password = password_iterator == map.end() ? "" : password_iterator->second;

        const auto auto_connect_iterator = map.find(AP_PROPERTIES_KEY_AUTO_CONNECT);
        const auto auto_connect = auto_connect_iterator == map.end() ? true : (auto_connect_iterator->second == "true");

        const auto channel_iterator = map.find(AP_PROPERTIES_KEY_CHANNEL);
        const auto channel = channel_iterator == map.end() ? 0 : std::stoi(channel_iterator->second);

        settings::WifiApSettings settings(
            ssid,
            password,
            auto_connect,
            channel
        );

        if (!settings::save(settings)) {
            TT_LOG_E(TAG, "Failed to save settings for %s", ssid.c_str());
        } else {
            TT_LOG_I(TAG, "Imported %s from %s", ssid.c_str(), filePath.c_str());
        }
    }

    const auto auto_remove_iterator = map.find(AP_PROPERTIES_KEY_AUTO_REMOVE);
    if (auto_remove_iterator != map.end() && auto_remove_iterator->second == "true") {
        if (!remove(filePath.c_str())) {
            TT_LOG_E(TAG, "Failed to auto-remove %s", filePath.c_str());
        } else {
            TT_LOG_I(TAG, "Auto-removed %s", filePath.c_str());
        }
    }
}

static void importWifiApSettings(std::shared_ptr<hal::sdcard::SdCardDevice> sdcard) {
    auto path = file::getChildPath(sdcard->getMountPath(), "settings");

    std::vector<dirent> dirent_list;
    if (file::scandir(path, dirent_list, [](const dirent* entry) {
        switch (entry->d_type) {
            case file::TT_DT_DIR:
            case file::TT_DT_CHR:
            case file::TT_DT_LNK:
                return -1;
            case file::TT_DT_REG:
            default: {
                std::string name = entry->d_name;
                if (name.ends_with(".ap.properties")) {
                    return 0;
                } else {
                    return -1;
                }
            }
        }
    }, nullptr) == 0) {
        return;
    }

    if (dirent_list.empty()) {
        TT_LOG_W(TAG, "No AP files found at %s", sdcard->getMountPath().c_str());
        return;
    }

    for (auto& dirent : dirent_list) {
        std::string absolute_path = std::format("{}/{}", path, dirent.d_name);
        importWifiAp(absolute_path);
    }
}

static void importGlobalWifiProperties() {
    // Support a single global properties file on the data partition so users without an
    // SD card can provision Wi-Fi by writing /data/settings/wifi_provision.properties
    // (we avoid clobbering the service's wifi.properties which stores enableOnBoot)
    auto file_path = file::getChildPath(file::MOUNT_POINT_DATA, "settings/wifi_provision.properties");
    if (!file::isFile(file_path)) {
        return;
    }

    std::map<std::string, std::string> map;
    if (!file::loadPropertiesFile(file_path, map)) {
        TT_LOG_E(TAG, "Failed to load %s", file_path.c_str());
        return;
    }

    const auto ssid_iter = map.find("ssid");
    if (ssid_iter == map.end()) {
        TT_LOG_W(TAG, "%s missing ssid entry", file_path.c_str());
        return;
    }

    const auto ssid = ssid_iter->second;
    const auto password_iter = map.find("password");
    const auto auto_connect_iter = map.find("autoConnect");
    const auto channel_iter = map.find("channel");
    const auto auto_remove_iter = map.find("autoRemovePropertiesFile");

    std::string password = password_iter == map.end() ? std::string() : password_iter->second;
    bool autoConnect = auto_connect_iter == map.end() ? true : (auto_connect_iter->second == "true");
    int32_t channel = channel_iter == map.end() ? 0 : std::stoi(channel_iter->second);
    bool autoRemove = auto_remove_iter == map.end() ? false : (auto_remove_iter->second == "true");

    if (!settings::contains(ssid)) {
        settings::WifiApSettings settings(
            ssid,
            password,
            autoConnect,
            channel
        );

        if (!settings::save(settings)) {
            TT_LOG_E(TAG, "Failed to save settings for %s", ssid.c_str());
        } else {
            TT_LOG_I(TAG, "Imported %s from %s", ssid.c_str(), file_path.c_str());
        }
    }

    if (autoRemove) {
        if (!file::deleteFile(file_path)) {
            TT_LOG_E(TAG, "Failed to auto-remove %s", file_path.c_str());
        } else {
            TT_LOG_I(TAG, "Auto-removed %s", file_path.c_str());
        }
    }
}

void bootSplashInit() {
    getMainDispatcher().dispatch([] {
        const auto sdcards = hal::findDevices<hal::sdcard::SdCardDevice>(hal::Device::Type::SdCard);
        for (auto& sdcard : sdcards) {
            if (sdcard->isMounted()) {
                importWifiApSettings(sdcard);
            } else {
                TT_LOG_W(TAG, "Skipping unmounted SD card %s", sdcard->getMountPath().c_str());
            }
        }
    // Also support provisioning via a single properties file on the data partition
    // so devices without an SD card can be provisioned by dropping
    // /data/settings/wifi_provision.properties
    importGlobalWifiProperties();
    });
}

}
