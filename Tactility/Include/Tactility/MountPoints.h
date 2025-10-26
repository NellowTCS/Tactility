#pragma once

#include <dirent.h>
#include <vector>

namespace tt::file {

constexpr auto* SYSTEM_PARTITION_NAME = "system";

#if defined(ESP_PLATFORM)
constexpr auto* MOUNT_POINT_SYSTEM = "/system";
#elif defined(__EMSCRIPTEN__)
constexpr auto* MOUNT_POINT_SYSTEM = "/system";
#else
constexpr auto* MOUNT_POINT_SYSTEM = "system";
#endif

constexpr auto* DATA_PARTITION_NAME = "data";

#if defined(ESP_PLATFORM)
constexpr auto* MOUNT_POINT_DATA = "/data";
#elif defined(__EMSCRIPTEN__)
constexpr auto* MOUNT_POINT_DATA = "/data";
#else
constexpr auto* MOUNT_POINT_DATA = "data";
#endif

std::vector<dirent> getMountPoints();

} // namespace
