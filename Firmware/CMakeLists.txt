cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE SOURCE_FILES "Source/*.c*")

if (DEFINED ENV{ESP_IDF_VERSION})
    # Read board id/project
    include("../Buildscripts/board.cmake")
    init_tactility_globals("../sdkconfig")
    get_property(TACTILITY_BOARD_PROJECT GLOBAL PROPERTY TACTILITY_BOARD_PROJECT)

    idf_component_register(
        SRCS ${SOURCE_FILES}
        REQUIRES ${BOARD_COMPONENTS}
        REQUIRES Tactility TactilityC ${TACTILITY_BOARD_PROJECT} esp_http_client json
    )
else ()

    add_executable(FirmwareSim ${SOURCE_FILES})
    target_link_libraries(FirmwareSim
        PRIVATE Tactility
        PRIVATE TactilityCore
        PRIVATE Simulator
        PRIVATE SDL2::SDL2-static SDL2-static
    )

    add_definitions(-D_Nullable=)
    add_definitions(-D_Nonnull=)

    if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        # Bundle system and data partitions into the virtual filesystem so assets/settings are available at runtime
        target_link_options(AppSim PRIVATE
            "--preload-file=${CMAKE_SOURCE_DIR}/Data/system@/system"
            "--preload-file=${CMAKE_SOURCE_DIR}/Data/data@/data"
        )
    endif ()
endif ()