cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (DEFINED ENV{ESP_IDF_VERSION})
    file(GLOB_RECURSE SOURCE_FILES Source/*.c*)

    idf_component_register(
        SRCS ${SOURCE_FILES}
        INCLUDE_DIRS "Include/"
        PRIV_INCLUDE_DIRS "Private/"
        REQUIRES mbedtls nvs_flash esp_rom esp_timer
    )

    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-unused-variable)
    endif()
else()
    file(GLOB_RECURSE SOURCES "Source/*.c*")

    add_library(TactilityCore OBJECT)

    target_sources(TactilityCore
        PRIVATE ${SOURCES}
    )

    include_directories(
        PRIVATE Private/
    )

    target_include_directories(TactilityCore SYSTEM
        PUBLIC Include/
    )

    add_definitions(-D_Nullable=)
    add_definitions(-D_Nonnull=)

    target_link_libraries(TactilityCore
        PUBLIC mbedtls
        PUBLIC freertos_kernel
    )
endif()