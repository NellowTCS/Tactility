cmake_minimum_required(VERSION 3.20)

# Emscripten build for web
file(GLOB_RECURSE SOURCES "Source/*.c*")
file(GLOB_RECURSE HEADERS "Source/*.h*")

add_executable(WebSimulator ${SOURCES})

target_sources(WebSimulator
    PRIVATE ${SOURCES}
    PUBLIC ${HEADERS}
)

target_link_libraries(WebSimulator
    PRIVATE Tactility
    PRIVATE TactilityCore
    PRIVATE lvgl
)

# Enable pthreads for threading, SDL2 for input/output
set_target_properties(WebSimulator PROPERTIES
    COMPILE_FLAGS "-pthread -sUSE_SDL=2"
    LINK_FLAGS "-pthread -sUSE_SDL=2 -s PROXY_TO_PTHREAD=1 -s PTHREAD_POOL_SIZE=8 -s ALLOW_MEMORY_GROWTH=1 -s EXPORTED_FUNCTIONS=['_main','_vTaskStartScheduler']"
)

add_definitions(-D_Nullable=)
add_definitions(-D_Nonnull=)
