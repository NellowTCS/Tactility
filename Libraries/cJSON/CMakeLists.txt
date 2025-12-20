file(GLOB SOURCES "cJSON/*.c")
'
if (EMSCRIPTEN)
    list(FILTER SOURCES EXCLUDE REGEX ".*/test\.c$")
endif()

add_library(cJSON STATIC)
target_sources(cJSON PRIVATE ${SOURCES})
include_directories(cJSON "cJSON/")
target_include_directories(cJSON PUBLIC "cJSON/")
