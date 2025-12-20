file(GLOB SOURCES "cJSON/*.c")
list(FILTER SOURCES EXCLUDE REGEX ".*/test\.c$")

add_library(cJSON STATIC)
target_sources(cJSON PRIVATE ${SOURCES})
include_directories(cJSON "cJSON/")
target_include_directories(cJSON PUBLIC "cJSON/")
