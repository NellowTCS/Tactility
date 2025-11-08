# Add the nested ssd1680 component
set(ssd1680_COMPONENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ssd1680")
add_subdirectory("${ssd1680_COMPONENT_DIR}" ssd1680_build)

idf_component_register(
    SRC_DIRS "Source"
    INCLUDE_DIRS "Source"
    REQUIRES Tactility driver ssd1680
)
