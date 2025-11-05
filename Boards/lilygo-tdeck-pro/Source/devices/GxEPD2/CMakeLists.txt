idf_component_register(SRCS "GxEPD2_EPD.cpp" "GxEPD2_310_GDEQ031T10.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver spi_flash freertos log)