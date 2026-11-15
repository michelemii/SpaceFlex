# Driver components were split in ESP-IDF 5.3 (esp_driver_*); older 5.x use "driver".
set(reqs freertos log esp_rom esp_common)
if(IDF_VERSION_MAJOR GREATER 5 OR (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3))
    list(APPEND reqs esp_driver_spi esp_driver_gpio esp_driver_uart esp_driver_pcnt)
else()
    list(APPEND reqs driver)
endif()

idf_component_register(SRCS "main.c" "ads131e04.c" "proto.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${reqs})
