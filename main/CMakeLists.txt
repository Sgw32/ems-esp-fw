idf_component_register(SRCS "main.c" "ssd1306_display.c" "pwr_button.c" "ble_service.c"
                       REQUIRES esp32-max30100
                       INCLUDE_DIRS ".")
