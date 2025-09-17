idf_build_get_property(project_dir PROJECT_DIR)

set(srcs
    "i2c_master.c"
)

set(priv_requires
    esp32-max30100
)

set(include_dirs
    "."
    "ems_common_driver/utils"
    "ems_common_driver/ems_common"
    "ems_common_driver/"
    "ems_common_driver/esp32"
    "ems_common_driver/ems_drivers"
)

if(CONFIG_APP_MAX30100_TEST)
    list(APPEND srcs "max30100_test_app.c")
else()
    list(APPEND srcs
        "ems_ble_ota.c"
        "device_sm.c"
        "gatt_svr.c"
        "hrm.c"
        "main.c"
        "ssd1306_display.c"
        "pwr_button.c"
        "extra_tasks.c"
        "ems_common_driver/utils/movingaverage.c"
        "ems_common_driver/utils/atomos.c"
        "ems_common_driver/utils/str.c"
        "ems_common_driver/ems_common/ems_config.c"
        "ems_common_driver/ems_common/ems_pulse.c"
        "ems_common_driver/ems_common/pwr.c"
        "ems_common_driver/ems_drivers/measure.c"
        "ems_common_driver/ems_common/ems_time.c"
        "ems_common_driver/ems_common/statemash.c"
        "ems_common_driver/esp32/esp32_id.c"
        "ems_common_driver/ems_common/effects.c"
        "ems_common_driver/ems_drivers/ow.c"
        "ems_common_driver/ems_common/cmd_parser.c"
        "ems_common_driver/ems_drivers/hv2201.c"
        "ems_common_driver/ems_drivers/led.c"
        "ems_common_driver/ems_drivers/uart.c"
        "ems_common_driver/ems.c"
    )

    list(APPEND priv_requires
        max5815
        ssd1306
        bt
        nvs_flash
        app_update
        bootloader_support
    )
endif()

idf_component_register(
    SRCS ${srcs}
    PRIV_REQUIRES ${priv_requires}
    INCLUDE_DIRS ${include_dirs}
    EMBED_TXTFILES ${project_dir}/rsa_key/private.pem
)

if(NOT CONFIG_APP_MAX30100_TEST)
    create_esp_enc_img(
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        ${project_dir}/rsa_key/private.pem
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_secure.bin
        app
    )
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
