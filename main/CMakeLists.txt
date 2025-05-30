idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "ems_ble_ota.c" "device_sm.c" "gatt_svr.c" "hrm.c" "i2c_master.c" "main.c" "ssd1306_display.c" "pwr_button.c"
                            "ems_common_driver/utils/movingaverage.c" "ems_common_driver/utils/atomos.c" 
                            "ems_common_driver/ems_common/ems_config.c" "ems_common_driver/ems_common/ems_pulse.c" 
                            "ems_common_driver/ems_common/pwr.c" "ems_common_driver/ems_drivers/measure.c" 
                            "ems_common_driver/ems_common/ems_time.c" "ems_common_driver/ems_common/ems_time.c"
                            "ems_common_driver/ems_common/statemash.c"
                            "ems_common_driver/ems_drivers/hv2201.c" 
                            "ems_common_driver/ems.c"
                       PRIV_REQUIRES esp32-max30100 max5815 ssd1306 bt nvs_flash app_update
                       INCLUDE_DIRS "." "ems_common_driver/utils" "ems_common_driver/ems_common" "ems_common_driver/" "ems_common_driver/esp32" "ems_common_driver/ems_drivers"
                       EMBED_TXTFILES ${project_dir}/rsa_key/private.pem)

create_esp_enc_img(${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
${project_dir}/rsa_key/private.pem ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_secure.bin app)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)