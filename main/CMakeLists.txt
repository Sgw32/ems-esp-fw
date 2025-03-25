idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "ems_ble_ota.c" "device_sm.c" "gatt_svr.c" "hrm.c" "i2c_master.c" "main.c" "ssd1306_display.c" "pwr_button.c"
                       PRIV_REQUIRES esp32-max30100 ssd1306 bt nvs_flash app_update
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ${project_dir}/rsa_key/private.pem)

create_esp_enc_img(${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
${project_dir}/rsa_key/private.pem ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_secure.bin app)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)