cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ems_esp_fw VERSION 3.0.0)

idf_component_get_property(ble_conn_mgr_dir espressif__ble_conn_mgr COMPONENT_DIR)
target_compile_options(__idf_espressif__ble_conn_mgr PRIVATE -Wno-error=maybe-uninitialized)
