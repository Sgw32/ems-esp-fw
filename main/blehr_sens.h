/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

 #ifndef H_BLEHR_SENSOR_
 #define H_BLEHR_SENSOR_
 
 #include "nimble/ble.h"
 #include "modlog/modlog.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /** @file
  *  @brief BLE Heart Rate Sensor Service Header
  *
  *  This file contains definitions for the BLE Heart Rate Sensor (HRS) service,
  *  including characteristic UUIDs and function declarations.
  */
 
 /** @name Heart Rate Service UUIDs
  *  UUID definitions for the Heart Rate Service and its characteristics.
  *  @{
  */
 #define GATT_HRS_UUID                           0x180D  /**< Heart Rate Service UUID */
 #define GATT_HRS_MEASUREMENT_UUID               0x2A37  /**< Heart Rate Measurement UUID */
 #define GATT_HRS_BODY_SENSOR_LOC_UUID           0x2A38  /**< Body Sensor Location UUID */
 #define GATT_DEVICE_INFO_UUID                   0x180A  /**< Device Information Service UUID */
 #define GATT_MANUFACTURER_NAME_UUID             0x2A29  /**< Manufacturer Name UUID */
 #define GATT_MODEL_NUMBER_UUID                  0x2A24  /**< Model Number UUID */
 /** @} */
 
 /** @brief Handle for the Heart Rate Measurement characteristic */
 extern uint16_t hrs_hrm_handle;
 
 /** @brief Forward declarations for BLE stack structures */
 struct ble_hs_cfg;
 struct ble_gatt_register_ctxt;
 
 /**
  * @brief GATT server registration callback.
  *
  * This function is called when a GATT attribute is registered.
  *
  * @param ctxt Pointer to the registration context.
  * @param arg User-defined argument.
  */
 void ems_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
 
 /**
  * @brief Initializes the GATT server.
  *
  * This function sets up the GATT attributes and services for the Heart Rate Sensor.
  *
  * @return 0 on success, or an error code on failure.
  */
 int ems_gatt_svr_init(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* H_BLEHR_SENSOR_ */
 