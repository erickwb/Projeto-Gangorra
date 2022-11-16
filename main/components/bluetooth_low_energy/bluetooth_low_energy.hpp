#ifndef BLUETOOTH_LOW_ENERGY_H
#define BLUETOOTH_LOW_ENERGY_H

#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define GATTS_TAG "BLE TASK"

enum bluetooth_profile_id {
     PROFILE_ANGLE_APP_ID,
     PROFILE_P_CONTROL_APP_ID,
     PROFILE_I_CONTROL_APP_ID,
     PROFILE_D_CONTROL_APP_ID,
     PROFILE_MODE_CONTROL_APP_ID,
     PROFILE_SENSOR_CONTROL_APP_ID
};

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

#endif