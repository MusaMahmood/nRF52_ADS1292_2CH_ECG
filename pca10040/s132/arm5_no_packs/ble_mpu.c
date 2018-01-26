/* 
  * This code is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include "ble_mpu.h"
#include "app_error.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include <stdint.h>
#include <string.h>

void ble_mpu_on_ble_evt(ble_mpu_t *p_mpu, ble_evt_t *p_ble_evt) {
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    p_mpu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    break;
  case BLE_GAP_EVT_DISCONNECTED:
    p_mpu->conn_handle = BLE_CONN_HANDLE_INVALID;
  default:
    // No implementation needed.
    break;
  }
}

static uint32_t ble_char_combined_add(ble_mpu_t *p_mpu) {
  uint32_t err_code = 0; // Variable to hold return codes from library and softdevice functions

  ble_uuid_t char_uuid;
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_COMBINED_CHARACTERISTC_UUID);
  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;

  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.max_len = 24;
  attr_char_value.init_len = 0;
  uint8_t value[24];
  attr_char_value.p_value = value;
  err_code = sd_ble_gatts_characteristic_add(p_mpu->service_handle,
      &char_md,
      &attr_char_value,
      &p_mpu->combined_char_handles);
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_mpu_service_init(ble_mpu_t *p_mpu) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions
  uint16_t service_handle;
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = {BLE_MPU_BASE_UUID};

  err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_mpu->uuid_type));
  APP_ERROR_CHECK(err_code);

  service_uuid.type = p_mpu->uuid_type;
  service_uuid.uuid = BLE_UUID_MPU_SERVICE_UUID;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
  APP_ERROR_CHECK(err_code);

  ble_char_combined_add(p_mpu);
}
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255) || defined(MPU9250))

void ble_mpu_combined_update_v2(ble_mpu_t *p_mpu) {
  uint32_t err_code;
  if (p_mpu->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t hvx_len = 24;
    ble_gatts_hvx_params_t const hvx_params = {
      .handle = p_mpu->combined_char_handles.value_handle,
      .type = BLE_GATT_HVX_NOTIFICATION,
      .offset = 0,
      .p_len = &hvx_len,
      .p_data = p_mpu->mpu_buffer,
    };
    sd_ble_gatts_hvx(p_mpu->conn_handle, &hvx_params);
  }
}

#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/