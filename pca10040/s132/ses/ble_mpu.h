/* 
  * This code is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#ifndef MPU_SERVICE_H__
#define MPU_SERVICE_H__

#include "ble.h"
#include "ble_srv_common.h"
//#include "app_mpu.h"
#include <stdint.h>

// Base UUID
#define BLE_MPU_BASE_UUID {0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8B, 0x22}

// Service UUID
#define BLE_UUID_MPU_SERVICE_UUID 0xA3A0 // Just a random, but recognizable value

// Characteristic UUIDs
#define BLE_UUID_ACCEL_CHARACTERISTC_UUID 0xA3A1 // Just a random, but recognizable value
#define BLE_UUID_GYROS_CHARACTERISTC_UUID 0xA3A2
#define BLE_UUID_MAGN_CHARACTERISTC_UUID 0xA3A3
#define BLE_UUID_TEMP_CHARACTERISTC_UUID 0xA3A4 //0x2A1C //using standard temp measurement char:0x2A1C
#define BLE_UUID_COMBINED_CHARACTERISTC_UUID 0xA3A5

typedef struct
{
  uint8_t uuid_type;
  uint16_t conn_handle;                        /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
  uint16_t service_handle;                     /**< Handle of ble Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t combined_char_handles;
  uint8_t mpu_buffer[240]; //20 sensor readings!
  uint8_t mpu_count;
} ble_mpu_t;

/**@brief Function for handling BLE Stack events related to mpu service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to mpu Service.
 *
 * @param[in]   p_mpu       mpu structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_mpu_on_ble_evt(ble_mpu_t *p_mpu, ble_evt_t *p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_mpu       Pointer to ble mpu structure.
 */
void ble_mpu_service_init(ble_mpu_t *p_mpu);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_mpu                     mpu structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255) || defined(MPU9250))
//void ble_mpu_combined_update(ble_mpu_t *p_mpu, combined_values_t *combined_values);
void ble_mpu_combined_update_v2(ble_mpu_t *p_mpu);
//void ble_mpu_accel_update(ble_mpu_t *p_mpu, accel_values_t *accel_values);
//void ble_mpu_gyro_update(ble_mpu_t *p_mpu, gyro_values_t *gyro_values);
//void ble_mpu_magnt_update(ble_mpu_t *p_mpu, magn_values_t *magn_values);
//void ble_mpu_temp_update(ble_mpu_t *p_mpu, temp_value_t *temperature);
#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/


#endif /* _ MPU_SERVICE_H__ */