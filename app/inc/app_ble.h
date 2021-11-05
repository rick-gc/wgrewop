/*
 * @Author       : yfwang
 * @Date         : 2021-08-26 15:55:36
 * @LastEditTime : 2021-08-30 11:04:27
 * @LastEditors  : yfwang
 * @FilePath     : \powergw_ble_freertos\app\inc\app_ble.h
 */

#ifndef _APP_BLE_H_
#define _APP_BLE_H_

void ble_stack_init(void);
void gap_params_init(void);
void gatt_init(void);
void advertising_init(void);
void services_init(void);
void conn_params_init(void);
void peer_manager_init(void);
void advertising_start(void *p_erase_bonds);

#endif /* _APP_BLE_H_ */