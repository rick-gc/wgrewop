/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-11-05 15:25:44
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\main.h
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf_libuarte_async.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_wdt.h"
#include "nrfx_gpiote.h"
#include "nrfx_spim.h"
#include "nrfx_saadc.h"
#include "nrfx_temp.h"
#include "nrf_queue.h"
#include "hardfault.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "custom_board.h"
#include "user_config.h"
#include "sx126x.h"
#include "sx128x.h"
#include "app_ble.h"
#include "platform.h"
#include "common.h"
#include "hdlc.h"
#include "sx1268_platform.h"
#include "sx1280_platform.h"
#include "saadc.h"
#include "mpower.h"
#include "lpower.h"
#include "uarte.h"
#include "lte.h"
#include "user_task.h"

typedef struct
{
	bool freertos_on;
	bool lteState;

	SemaphoreHandle_t semaphore_spi1;
	SemaphoreHandle_t semaphore_spi1_xfer_complete;
	SemaphoreHandle_t semaphore_spi2;
	SemaphoreHandle_t semaphore_spi2_xfer_complete;
	SemaphoreHandle_t semphore_sx1268_rx_done;
	SemaphoreHandle_t semphore_sx1268_tx_done;
	SemaphoreHandle_t semphore_sx1268_dio_done;
	SemaphoreHandle_t semphore_sx1280_1_dio1_done;
	SemaphoreHandle_t semphore_sx1280_2_dio1_done;
	SemaphoreHandle_t semphore_sx1280_1_dio_process;
	SemaphoreHandle_t semphore_sx1280_2_dio_process;
	SemaphoreHandle_t semphore_sx1280_tx_done;
	SemaphoreHandle_t semphore_uarte_lte_tx_done;
	SemaphoreHandle_t semphore_saadc_sample_done;
	SemaphoreHandle_t semphore_lte_connect_done;
	SemaphoreHandle_t semphore_tdma_start;

	xQueueHandle queue_uarte_lte_tx_msg;
	xQueueHandle queue_uarte_lte_rx_msg;
	xQueueHandle queue_sensor_msg;
	xQueueHandle queue_mpower_sensor_msg;
	xQueueHandle queue_mpower_send_msg;
	xQueueHandle queue_sx1268_send_msg;
	xQueueHandle queue_sx1268_receive_msg;

#if NRF_LOG_ENABLED
	TaskHandle_t task_handle_log;
#endif
	TaskHandle_t task_handle_lte;
	TaskHandle_t task_handle_lte_uarte_tx;
	TaskHandle_t task_handle_lte_uarte_rx;
	TaskHandle_t task_handle_sx1268;
	TaskHandle_t task_handle_low_power;
	TaskHandle_t task_handle_tdma;
	TaskHandle_t task_handle_sx1280;
	TaskHandle_t task_handle_sx1280_1_dio_process;
	TaskHandle_t task_handle_sx1280_2_dio_process;
	TaskHandle_t task_handle_micro_power;
	TaskHandle_t task_handle_watch_dog;

	nrf_drv_wdt_channel_id m_channel_id;
} global_t;

extern global_t global;
#endif /* _MAIN_H_ */
