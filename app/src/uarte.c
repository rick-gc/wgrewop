/*
 * @Author       : yfwang
 * @Date         : 2021-10-14 15:07:51
 * @LastEditTime : 2021-10-26 15:33:06
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\uarte.c
 */

#include "main.h"

uint8_t lte_rx_data_buf[LTE_RX_BUF_LEN];
uint16_t lte_rx_len;

NRF_LIBUARTE_ASYNC_DEFINE(uarte_lte, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 4, 512, 10);

void uarte_event_handler_lte(void *context, nrf_libuarte_async_evt_t *p_evt)
{
	// nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;

	switch (p_evt->type)
	{
	case NRF_LIBUARTE_ASYNC_EVT_ERROR:
		NRF_LOG_ERROR(
			"LTE NRF_LIBUARTE_ASYNC_EVT_ERROR errorsrc: %u,overrun_length:%u ",
			p_evt->data.errorsrc, p_evt->data.overrun_err.overrun_length);
		break;
	case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
		// 如果数据长度超过缓存大小，那么提前处理之前的数据
		if (p_evt->data.rxtx.length <= LTE_RX_BUF_LEN && p_evt->data.rxtx.length > 0)
		{
			// if (lte_rx_len + p_evt->data.rxtx.length >= LTE_RX_BUF_LEN)
			// {
			// 	// 将要处理的数据复制并发送到处理队列
			// 	uarte_rx_copy_and_queue_send_fromISR(global.queue_uarte_lte_rx_msg, lte_rx_data_buf, lte_rx_len);
			// 	lte_rx_len = 0;
			// }
			//缓存数据
			memcpy(lte_rx_data_buf + lte_rx_len, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
			lte_rx_len += p_evt->data.rxtx.length;
			// NRF_LOG_INFO("LTE  RX %d bytes:%s", p_evt->data.rxtx.length, p_evt->data.rxtx.p_data);
			NRF_LOG_HEXDUMP_INFO(p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
			queue_send_ISR(global.queue_uarte_lte_rx_msg, lte_rx_data_buf, lte_rx_len);
			lte_rx_len = 0;
			//启动接收超时定时器
			// xTimerStart(global.timer_handle_lte_rx_timeout, 0);
			// 完成接收内存的释放
			nrf_libuarte_async_rx_free(&uarte_lte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
		}
		break;
	case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
		xSemaphoreGiveFromISR(global.semphore_uarte_lte_tx_done, NULL);
		break;
	default:
		break;
	}
}

void uarte_lte_init(nrf_uarte_baudrate_t baudRate)
{
	nrf_libuarte_async_config_t uarte_config;

	uarte_config.parity = NRF_UARTE_PARITY_EXCLUDED;
	uarte_config.hwfc = NRF_UARTE_HWFC_DISABLED;
	uarte_config.cts_pin = NRF_UARTE_PSEL_DISCONNECTED;
	uarte_config.rts_pin = NRF_UARTE_PSEL_DISCONNECTED;
	uarte_config.int_prio = APP_IRQ_PRIORITY_LOW_MID;
	uarte_config.tx_pin = LTE_UART_TX_PIN;
	uarte_config.rx_pin = LTE_UART_RX_PIN;
	uarte_config.baudrate = baudRate;
	// 这个串口接收的IDLE的参数，这个应当和波特率相适应
	if (NRF_UARTE_BAUDRATE_9600 == baudRate)
	{
		uarte_config.timeout_us = 10000;
	}
	else if (NRF_UARTE_BAUDRATE_115200 == baudRate)
	{
		uarte_config.timeout_us = 2000;
	}
	else
	{
		uarte_config.timeout_us = 2000;
	}

	ret_code_t err_code = nrf_libuarte_async_init(&uarte_lte, &uarte_config,
												  uarte_event_handler_lte,
												  NULL);
	APP_ERROR_CHECK(err_code);

	nrf_libuarte_async_enable(&uarte_lte);
}

ret_code_t uarte_tx(uint8_t *buf, uint16_t len)
{
	return nrf_libuarte_async_tx(&uarte_lte, buf, (size_t)len);
}

void uarte_uninit()
{
	nrf_libuarte_async_uninit(&uarte_lte);
}
