/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-11-05 14:15:59
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\user_task.c
 */

#include "main.h"

void task_lte()
{
	uint32_t current_tick = xTaskGetTickCount();
	msg_t msg;
	uint8_t lte_mid[4];

	saadc_init();
	mac_addr_init();
	ec600s_io_init();
	// uarte_lte_init(NRF_UARTE_BAUDRATE_115200);

	for (;;)
	{
		ec600s_power_button(EC600_ON);

		if (pdTRUE == xSemaphoreTake(global.semphore_lte_connect_done, portMAX_DELAY))
		{
			lte_get_time_tx();
			lte_version_tx();
			lte_voltage_tx();
			pack_sensor_msg();

			lte_close();
			if (pdTRUE == xSemaphoreTake(global.semphore_lte_connect_done, ssticks(60)))
			{
				ec600s_power_button(EC600_OFF);
			}
			else
			{
				ec600s_power_button(EC600_OFF);
			}
		}
		else
		{
			ec600s_power_button(EC600_OFF);
		}
		// vTaskDelayUntil(&current_tick, ssticks(300));
	}
}

void task_uarte_lte_rx()
{
	msg_t msg, hdlc;
	un_2u8_u16 crc, type;
	un_4u8_u32 mid;

	NRF_LOG_INFO("task_uarte_lte_rx start");
	for (;;)
	{
		if (pdTRUE == xQueueReceive(global.queue_uarte_lte_rx_msg, &msg, portMAX_DELAY))
		{
			hdlc.buf = pvPortMalloc(msg.len);
			if (hdlc.buf != NULL)
			{
				hdlc.len = hdlc_frame_decode(hdlc.buf, msg.buf, msg.len);
				freepoint(msg.buf);

				if (hdlc.len > 3)
				{
					crc.b = RTU_CRC(hdlc.buf, hdlc.len - 2);
					if ((*(hdlc.buf + hdlc.len - 2) == crc.a[1]) &&
						(*(hdlc.buf + hdlc.len - 1) == crc.a[0]))
					{
						type.a[1] = *hdlc.buf;
						type.a[0] = *(hdlc.buf + 1);
						if (type.b != 0xbbbb && hdlc.len > 7)
						{
							mid.a[3] = *(hdlc.buf + 2);
							mid.a[2] = *(hdlc.buf + 3);
							mid.a[1] = *(hdlc.buf + 4);
							mid.a[0] = *(hdlc.buf + 5);
						}
						NRF_LOG_HEXDUMP_INFO(mid.a, 4);
						switch (type.b)
						{
						case 0x0002:
							NRF_LOG_INFO("0002");
							lte_reply(type.b, mid.b);
							switch (*(hdlc.buf + 6))
							{
							case 0:
								xSemaphoreGive(global.semphore_lte_connect_done);
								NRF_LOG_INFO("lte connect success");
								break;
							case 1:
								NRF_LOG_INFO("lte connect fail");
								break;
							case 2:
								NRF_LOG_INFO("lte connect error");
								break;
							}
							break;
						case 0xbbbb:
							NRF_LOG_INFO("close lte");
							xSemaphoreGive(global.semphore_lte_connect_done);
							break;
						case 0x8204:
							NRF_LOG_INFO("get timestamp");
							lte_get_time_rx(hdlc.buf + 6);
							break;
						case 0x8101:
							NRF_LOG_INFO("Version send success");
							break;
						case 0x8102:
							NRF_LOG_INFO("Voltage send success");
							break;
						case 0x8001:
							NRF_LOG_INFO("sensor data send success");
							break;
						case 0x0301:
							NRF_LOG_INFO("path config");
							path_config_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							path_config_tx();
							break;
						case 0x0351:
							NRF_LOG_INFO("black and white list of hj");
							bw_list_hj_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							bw_list_hj_tx();
							break;
						case 0x0353:
							NRF_LOG_INFO("reset black and white list of hj");
							bw_list.dev_white_list_num = 0;
							bw_list.dev_black_list_num = 0;
							lte_reply_state(type.b, mid.b, 1);
							break;
						case 0x0361:
							NRF_LOG_INFO("black and white list of sensor");
							bw_list_sensor_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							bw_list_sensor_tx(hdlc.buf + 6);
							break;
						case 0x0363:
							NRF_LOG_INFO("reset black and white list of sensor");
							bw_list.sen_white_list_num = 0;
							bw_list.sen_black_list_num = 0;
							lte_reply_state(type.b, mid.b, 1);
							break;
						case 0x0211:
							NRF_LOG_INFO("hj path config");
							hj_config_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							hj_config_tx();
							break;
						case 0x0201:
							NRF_LOG_INFO("hj black and white list");
							hj_bw_list_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							hj_bw_list_tx();
							break;
						case 0x0203:
							NRF_LOG_INFO("hj black and white list reset");
							lte_reply_state(type.b, mid.b, 1);
							break;
						case 0x0111:
							NRF_LOG_INFO("sensor config");
							sensor_config_rx(hdlc.buf + 6);
							lte_reply_state(type.b, mid.b, 1);
							break;
						case 0x0110:
							NRF_LOG_INFO("sensor config reply");
							sensor_config_reply(mid.b, hdlc.buf + 6);
							break;
						default:
							break;
						}
					}
					else
					{
						NRF_LOG_INFO("crc error");
					}
				}
				freepoint(hdlc.buf);
			}
		}
	}
}

void task_uarte_lte_tx()
{
	msg_t msg;

	NRF_LOG_INFO("task_uarte_lte_tx start");
	for (;;)
	{
		if (pdTRUE == xQueueReceive(global.queue_uarte_lte_tx_msg, &msg, portMAX_DELAY))
		{
			if (NRF_SUCCESS == uarte_tx(msg.buf, msg.len))
			{
				xSemaphoreTake(global.semphore_uarte_lte_tx_done, portMAX_DELAY);
			}
			else
			{
			}
			freepoint(msg.buf);
		}
	}
}

void task_sx1268()
{
	msg_t msg;
	// uint8_t sensor_data_buff[2048], sensor_num = 0;
	// uint8_t topology_buf[1024] = {0x06, 0x01, 0x01, 0x01}, topology_num = 0;
	// uint16_t sensor_data_len = 3, topology_len = 5;
	NRF_LOG_INFO("1268 task start");

	sx1268_gpio_init();
	spi1_init();

	sx1268.device_id = SX1268_DEVICE_ID;
	sx1268.read_buffer = spi1_read_buffer;			   // SPI读取函数，要求是同步操作
	sx1268.write_buffer = spi1_write_buffer;		   // SPI写入函数，要求是同步操作
	sx1268.write_read_buffer = spi1_write_read_buffer; // SPI写入读取函数，要求是同步操作
	sx1268.delay_ms = platform_delay_ms;			   // 延迟函数
	sx1268.wait_on_busy = sx126x_wait_on_busy;		   // 检查SX126x BUSY引脚是否已经为低电平，见下方
	sx1268.cs_high = sx1268_cs_high;				   // SX1268 SPI CS引脚设置为高电平函数
	sx1268.cs_low = sx1268_cs_low;					   // SX1268 SPI CS引脚设置为低电平函数
	sx1268.rst_high = sx1268_rst_high;				   // SX1268 RST引脚设置为高电平函数
	sx1268.rst_low = sx1268_rst_low;				   // SX1268 RST引脚设置为低电平函数
	sx1268.ant_sw_tx = sx1268_ant_sw_tx;			   // SX1268 控制外部射频开关为发射
	sx1268.ant_sw_rx = sx1268_ant_sw_rx;			   // SX1268 控制外部射频开关为接收
	sx1268.tcxo_enable = sx1268_tcxo_enable;		   // SX1268 外部TCXO供电开启
	sx1268.tcxo_disable = sx1268_tcxo_disable;		   // SX1268 外部TCXO供电关闭

	sx1268.tx_power = 17; // 发射功率
	sx1268.symble_timeout = 5;
	sx1268.rx_continuous = true;				// 是否连续接收
	sx1268.dio2_as_rf_switch_ctrl_flag = false; // 是否将dio2作为射频开关切换控制
	// sx1268.callback.rxDone = lora_rx_done; 		// 接收完成回调函数
	// sx1268.callback.txDone = lora_tx_done; 		// 发送完成回调函数
	// sx1268.callback.txTimeout = lora_tx_timeout; // 发送超时回调函数

	sx1268.modulation_params.PacketType = PACKET_TYPE_LORA;			 // 传输类型为LORA
	sx1268.modulation_params.Params.LoRa.Bandwidth = LORA_BW_500;	 // 带宽设置为500KHz
	sx1268.modulation_params.Params.LoRa.CodingRate = LORA_CR_4_5;	 // CR=4/5
	sx1268.modulation_params.Params.LoRa.LowDatarateOptimize = 0;	 // 低速率优化开关
	sx1268.modulation_params.Params.LoRa.SpreadingFactor = LORA_SF5; // SF=5

	sx1268.packet_params.PacketType = PACKET_TYPE_LORA;
	sx1268.packet_params.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH; // 包含Header，Header中带有数据长度
	sx1268.packet_params.Params.LoRa.CrcMode = LORA_CRC_ON;					   // CRC校验打开
	sx1268.packet_params.Params.LoRa.PayloadLength = 200;					   // PayLoad长度
	sx1268.packet_params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;				   // IQ 配置
	sx1268.packet_params.Params.LoRa.PreambleLength = 8;					   // 前导码长度

	xSemaphoreGive(global.semaphore_spi1);

	if (sx126x_init(&sx1268))
	{
		NRF_LOG_INFO("SX1268 init success.");
	}
	else
	{
		NRF_LOG_WARNING("SX1268 init fail.");
	}

	uint8_t payload_len = 0;
	uint8_t payload_data[255] = {0};
	uint16_t sx1268_irq_status;

	sx126x_set_rf_frequency(&sx1268, 506000000);
	sx126x_rx(&sx1268, 0);

	// xTimerStart(global.timer_handle_sx1268_tx, 3);
	xSemaphoreGive(global.semphore_tdma_start);

	for (;;)
	{
		xSemaphoreTake(global.semphore_sx1268_dio_done, portMAX_DELAY);
		sx1268_irq_status = sx126x_get_irq_status(&sx1268);
		sx126x_clear_irq_status(&sx1268, 0xFFFF);
		// NRF_LOG_INFO("sx1268 irq status %d", sx1268_irq_status);

		if ((sx1268_irq_status & IRQ_TX_DONE) == IRQ_TX_DONE)
		{
			// sx126x_rx(&sx1268, 0);
			xSemaphoreGive(global.semphore_sx1268_tx_done);
			// NRF_LOG_INFO("sx1268 tx done");
		}
		else if ((sx1268_irq_status & IRQ_RX_DONE) == IRQ_RX_DONE)
		{
			sx126x_get_packet_status(&sx1268, &sx1268.packet_status);
			NRF_LOG_INFO("sx1268 rx rssi %d", sx1268.packet_status.Params.LoRa.RssiPkt);
			sx126x_get_payload(&sx1268, payload_data, &payload_len, 255);
			NRF_LOG_INFO("sx1268 get payload len =  %d", payload_len);
			if (payload_len > 0)
			{
				NRF_LOG_HEXDUMP_INFO(payload_data, payload_len);
				msg.len = payload_len;
				msg.buf = pvPortMalloc(msg.len);
				memcpy(msg.buf, payload_data, msg.len);
				xQueueSend(global.queue_sx1268_receive_msg, &msg, portMAX_DELAY);
				NRF_LOG_INFO("1268|%d|%d|%d|||||", uxTaskGetStackHighWaterMark(NULL), xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
			}
		}
		// NRF_LOG_INFO("1268|%d|%d|%d|||||", uxTaskGetStackHighWaterMark(NULL), xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}
}

void task_tdma()
{
	msg_t msg;
	un_2u8_u16 bch_num, crc;
	TickType_t delay_ticks[200] = {0};
	uint8_t i;

	NRF_LOG_INFO("task tdma start");
	// 在configTICK_RATE_HZ = 8192的情况下，200个时间片，其中192个=41，剩余8个=40
	// 总计为8192个tick
	for (i = 0; i < 200; i++)
	{
		if (i % 25 == 0)
		{
			delay_ticks[i] = 40;
		}
		else
		{
			delay_ticks[i] = 41;
		}
	}

	i = 0;
	TickType_t current_tick = xTaskGetTickCount();

	xSemaphoreTake(global.semphore_tdma_start, portMAX_DELAY);
	for (;;)
	{
		// 这里是时隙的起始位置
		if (i == 0)
		{
			bch[10] = bch_num.a[1];
			bch[11] = bch_num.a[0];
			crc.b = RTU_CRC(bch, 24);
			bch[24] = crc.a[1];
			bch[25] = crc.a[0];
			sx126x_tx(&sx1268, bch, 55, 0);
			xSemaphoreTake(global.semphore_sx1268_tx_done, portMAX_DELAY);
			bch_num.b++;
			// NRF_LOG_HEXDUMP_INFO(bch, 55);
		}
		else if (i == 3)
		{
			crc.b = RTU_CRC(lpower_dcch.buf, lpower_dcch.len + 2);
			lpower_dcch.buf[lpower_dcch.len + 2] = crc.a[1];
			lpower_dcch.buf[lpower_dcch.len + 3] = crc.a[0];
			sx126x_tx(&sx1268, lpower_dcch.buf, lpower_dcch.len + 4, 0);
			xSemaphoreTake(global.semphore_sx1268_tx_done, portMAX_DELAY);
			// NRF_LOG_HEXDUMP_INFO(lpower_dcch.buf, lpower_dcch.len + 4);
			if (lpower_dcch.access_flag == 1)
			{
				lpower_dcch.access_flag = 2;
				lpower_dcch.len -= 9;
				lpower_dcch.buf[1] = lpower_dcch.len;
			}
		}

		// for (;;)
		// {
		// 	if (pdTRUE == xQueueReceive(global.queue_sx1268_send_msg, &msg, usticks(10 * 1000)))
		// 	{
		// 		sx126x_tx(&sx1268, msg.buf, msg.len, 0);
		// 		xSemaphoreTake(global.semphore_sx1268_tx_done, portMAX_DELAY);
		// 		freepoint(msg.buf);
		// 	}
		// 	else
		// 	{
		// 		// NRF_LOG_INFO("no dsch");
		// 		break;
		// 	}
		// }
		// sx126x_set_standby(&sx1268, STDBY_RC);

		else if (i == 100)
		{
			sx126x_rx(&sx1268, 0);
		}

		// 每个时隙的开始，i是时隙的编号，0~99表示下行，100~199表示上行
		vTaskDelayUntil(&current_tick, delay_ticks[i]);
		i = (i + 1) % 200;
	}
}

void task_low_power()
{
	msg_t msg;
	uint16_t crc;
	for (;;)
	{
		if (pdTRUE == xQueueReceive(global.queue_sx1268_receive_msg, &msg, usticks(10 * 1000)))
		{
			crc = RTU_CRC(msg.buf, msg.len - 2);
			if ((*(msg.buf + msg.len - 2) == (crc >> 8)) &&
				(*(msg.buf + msg.len - 1) == (uint8_t)crc))
			{
				lpower_message.head = *msg.buf;
				lpower_message.len = *(msg.buf + 1);
				switch (lpower_message.type)
				{
				case BCH:
					// bch_process(*(msg.buf+2),lpower_message.len);
					break;
				case DCCH:
					break;
				case DSCH:
					break;
				case URCH:
					urch_process(msg.buf + 2, lpower_message.len);
					break;
				case USCH:
					NRF_LOG_INFO("usch_process");
					if (lpower_message.layer)
					{
						usch_msg_process(msg.buf + 2, lpower_message.len);
					}
					else
					{
						usch_mac_process(msg.buf + 2, lpower_message.len);
					}
					break;
				default:
					break;
				}
			}
			freepoint(msg.buf);
		}
	}
}

void task_sx1280()
{
	msg_t msg;

	sx1280_ch1_gpio_init();
	sx1280_ch2_gpio_init();
	spi2_init();
	xSemaphoreGive(global.semaphore_spi2);
	sx1280_user_init();

	if (pdTRUE == xSemaphoreTake(global.semphore_sx1280_1_dio1_done, portMAX_DELAY))
	{
		NRF_LOG_INFO("semphore_sx1280_1_dio1_done");
	}
	if (pdTRUE == xSemaphoreTake(global.semphore_sx1280_2_dio1_done, portMAX_DELAY))
	{
		NRF_LOG_INFO("semphore_sx1280_2_dio1_done");
	}

	sx128x_set_rf_frequency(&sx1280_1, SX1280_COM_Freq);
	sx128x_set_rf_frequency(&sx1280_2, SX1280_MEG_Freq);
	sx1280TickTime.Step = SX128X_RADIO_TICK_SIZE_4000_US;
	sx1280TickTime.NbSteps = 0xFFFF;

	xSemaphoreGive(global.semphore_sx1280_1_dio_process);
	xSemaphoreGive(global.semphore_sx1280_2_dio_process);

	sx128x_set_rx(&sx1280_1, sx1280TickTime);
	sx128x_set_rx(&sx1280_2, sx1280TickTime);

	for (;;)
	{
		if (pdTRUE == xQueueReceive(global.queue_mpower_send_msg, &msg, portMAX_DELAY))
		{
			//获取msg
			//判断是否是ack
			//是:revice直接发送
			//不是:peek 等待信号量
			//有:删除队列
			//没有:peek 再次发送 循环3次后退出并删除sent_id
			NRF_LOG_HEXDUMP_INFO(msg.buf, msg.len);
			sx1280_1.packet_params.Params.LoRa.PayloadLength = msg.len;
			sx128x_set_packet_params(&sx1280_1, &sx1280_1.packet_params);
			sx128x_send_payload(&sx1280_1, msg.buf, msg.len, sx1280TickTime);
			xSemaphoreTake(global.semphore_sx1280_tx_done, portMAX_DELAY);
			freepoint(msg.buf);
		}
		NRF_LOG_INFO("1280|%d|%d|%d|||||", uxTaskGetStackHighWaterMark(NULL), xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}
}

void task_sx1280_1_dio_process()
{
	msg_t msg;

	uint8_t payload_buff[253];
	uint8_t payload_buffer_size;
	uint16_t sx1280irq_status;
	uint8_t tempsize, tempoffset;

	xSemaphoreTake(global.semphore_sx1280_1_dio_process, portMAX_DELAY);
	for (;;)
	{
		if (pdTRUE == xSemaphoreTake(global.semphore_sx1280_1_dio1_done, portMAX_DELAY))
		{
			sx1280irq_status = sx128x_get_irq_status(&sx1280_1);
			// NRF_LOG_INFO("SX1280 IRQ status %x", sx1280irq_status);
			sx128x_clear_irq_status(&sx1280_1, SX128X_IRQ_RADIO_ALL);
			if ((sx1280irq_status & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE)
			{
				sx128x_get_rx_nuffer_status(&sx1280_1, &tempsize, &tempoffset);
				// NRF_LOG_INFO("tempsize=%d tempoffset=%d", tempsize, tempoffset);
				sx128x_get_payload(&sx1280_1, &payload_buff, &payload_buffer_size, 253);
				// NRF_LOG_INFO("payload_buffer_size:%d", payload_buffer_size);
				if (payload_buffer_size > 0)
				{
					msg.len = payload_buffer_size;
					msg.buf = pvPortMalloc(msg.len);
					memcpy(msg.buf, payload_buff, msg.len);
					xQueueSend(global.queue_mpower_sensor_msg, &msg, portMAX_DELAY);
					// NRF_LOG_HEXDUMP_INFO(payload_buff, payload_buffer_size);
				}
			}
			else if ((sx1280irq_status & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
			{
				xSemaphoreGive(global.semphore_sx1280_tx_done);
				NRF_LOG_INFO("sx1280_1 tx done");
			}
		}
	}
}

void task_sx1280_2_dio_process()
{
	msg_t msg;
	uint8_t payload_buff[253];
	uint8_t payload_buffer_size;
	uint16_t sx1280irq_status;
	uint8_t tempsize, tempoffset;

	xSemaphoreTake(global.semphore_sx1280_2_dio_process, portMAX_DELAY);
	for (;;)
	{
		if (pdTRUE == xSemaphoreTake(global.semphore_sx1280_2_dio1_done, portMAX_DELAY))
		{
			sx1280irq_status = sx128x_get_irq_status(&sx1280_2);
			NRF_LOG_INFO("SX1280 IRQ status %x", sx1280irq_status);
			sx128x_clear_irq_status(&sx1280_2, SX128X_IRQ_RADIO_ALL);
			if ((sx1280irq_status & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE)
			{
				sx128x_get_rx_nuffer_status(&sx1280_2, &tempsize, &tempoffset);
				// NRF_LOG_INFO("tempsize=%d tempoffset=%d", tempsize, tempoffset);
				sx128x_get_payload(&sx1280_2, &payload_buff, &payload_buffer_size, 253);
				// NRF_LOG_INFO("payload_buffer_size:%d", payload_buffer_size);
				if (payload_buffer_size > 0)
				{
					msg.len = payload_buffer_size;
					msg.buf = pvPortMalloc(msg.len);
					memcpy(msg.buf, payload_buff, msg.len);
					xQueueSend(global.queue_mpower_sensor_msg, &msg, portMAX_DELAY);
					// NRF_LOG_HEXDUMP_INFO(payload_buff, payload_buffer_size);
				}
			}
			else if ((sx1280irq_status & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
			{
				NRF_LOG_INFO("sx1280_2 tx done");
			}
		}
	}
}

void task_micro_power()
{
	msg_t msg;
	uint8_t len, mpreq_type, mpack;
	head_byte hbyte;
	un_8u8_u64 mpower_id;
	for (;;)
	{
		if (pdTRUE == xQueueReceive(global.queue_mpower_sensor_msg, &msg, portMAX_DELAY))
		{
			if (*(msg.buf + msg.len - 1) == checksum(msg.buf, msg.len - 1))
			{
				hbyte.value = *msg.buf;
				len = *(msg.buf + 1);
				mpower_id.a[5] = *(msg.buf + 2);
				mpower_id.a[4] = *(msg.buf + 3);
				mpower_id.a[3] = *(msg.buf + 4);
				mpower_id.a[2] = *(msg.buf + 5);
				mpower_id.a[1] = *(msg.buf + 6);
				mpower_id.a[0] = *(msg.buf + 7);
				switch (hbyte.type)
				{
				case MESSAGE:
					message_process(msg.buf + 2, msg.len - 3);
					break;
				case REQ:
					mpreq_type = *(msg.buf + 8);
					req_process(mpreq_type, mpower_id);
					break;
				case BURST:
					burst_process(msg.buf + 2, msg.len - 3, hbyte.ind);
					break;
				case ACK:
					mpack = *(msg.buf + 8);
					ack_process(mpack, mpower_id);
					break;
				default:
					break;
				}
			}
			else
			{
				NRF_LOG_INFO("checksum error");
			}
			freepoint(msg.buf);
		}
	}
}
