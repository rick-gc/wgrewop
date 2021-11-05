/*
 * @Author       : yfwang
 * @Date         : 2021-09-16 14:40:19
 * @LastEditTime : 2021-09-28 02:46:03
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\sx1280_platform.c
 */

#include "main.h"

SX128X_TickTime_t sx1280TickTime;

void sx1280_user_init(void)
{
	sx1280_1.read_buffer = spi2_read_buffer;			 // SPI读取函数，要求是同步操作
	sx1280_1.write_buffer = spi2_write_buffer;			 // SPI写入函数，要求是同步操作
	sx1280_1.write_read_buffer = spi2_write_read_buffer; // SPI写入读取函数，要求是同步操作
	sx1280_1.delay_ms = platform_delay_ms;				 // 延迟函数
	sx1280_1.wait_on_busy = sx1280_ch1_wait_on_busy;	 // 检查SX1280 BUSY引脚是否已经为低电平，见下方
	sx1280_1.cs_high = sx1280_ch1_cs_high;				 // SX1280 SPI CS引脚设置为高电平函数
	sx1280_1.cs_low = sx1280_ch1_cs_low;				 // SX1280 SPI CS引脚设置为低电平函数
	sx1280_1.rst_high = sx1280_ch1_rst_high;			 // SX1280 RST引脚设置为高电平函数
	sx1280_1.rst_low = sx1280_ch1_rst_low;				 // SX1280 RST引脚设置为低电平函数
	sx1280_1.ant_sw_tx = sx1280_ch1_ant_sw_tx;			 // SX1280 控制外部射频开关为发射
	sx1280_1.ant_sw_rx = sx1280_ch1_ant_sw_rx;			 // SX1280 控制外部射频开关为接收
	sx1280_1.tcxo_enable = sx1280_ch1_tcxo_enable;		 // SX1280 外部TCXO供电开启
	sx1280_1.tcxo_disable = sx1280_ch1_tcxo_disable;	 // SX1280 外部TCXO供电关闭

	sx1280_1.tx_power = 13;								  // 发射功率
	// sx1280_1.callbacks.rxDone = sx1280_ch1_rx_done;		  // 接收完成回调函数
	// sx1280_1.callbacks.txDone = sx1280_ch1_tx_done;		  // 发送完成回调函数
	// sx1280_1.callbacks.txTimeout = sx1280_ch1_tx_timeout; // 发送超时回调函数

	sx1280_1.regulator_mode = SX128X_USE_DCDC;
	sx1280_1.modulation_params.PacketType = SX128X_PACKET_TYPE_LORA;		  // 传输类型为LORA
	sx1280_1.modulation_params.Params.LoRa.Bandwidth = SX128X_LORA_BW_0800;	  // 带宽设置为800KHz
	sx1280_1.modulation_params.Params.LoRa.CodingRate = SX128X_LORA_CR_4_5;	  // CR=4/5
	sx1280_1.modulation_params.Params.LoRa.SpreadingFactor = SX128X_LORA_SF8; // SF=7

	sx1280_1.packet_params.PacketType = SX128X_PACKET_TYPE_LORA;
	sx1280_1.packet_params.Params.LoRa.HeaderType = SX128X_LORA_PACKET_VARIABLE_LENGTH; // 包含Header，Header中带有数据长度
	sx1280_1.packet_params.Params.LoRa.CrcMode = SX128X_LORA_CRC_ON;					// CRC校验打开
	sx1280_1.packet_params.Params.LoRa.PayloadLength = 10;								// PayLoad长度
	sx1280_1.packet_params.Params.LoRa.InvertIQ = SX128X_LORA_IQ_NORMAL;				// IQ 配置
	sx1280_1.packet_params.Params.LoRa.PreambleLength = 0x13;							// 前导码长度

	sx128x_init(&sx1280_1);

    sx1280_2.read_buffer = spi2_read_buffer;			 // SPI读取函数，要求是同步操作
	sx1280_2.write_buffer = spi2_write_buffer;			 // SPI写入函数，要求是同步操作
	sx1280_2.write_read_buffer = spi2_write_read_buffer; // SPI写入读取函数，要求是同步操作
	sx1280_2.delay_ms = platform_delay_ms;				 // 延迟函数
	sx1280_2.wait_on_busy = sx1280_ch2_wait_on_busy;	 // 检查SX1280 BUSY引脚是否已经为低电平，见下方
	sx1280_2.cs_high = sx1280_ch2_cs_high;				 // SX1280 SPI CS引脚设置为高电平函数
	sx1280_2.cs_low = sx1280_ch2_cs_low;				 // SX1280 SPI CS引脚设置为低电平函数
	sx1280_2.rst_high = sx1280_ch2_rst_high;			 // SX1280 RST引脚设置为高电平函数
	sx1280_2.rst_low = sx1280_ch2_rst_low;				 // SX1280 RST引脚设置为低电平函数
	sx1280_2.ant_sw_tx = sx1280_ch2_ant_sw_tx;			 // SX1280 控制外部射频开关为发射
	sx1280_2.ant_sw_rx = sx1280_ch2_ant_sw_rx;			 // SX1280 控制外部射频开关为接收
	sx1280_2.tcxo_enable = sx1280_ch2_tcxo_enable;		 // SX1280 外部TCXO供电开启
	sx1280_2.tcxo_disable = sx1280_ch2_tcxo_disable;	 // SX1280 外部TCXO供电关闭

	sx1280_2.tx_power = 13;								  // 发射功率
	// sx1280_2.callbacks.rxDone = sx1280_ch1_rx_done;		  // 接收完成回调函数
	// sx1280_2.callbacks.txDone = sx1280_ch1_tx_done;		  // 发送完成回调函数
	// sx1280_2.callbacks.txTimeout = sx1280_ch1_tx_timeout; // 发送超时回调函数

	sx1280_2.regulator_mode = SX128X_USE_DCDC;
	sx1280_2.modulation_params.PacketType = SX128X_PACKET_TYPE_LORA;		  // 传输类型为LORA
	sx1280_2.modulation_params.Params.LoRa.Bandwidth = SX128X_LORA_BW_0800;	  // 带宽设置为800KHz
	sx1280_2.modulation_params.Params.LoRa.CodingRate = SX128X_LORA_CR_4_5;	  // CR=4/5
	sx1280_2.modulation_params.Params.LoRa.SpreadingFactor = SX128X_LORA_SF8; // SF=7

	sx1280_2.packet_params.PacketType = SX128X_PACKET_TYPE_LORA;
	sx1280_2.packet_params.Params.LoRa.HeaderType = SX128X_LORA_PACKET_VARIABLE_LENGTH; // 包含Header，Header中带有数据长度
	sx1280_2.packet_params.Params.LoRa.CrcMode = SX128X_LORA_CRC_ON;					// CRC校验打开
	sx1280_2.packet_params.Params.LoRa.PayloadLength = 10;								// PayLoad长度
	sx1280_2.packet_params.Params.LoRa.InvertIQ = SX128X_LORA_IQ_NORMAL;				// IQ 配置
	sx1280_2.packet_params.Params.LoRa.PreambleLength = 0x13;							// 前导码长度

	sx128x_init(&sx1280_2);
}
