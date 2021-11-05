/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 15:31:04
 * @LastEditTime : 2021-10-14 15:18:41
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\saadc.c
 */

#include "main.h"

un_2u8_u16 voltage;
static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];

void saadc_sample()
{
	nrfx_saadc_sample();
	vTaskDelay(usticks(1000));
	nrfx_saadc_sample();
	vTaskDelay(usticks(1000));
	nrfx_saadc_sample();
	vTaskDelay(usticks(1000));
}

void saadc_callback(nrfx_saadc_evt_t const *p_event)
{
	if (p_event->type == NRFX_SAADC_EVT_DONE)
	{
		ret_code_t err_code;

		err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);

		uint16_t buf = 0;

		for (uint16_t i = 0; i < SAMPLES_IN_BUFFER; i++)
		{
			buf += p_event->data.done.p_buffer[i];
		}
		voltage.b = buf * 1000 / SAMPLES_IN_BUFFER / 1024 * 3 / 5 * 6 * 6;
		NRF_LOG_INFO("voltage:%d", voltage.b);
		xSemaphoreGiveFromISR(global.semphore_saadc_sample_done, NULL);
	}
}

void saadc_init(void)
{
	ret_code_t err_code;
	nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
	nrf_saadc_channel_config_t channel_config =
		NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

	err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_buffer_convert(m_buffer_pool, SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);

	nrf_gpio_cfg_output(ADC_PIN);
	nrf_gpio_pin_clear(ADC_PIN);
}
