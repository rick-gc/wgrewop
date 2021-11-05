/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-09-29 16:07:54
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\platform.c
 */

#include "main.h"

sx126x_dev sx1268;
sx128x_dev sx1280_1;
sx128x_dev sx1280_2;

void gpio_irq_handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// NRF_LOG_INFO("gpio irq handle");
	if (pin == SX1268_DIO1_PIN)
	{
		xSemaphoreGiveFromISR(global.semphore_sx1268_dio_done, NULL);
		// NRF_LOG_INFO("SX1268_1_DIO1_PIN");
	}
	if (pin == SX1268_DIO2_PIN)
	{
		NRF_LOG_INFO("SX1268_1_DIO2_PIN");
	}
	if (pin == SX1280_1_DIO1_PIN)
	{
		xSemaphoreGiveFromISR(global.semphore_sx1280_1_dio1_done, NULL);
		// NRF_LOG_INFO("SX1280_1_DIO1_PIN");
	}
	if (pin == SX1280_1_DIO2_PIN)
	{
		xSemaphoreGiveFromISR(global.semphore_sx1280_1_dio1_done, NULL);
		// NRF_LOG_INFO("sx1280_1_DIO2_PIN");
	}
	if (pin == SX1280_2_DIO1_PIN)
	{
		xSemaphoreGiveFromISR(global.semphore_sx1280_2_dio1_done, NULL);
		// NRF_LOG_INFO("sx1280_2_DIO1_PIN");
	}
	if (pin == SX1280_2_DIO2_PIN)
	{
		xSemaphoreGiveFromISR(global.semphore_sx1280_2_dio1_done, NULL);
		// NRF_LOG_INFO("sx1280_2_DIO2_PIN");
	}
}

void platform_delay_ms(uint32_t nms)
{
	if (global.freertos_on == true)
	{
		vTaskDelay(nms * configTICK_RATE_HZ / 1000);
	}
	else
	{
		nrf_delay_ms(nms);
	}
}

/* SPI1 */
nrfx_spim_t m_spi_1 = NRFX_SPIM_INSTANCE(1);

nrfx_err_t spi1_init(void)
{
	nrfx_err_t ret_code = 0;

	/* Initialize spim1 instance */
	nrfx_spim_config_t spi1_config = NRFX_SPIM_DEFAULT_CONFIG;
	spi1_config.frequency = NRF_SPIM_FREQ_8M;
	spi1_config.ss_pin = NRFX_SPIM_PIN_NOT_USED;
	spi1_config.miso_pin = SPI1_MISO_PIN;
	spi1_config.mosi_pin = SPI1_MOSI_PIN;
	spi1_config.sck_pin = SPI1_SCK_PIN;
	spi1_config.mode = NRF_SPIM_MODE_0;
	ret_code = nrfx_spim_init(&m_spi_1, &spi1_config, spi1_event_handler, NULL);
	return ret_code;
}

void spi1_uninit(void)
{
	nrfx_spim_uninit(&m_spi_1);
}

void spi1_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
	BaseType_t xHigherPriorityTaskWoken;

	switch (p_event->type)
	{
	case NRFX_SPIM_EVENT_DONE:
		if (global.freertos_on)
		{
			xSemaphoreGiveFromISR(global.semaphore_spi1_xfer_complete,
								  &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		break;
	default:
		break;
	}
}

uint32_t spi1_write_buffer(const uint8_t *data, uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(data, len);
	result = nrfx_spim_xfer(&m_spi_1, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi1);
	}
	return result;
}

uint32_t spi1_read_buffer(uint8_t *data, uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_RX(data, len);
	result = nrfx_spim_xfer(&m_spi_1, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi1);
	}
	return result;
}

uint32_t spi1_write_read_buffer(const uint8_t *tx_data, uint8_t *rx_data,
								uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc =
		NRFX_SPIM_XFER_TRX(tx_data, len, rx_data, len);
	result = nrfx_spim_xfer(&m_spi_1, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi1_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi1);
	}
	return result;
}

/* SPI2 */
nrfx_spim_t m_spi_2 = NRFX_SPIM_INSTANCE(2);

nrfx_err_t spi2_init(void)
{
	nrfx_err_t ret_code = 0;

	/* Initialize spim1 instance */
	nrfx_spim_config_t spi2_config = NRFX_SPIM_DEFAULT_CONFIG;
	spi2_config.frequency = NRF_SPIM_FREQ_8M;
	spi2_config.ss_pin = NRFX_SPIM_PIN_NOT_USED;
	spi2_config.miso_pin = SPI2_MISO_PIN;
	spi2_config.mosi_pin = SPI2_MOSI_PIN;
	spi2_config.sck_pin = SPI2_SCK_PIN;
	spi2_config.mode = NRF_SPIM_MODE_0;
	ret_code = nrfx_spim_init(&m_spi_2, &spi2_config, spi2_event_handler, NULL);
	return ret_code;
}

void spi2_uninit(void)
{
	nrfx_spim_uninit(&m_spi_2);
}

void spi2_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
	BaseType_t xHigherPriorityTaskWoken;

	switch (p_event->type)
	{
	case NRFX_SPIM_EVENT_DONE:
		if (global.freertos_on)
		{
			xSemaphoreGiveFromISR(global.semaphore_spi2_xfer_complete,
								  &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		break;
	default:
		break;
	}
}

uint32_t spi2_write_buffer(const uint8_t *data, uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(data, len);
	result = nrfx_spim_xfer(&m_spi_2, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi2);
	}
	return result;
}

uint32_t spi2_read_buffer(uint8_t *data, uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_RX(data, len);
	result = nrfx_spim_xfer(&m_spi_2, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi2);
	}
	return result;
}

uint32_t spi2_write_read_buffer(const uint8_t *tx_data, uint8_t *rx_data,
								uint16_t len)
{
	uint8_t result = 0;
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
	}
	nrfx_spim_xfer_desc_t xfer_desc =
		NRFX_SPIM_XFER_TRX(tx_data, len, rx_data, len);
	result = nrfx_spim_xfer(&m_spi_2, &xfer_desc, 0);
	if (global.freertos_on)
	{
		xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
		xSemaphoreGive(global.semaphore_spi2);
	}
	return result;
}

void sx126x_wait_on_busy()
{
	while (nrf_gpio_pin_read(SX1268_BUSY_PIN) == 1)
	{
	}
}

void sx1268_cs_high()
{
	nrf_gpio_pin_set(SX1268_CS_PIN);
}

void sx1268_cs_low()
{
	nrf_gpio_pin_clear(SX1268_CS_PIN);
}

void sx1268_rst_high()
{
	nrf_gpio_pin_set(SX1268_RESET_PIN);
}

void sx1268_rst_low()
{
	nrf_gpio_pin_clear(SX1268_RESET_PIN);
}

void sx1268_ant_sw_tx()
{
}

void sx1268_ant_sw_rx()
{
}

void sx1268_tcxo_enable()
{
}

void sx1268_tcxo_disable()
{
}
void sx1268_gpio_init(void)
{
	nrf_gpio_cfg_output(SX1268_RESET_PIN);
	nrf_gpio_pin_set(SX1268_RESET_PIN);

	nrf_gpio_cfg_output(SX1268_PWRCTRL_PIN);
	nrf_gpio_pin_set(SX1268_PWRCTRL_PIN);

	nrf_gpio_cfg_output(SX1268_CS_PIN);
	nrf_gpio_pin_set(SX1268_CS_PIN);

	nrf_gpio_cfg_input(SX1268_BUSY_PIN, NRF_GPIO_PIN_NOPULL);

	nrfx_gpiote_in_config_t dio1_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	dio1_config.pull = GPIO_PIN_CNF_PULL_Pulldown;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1268_DIO1_PIN, &dio1_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1268_DIO1_PIN, true);

	nrfx_gpiote_in_config_t dio2_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	dio2_config.pull = GPIO_PIN_CNF_PULL_Pulldown;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1268_DIO2_PIN, &dio2_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1268_DIO2_PIN, true);
}

/* SX1280 CH1 */
void sx1280_ch1_gpio_init(void)
{

	nrf_gpio_cfg_output(SX1280_1_PWRKEY_PIN);
	nrf_gpio_cfg_output(SX1280_1_CS_PIN);
	nrf_gpio_cfg_output(SX1280_1_RESET_PIN);
	nrf_gpio_cfg_output(SX1280_1_TX_EN_PIN);
	nrf_gpio_cfg_output(SX1280_1_RX_EN_PIN);

	nrf_gpio_cfg_input(SX1280_1_BUSY_PIN, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_pin_set(SX1280_1_PWRKEY_PIN);
	nrf_gpio_pin_clear(SX1280_1_RESET_PIN);
	nrf_gpio_pin_set(SX1280_1_CS_PIN);
	nrf_gpio_pin_clear(SX1280_1_TX_EN_PIN);
	nrf_gpio_pin_clear(SX1280_1_RX_EN_PIN);

	nrfx_gpiote_in_config_t dio_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	dio_config.pull = GPIO_PIN_CNF_PULL_Pullup;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1280_1_DIO1_PIN, &dio_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1280_1_DIO1_PIN, true);
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1280_1_DIO2_PIN, &dio_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1280_1_DIO2_PIN, true);
}

void sx1280_ch1_wait_on_busy()
{
	while (nrf_gpio_pin_read(SX1280_1_BUSY_PIN) == 1)
		;
}

void sx1280_ch1_cs_high()
{
	nrf_gpio_pin_set(SX1280_1_CS_PIN);
}

void sx1280_ch1_cs_low()
{
	nrf_gpio_pin_clear(SX1280_1_CS_PIN);
}

void sx1280_ch1_rst_high()
{
	nrf_gpio_pin_set(SX1280_1_RESET_PIN);
}

void sx1280_ch1_rst_low()
{
	nrf_gpio_pin_clear(SX1280_1_RESET_PIN);
}

void sx1280_ch1_ant_sw_tx()
{
	nrf_gpio_pin_set(SX1280_1_TX_EN_PIN);
	nrf_gpio_pin_clear(SX1280_1_RX_EN_PIN);
}

void sx1280_ch1_ant_sw_rx()
{
	nrf_gpio_pin_clear(SX1280_1_TX_EN_PIN);
	nrf_gpio_pin_set(SX1280_1_RX_EN_PIN);
}

void sx1280_ch1_tcxo_enable()
{
	// dummy
}

void sx1280_ch1_tcxo_disable()
{
	// dummy
}

/* SX1280 CH2 */
void sx1280_ch2_gpio_init(void)
{
	nrf_gpio_cfg_output(SX1280_2_PWRKEY_PIN);
	nrf_gpio_cfg_output(SX1280_2_CS_PIN);
	nrf_gpio_cfg_output(SX1280_2_RESET_PIN);
	nrf_gpio_cfg_output(SX1280_2_TX_EN_PIN);
	nrf_gpio_cfg_output(SX1280_2_RX_EN_PIN);

	nrf_gpio_cfg_input(SX1280_2_BUSY_PIN, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_pin_set(SX1280_2_PWRKEY_PIN);
	nrf_gpio_pin_clear(SX1280_2_RESET_PIN);
	nrf_gpio_pin_set(SX1280_2_CS_PIN);
	nrf_gpio_pin_clear(SX1280_2_TX_EN_PIN);
	nrf_gpio_pin_clear(SX1280_2_RX_EN_PIN);

	nrfx_gpiote_in_config_t dio_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	dio_config.pull = GPIO_PIN_CNF_PULL_Pullup;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1280_2_DIO1_PIN, &dio_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1280_2_DIO1_PIN, true);
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX1280_2_DIO2_PIN, &dio_config, gpio_irq_handle));
	nrfx_gpiote_in_event_enable(SX1280_2_DIO2_PIN, true);
}

void sx1280_ch2_wait_on_busy()
{
	while (nrf_gpio_pin_read(SX1280_2_BUSY_PIN) == 1)
		;
}

void sx1280_ch2_cs_high()
{
	nrf_gpio_pin_set(SX1280_2_CS_PIN);
}

void sx1280_ch2_cs_low()
{
	nrf_gpio_pin_clear(SX1280_2_CS_PIN);
}

void sx1280_ch2_rst_high()
{
	nrf_gpio_pin_set(SX1280_2_RESET_PIN);
}

void sx1280_ch2_rst_low()
{
	nrf_gpio_pin_clear(SX1280_2_RESET_PIN);
}

void sx1280_ch2_ant_sw_tx()
{
	nrf_gpio_pin_set(SX1280_2_TX_EN_PIN);
	nrf_gpio_pin_clear(SX1280_2_RX_EN_PIN);
}

void sx1280_ch2_ant_sw_rx()
{
	nrf_gpio_pin_clear(SX1280_2_TX_EN_PIN);
	nrf_gpio_pin_set(SX1280_2_RX_EN_PIN);
}

void sx1280_ch2_tcxo_enable()
{
	// dummy
}

void sx1280_ch2_tcxo_disable()
{
	// dummy
}
