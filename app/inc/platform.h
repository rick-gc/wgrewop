/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-09-28 00:30:56
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\platform.h
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

typedef enum
{
    LPOWER_SLAVE,
    LPOWER_MASTER,
    LPOWER_RFU,
    MPOWER_DATA,
    MPOWER_CONTROL
}node_device;

extern sx126x_dev sx1268;
extern sx128x_dev sx1280_1;
extern sx128x_dev sx1280_2;

void gpio_irq_handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/* COMMON */
void platform_delay_ms(uint32_t nms);

/* SPI1 */
nrfx_err_t spi1_init(void);
void spi1_uninit(void);
void spi1_event_handler(nrfx_spim_evt_t const *p_event, void *p_context);
uint32_t spi1_read_buffer(uint8_t *data, uint16_t len);
uint32_t spi1_write_buffer(const uint8_t *data, uint16_t len);
uint32_t spi1_write_read_buffer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/* SPI2 */
nrfx_err_t spi2_init(void);
void spi2_uninit(void);
void spi2_event_handler(nrfx_spim_evt_t const *p_event, void *p_context);
uint32_t spi2_read_buffer(uint8_t *data, uint16_t len);
uint32_t spi2_write_buffer(const uint8_t *data, uint16_t len);
uint32_t spi2_write_read_buffer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/* SX1268 */
void sx126x_wait_on_busy();
void sx1268_cs_high();
void sx1268_cs_low();
void sx1268_rst_high();
void sx1268_rst_low();
void sx1268_ant_sw_tx();
void sx1268_ant_sw_rx();
void sx1268_tcxo_enable();
void sx1268_tcxo_disable();
void sx1268_gpio_init();

/* SX1280 */
void sx1280_ch1_gpio_init();
void sx1280_ch1_wait_on_busy();
void sx1280_ch1_cs_high();
void sx1280_ch1_cs_low();
void sx1280_ch1_rst_high();
void sx1280_ch1_rst_low();
void sx1280_ch1_ant_sw_tx();
void sx1280_ch1_ant_sw_rx();
void sx1280_ch1_tcxo_enable();
void sx1280_ch1_tcxo_disable();
void sx1280_ch1_gpio_init();

void sx1280_ch2_gpio_init();
void sx1280_ch2_wait_on_busy();
void sx1280_ch2_cs_high();
void sx1280_ch2_cs_low();
void sx1280_ch2_rst_high();
void sx1280_ch2_rst_low();
void sx1280_ch2_ant_sw_tx();
void sx1280_ch2_ant_sw_rx();
void sx1280_ch2_tcxo_enable();
void sx1280_ch2_tcxo_disable();

#endif /* _PLATFORM_H_ */