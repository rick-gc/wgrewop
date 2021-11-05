/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-10-14 14:55:36
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\custom_board\custom_board.h
 */

#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

//ec600
#define LTE_UART_TX_PIN                NRF_GPIO_PIN_MAP(1,14)
#define LTE_UART_RX_PIN                NRF_GPIO_PIN_MAP(1,15)
#define LTE_STATUS_PIN                 NRF_GPIO_PIN_MAP(0,28)
#define LTE_POWER_EN_PIN               NRF_GPIO_PIN_MAP(0,29)
#define LTE_PWRKEY_PIN                 NRF_GPIO_PIN_MAP(0,3)

// SPI2 Lora sx1268 use
#define SPI1_SCK_PIN 				   NRF_GPIO_PIN_MAP(0,4)
#define SPI1_MISO_PIN				   NRF_GPIO_PIN_MAP(0,27)
#define SPI1_MOSI_PIN				   NRF_GPIO_PIN_MAP(0,26)

// Lora sx1268
#define SX1268_PWRCTRL_PIN             NRF_GPIO_PIN_MAP(1,9)
#define SX1268_RESET_PIN               NRF_GPIO_PIN_MAP(1,8)
#define SX1268_BUSY_PIN                NRF_GPIO_PIN_MAP(0,5)

#define SX1268_DIO1_PIN                NRF_GPIO_PIN_MAP(0,7)
#define SX1268_DIO2_PIN                NRF_GPIO_PIN_MAP(0,6)
#define SX1268_CS_PIN                  NRF_GPIO_PIN_MAP(0,31)

// SPI1 Lora sx1280 use
#define SPI2_SCK_PIN 				   NRF_GPIO_PIN_MAP(0,22)
#define SPI2_MISO_PIN				   NRF_GPIO_PIN_MAP(0,24)
#define SPI2_MOSI_PIN				   NRF_GPIO_PIN_MAP(0,23)

// Lora sx1280
 // output pin
#define SX1280_1_PWRKEY_PIN            NRF_GPIO_PIN_MAP(0,13)
#define SX1280_1_CS_PIN                NRF_GPIO_PIN_MAP(0,12)
#define SX1280_1_RESET_PIN             NRF_GPIO_PIN_MAP(0,17)
#define SX1280_1_TX_EN_PIN             NRF_GPIO_PIN_MAP(0,19)
#define SX1280_1_RX_EN_PIN             NRF_GPIO_PIN_MAP(0,11)
 // input pin
#define SX1280_1_BUSY_PIN              NRF_GPIO_PIN_MAP(0,16)
#define SX1280_1_DIO1_PIN              NRF_GPIO_PIN_MAP(0,15)
#define SX1280_1_DIO2_PIN              NRF_GPIO_PIN_MAP(0,14)
 // output pin
#define SX1280_2_PWRKEY_PIN            NRF_GPIO_PIN_MAP(0,25)
#define SX1280_2_CS_PIN                NRF_GPIO_PIN_MAP(0,21)
#define SX1280_2_RESET_PIN             NRF_GPIO_PIN_MAP(1,1)
#define SX1280_2_TX_EN_PIN             NRF_GPIO_PIN_MAP(1,2)
#define SX1280_2_RX_EN_PIN             NRF_GPIO_PIN_MAP(0,20)
 // input pin
#define SX1280_2_BUSY_PIN              NRF_GPIO_PIN_MAP(0,10)
#define SX1280_2_DIO1_PIN              NRF_GPIO_PIN_MAP(0,9)
#define SX1280_2_DIO2_PIN              NRF_GPIO_PIN_MAP(1,0)

//wdt
#define WDT_PIN                        NRF_GPIO_PIN_MAP(1,06)

//adc
#define ADC_PIN                        NRF_GPIO_PIN_MAP(0,8)

#endif /* CUSTOM_BOARD_H */