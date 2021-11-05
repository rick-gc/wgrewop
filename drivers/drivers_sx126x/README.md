# SX126x驱动说明

## 初始化
```
sx126x_dev sx1268;
device_id = SX1268_DEVICE_ID;
sx1268.read_buffer = spi2_read_buffer; // SPI读取函数，要求是同步操作
sx1268.write_buffer = spi2_write_buffer; // SPI写入函数，要求是同步操作
sx1268.write_read_buffer = spi2_write_read_buffer; // SPI写入读取函数，要求是同步操作
sx1268.delay_ms = platform_delay_ms; // 延迟函数
sx1268.wait_on_busy = sx126x_wait_on_busy; // 检查SX126x BUSY引脚是否已经为低电平，见下方
sx1268.cs_high = sx1268_cs_high; // SX1268 SPI CS引脚设置为高电平函数
sx1268.cs_low = sx1268_cs_low; // SX1268 SPI CS引脚设置为低电平函数
sx1268.rst_high = sx1268_rst_high; // SX1268 RST引脚设置为高电平函数
sx1268.rst_low = sx1268_rst_low; // SX1268 RST引脚设置为低电平函数
sx1268.ant_sw_tx = sx1268_ant_sw_tx; // SX1268 控制外部射频开关为发射
sx1268.ant_sw_rx = sx1268_ant_sw_rx; // SX1268 控制外部射频开关为接收
sx1268.tcxo_enable = sx1268_tcxo_enable; // SX1268 外部TCXO供电开启
sx1268.tcxo_disable = sx1268_tcxo_disable; // SX1268 外部TCXO供电关闭

sx1268.tx_power = 22; // 发射功率
sx1268.symble_timeout = 5; 
sx1268.rx_continuous = true; // 是否连续接收
sx1268.callback.rxDone = lora_rx_done; // 接收完成回调函数
sx1268.callback.txDone = lora_tx_done; // 发送完成回调函数
sx1268.callback.txTimeout = lora_tx_timeout; // 发送超时回调函数

sx1268.modulation_params.PacketType = PACKET_TYPE_LORA; // 传输类型为LORA
sx1268.modulation_params.Params.LoRa.Bandwidth = LORA_BW_250; // 带宽设置为250KHz
sx1268.modulation_params.Params.LoRa.CodingRate = LORA_CR_4_5; // CR=4/5
sx1268.modulation_params.Params.LoRa.LowDatarateOptimize = 0; // 低速率优化开关
sx1268.modulation_params.Params.LoRa.SpreadingFactor = LORA_SF7; // SF=7

sx1268.packet_params.PacketType = PACKET_TYPE_LORA;
sx1268.packet_params.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH; // 包含Header，Header中带有数据长度
sx1268.packet_params.Params.LoRa.CrcMode = LORA_CRC_ON; // CRC校验打开
sx1268.packet_params.Params.LoRa.PayloadLength = 40; // PayLoad长度
sx1268.packet_params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL; // IQ 配置
sx1268.packet_params.Params.LoRa.PreambleLength = 6; // 前导码长度

sx126x_init(&sx1268);
```

### 典型实现
```
nrfx_spim_t m_spi_2 = NRFX_SPIM_INSTANCE(2);

void spi2_event_handler (nrfx_spim_evt_t const *p_event, void *p_context)
{
  BaseType_t xHigherPriorityTaskWoken;

  switch (p_event->type)
  {
    case NRF_DRV_SPI_EVENT_DONE:
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

uint32_t spi2_write_buffer (const uint8_t *data, uint16_t len)
{
  uint8_t result = 0;
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(data, len);
  result = nrfx_spim_xfer (&m_spi_2, &xfer_desc, 0);
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
    xSemaphoreGive(global.semaphore_spi2);
  }
  return result;
}

uint32_t spi2_read_buffer (uint8_t *data, uint16_t len)
{
  uint8_t result = 0;
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
  }
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_RX(data, len);
  result = nrfx_spim_xfer (&m_spi_2, &xfer_desc, 0);
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
    xSemaphoreGive(global.semaphore_spi2);
  }
  return result;
}

uint32_t spi2_write_read_buffer (const uint8_t *tx_data, uint8_t *rx_data,
                                 uint16_t len)
{
  uint8_t result = 0;
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2, portMAX_DELAY);
  }
  nrfx_spim_xfer_desc_t xfer_desc =
      NRFX_SPIM_XFER_TRX(tx_data, len, rx_data, len);
  result = nrfx_spim_xfer (&m_spi_2, &xfer_desc, 0);
  if (global.freertos_on)
  {
    xSemaphoreTake(global.semaphore_spi2_xfer_complete, portMAX_DELAY);
    xSemaphoreGive(global.semaphore_spi2);
  }
  return result;
}

void platform_delay_ms (uint32_t nms)
{
  if (global.freertos_on == true)
  {
    vTaskDelay (nms * configTICK_RATE_HZ / 1000);
  }
  else
  {
    nrf_delay_ms (nms);
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

void sx1268_ant_sw_tx()
{
  nrf_gpio_pin_set(SX1268_ANT_SW);
}

void sx1268_ant_sw_rx()
{
  nrf_gpio_pin_clear(SX1268_ANT_SW);
}

void sx1268_gpio_init( void )
{
    nrf_gpio_cfg_output(SX1268_RESET_PIN);
    nrf_gpio_pin_set(SX1268_RESET_PIN);

    nrf_gpio_cfg_output(SX1268_TCXOEN_PIN);
    nrf_gpio_pin_set(SX1268_TCXOEN_PIN);

    nrf_gpio_cfg_output(SX1268_CS_PIN);
    nrf_gpio_pin_set(SX1268_CS_PIN);

    nrf_gpio_cfg_output(SX1268_ANT_SW);
    nrf_gpio_pin_clear(SX1268_ANT_SW);

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

void sx126x_wait_on_busy()
{
    while(nrf_gpio_pin_read(SX1268_BUSY_PIN) == 1 );
}

```

## 进入StandBy
```
// 进入RC方式StandBy
sx126x_set_standby(&sx1268, STDBY_RC);
// 进入XOSC方式StandBy，如果用的TCXO要求给TCXO使能供电，前面有tcxo_enable和tcxo_disable的配置函数
sx126x_set_standby(&sx1268, STDBY_XOSC);
```

## 进入Sleep
```
// 进入sleep，下次唤醒为warm start，速度较快
sx126x_set_sleep_warm_start(&sx1268);
// 进入sleep，下次唤醒为cold start，速度相对较慢
sx126x_set_sleep_cold_start(&sx1268);
```

## 设置频点
```
// 第二个参数为频率，单位为Hz
sx126x_set_rf_frequency (&sx1268, 470377000);
```

## 发送方式1
```
// 第二个参数为要发送的数据报文内容
// 第三个参数为要发送的数据长度
// 第四个参数为要发送的数据超时时间，单位为ms，如果不希望有超时，那么该参数为0
void sx126x_tx (sx126x_dev *dev, uint8_t *data, uint16_t len,
                  uint32_t timeout_ms);
```

## 发送方式2
```
// 本方法为把要发送的数据填写到FIFO中
// 第二个参数为要发送的数据报文内容
// 第三个参数为要发送的数据长度
void sx126x_tx_set_payload (sx126x_dev *dev, uint8_t *data, uint16_t len);

// 本方法为要发送的数据进行提交，触发发送
// 第二个参数为要发送的数据超时时间，单位为ms，如果不希望有超时，那么该参数为0
void sx126x_tx_commit(sx126x_dev *dev, uint32_t timeout_ms);
```

## 接收
```
// 第二个参数为要接收数据超时时间，仅single模式有效，连续接收模式下该参数无效，单位为ms，如果不希望有超时，那么该参数为0xFFFFFF
sx126x_rx(&sx1268, uint32_t timeout_ms);
```
## 调用需要调用的函数

```
// 用来实现DIO1的中断后处理，相应如下中断：IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT | IRQ_TX_DONE，不要放在ISR中调用
sx126x_dio1_process(&sx1268);

// 用来实现DIO2的中断后处理，相应如下中断：IRQ_HEADER_VALID | IRQ_HEADER_ERROR | IRQ_PREAMBLE_DETECTED，不要放在ISR中调用
sx126x_dio2_process(&sx1268);

// 用来实现DIO2的中断后处理，暂未有作用
sx126x_dio3_process(&sx1268);
```

### 典型参考代码
```
void gpio_irq_handle (nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  msg_t msg;

  msg.tick = xTaskGetTickCountFromISR ();
  if (pin == SX1268_DIO1_PIN)
  {
    xQueueSendFromISR(global.queue_lora_dio1_msg, &msg, NULL);
  }
  else if (pin == SX1268_DIO2_PIN)
  {
    xQueueSendFromISR(global.queue_lora_dio2_msg, &msg, NULL);
  }
}

void task_lora_dio1 ()
{
  msg_t msg;
  char debug_str[64] =
  { 0 };

  while (true)
  {
    if (pdPASS
        == xQueueReceive (global.queue_lora_dio1_msg, &msg, portMAX_DELAY))
    {
      sprintf (debug_str, "sx1268_dio1_process: %u", msg.tick);
      SEGGER_SYSVIEW_Print (debug_str);
      global.sx1268.dio1_tick = msg.tick;
      sx126x_dio1_process (&global.sx1268);
    }
  }
}

void task_lora_dio2 ()
{
  msg_t msg;
  char debug_str[64] =
  { 0 };

  while (true)
  {
    if (pdPASS
        == xQueueReceive (global.queue_lora_dio2_msg, &msg, portMAX_DELAY))
    {
      sprintf (debug_str, "sx1268_dio2_process: %u", msg.tick);
      SEGGER_SYSVIEW_Print (debug_str);
      global.sx1268.dio2_tick = msg.tick;
      sx126x_dio2_process (&global.sx1268);
    }
  }
}
```