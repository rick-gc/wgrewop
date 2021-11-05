/*!
 * \file      sx126x.c
 *
 * \brief     SX126x driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <string.h>
#include "sx126x.h"

/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   ( double )32000000
#define FREQ_DIV                                    ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                                   ( double )( XTAL_FREQ / FREQ_DIV )

/*!
 * \brief Holds the internal operating mode of the radio
 */

void sx126x_reset (sx126x_dev *dev)
{
  dev->rst_low ();
  dev->delay_ms (10);
  dev->rst_high ();
}

bool sx126x_init (sx126x_dev *dev)
{
  uint8_t tmp_reg = 0;

  sx126x_reset (dev);

  //SX126xIoIrqInit( dioIrq );

  sx126x_wakeup (dev);
  sx126x_set_standby (dev, STDBY_RC);

  tmp_reg = sx126x_read_register (dev, REG_LR_SYNCWORD);
  if (tmp_reg != 0x14)
  {
    return false;
  }
  
  sx126x_set_dio2_as_rf_switch_ctrl ( dev, dev->dio2_as_rf_switch_ctrl_flag );

  sx126x_set_regulator_mode (dev, dev->regulator_mode);
  // Initialize TCXO control
  if (dev->tcxo_enable)
  {
    dev->tcxo_enable ();
  }
  // Force image calibration
  dev->image_calibrated = false;

  sx126x_set_buffer_base_address (dev, 0x00, 0x00);

  sx126x_set_tx_params (dev, dev->tx_power, RADIO_RAMP_200_US);

  sx126x_set_dio_irq_params (dev, IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE,
                             IRQ_RADIO_NONE);

  sx126x_set_stop_rx_timer_on_preamble_detect (dev, false);

  if (PACKET_TYPE_GFSK == dev->modulation_params.PacketType)
  {
    sx126x_set_sync_word (dev, ( uint8_t[]
        )
        { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 });
    sx126x_set_whitening_seed (dev, 0x01FF);
  }

  sx126x_set_modulation_params (dev, &dev->modulation_params);

  sx126x_set_packet_params (dev, &dev->packet_params);

  if (dev->rx_continuous)
  {
    dev->symble_timeout = 0;
  }

  if (PACKET_TYPE_LORA == dev->modulation_params.PacketType)
  {
    sx126x_set_lora_symb_num_timeout (dev, dev->symble_timeout);
    // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
    if (LORA_IQ_INVERTED == dev->packet_params.Params.LoRa.InvertIQ)
    {
      // RegIqPolaritySetup = @address 0x0736
      sx126x_write_register (dev, 0x0736,
                             sx126x_read_register (dev, 0x0736) & ~(1 << 2));
    }
    else
    {
      // RegIqPolaritySetup @address 0x0736
      sx126x_write_register (dev, 0x0736,
                             sx126x_read_register (dev, 0x0736) | (1 << 2));
    }
    // WORKAROUND END
  }
  return true;
}

void sx126x_tx (sx126x_dev *dev, uint8_t *data, uint16_t len,
                uint32_t timeout_ms)
{
  sx126x_set_dio_irq_params (dev, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                             IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE,
                             IRQ_RADIO_NONE);

  PacketParams_t packet_params;
  memcpy (&packet_params, &dev->packet_params, sizeof(PacketParams_t));

  if (dev->packet_type == PACKET_TYPE_LORA)
  {
    packet_params.Params.LoRa.PayloadLength = len;
  }
  else
  {
    packet_params.Params.Gfsk.PayloadLength = len;
  }

  sx126x_set_packet_params (dev, &packet_params);

  sx126x_send_payload (dev, data, len, timeout_ms * 1000 * 1000 / 15625);
}

void sx126x_tx_set_payload (sx126x_dev *dev, uint8_t *data, uint16_t len)
{
  sx126x_set_dio_irq_params (dev, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                             IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE,
                             IRQ_RADIO_NONE);

  PacketParams_t packet_params;
  memcpy (&packet_params, &dev->packet_params, sizeof(PacketParams_t));

  if (dev->packet_type == PACKET_TYPE_LORA)
  {
    packet_params.Params.LoRa.PayloadLength = len;
  }
  else
  {
    packet_params.Params.Gfsk.PayloadLength = len;
  }

  sx126x_set_packet_params (dev, &packet_params);

  sx126x_set_payload (dev, data, len);
}

void sx126x_tx_commit(sx126x_dev *dev, uint32_t timeout_ms)
{
  sx126x_set_tx (dev, timeout_ms * 1000 * 1000 / 15625);
}

void sx126x_rx (sx126x_dev *dev, uint32_t timeout_ms)
{
  sx126x_set_dio_irq_params (
      dev,
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_VALID
          | IRQ_HEADER_ERROR | IRQ_PREAMBLE_DETECTED,
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
      IRQ_RADIO_NONE,
      IRQ_RADIO_NONE);

  sx126x_set_packet_params (dev, &dev->packet_params);

  if (dev->rx_continuous == true)
  {
    sx126x_set_rx (dev, 0xFFFFFF); // Rx Continuous
  }
  else
  {
    sx126x_set_rx (dev, timeout_ms * 1000 * 1000 / 15625);
  }
}

void sx126x_check_device_ready (sx126x_dev *dev)
{
  if ((dev->op_mode == MODE_SLEEP) || (dev->op_mode == MODE_RX_DC))
  {
    sx126x_wakeup (dev);
    // Switch is turned off when device is in sleep mode and turned on is all other modes
    //SX126xAntSwOn( );  //dont need  YP
  }
  dev->wait_on_busy ();
}

void sx126x_set_payload (sx126x_dev *dev, uint8_t *payload, uint8_t size)
{
  sx126x_write_buffer (dev, 0x00, payload, size);
}

uint8_t sx126x_get_payload (sx126x_dev *dev, uint8_t *buffer, uint8_t *size,
                            uint8_t maxSize)
{
  uint8_t offset = 0;

  sx126x_get_rx_buffer_status (dev, size, &offset);
  if (*size > maxSize)
  {
    return 1;
  }
  sx126x_read_buffer (dev, offset, buffer, *size);
  return 0;
}

void sx126x_send_payload (sx126x_dev *dev, uint8_t *payload, uint8_t size,
                          uint32_t timeout)
{
  sx126x_set_payload (dev, payload, size);
  sx126x_set_tx (dev, timeout);
}

uint8_t sx126x_set_sync_word (sx126x_dev *dev, uint8_t *syncWord)
{
  sx126x_write_registers (dev, REG_LR_SYNCWORDBASEADDRESS, syncWord, 8);
  return 0;
}

void sx126x_set_crc_seed (sx126x_dev *dev, uint16_t seed)
{
  uint8_t buf[2];

  buf[0] = (uint8_t) ((seed >> 8) & 0xFF);
  buf[1] = (uint8_t) (seed & 0xFF);

  switch (sx126x_get_packet_type (dev))
  {
    case PACKET_TYPE_GFSK:
      sx126x_write_registers (dev, REG_LR_CRCSEEDBASEADDR, buf, 2);
      break;

    default:
      break;
  }
}

void sx126x_set_crc_polynomial (sx126x_dev *dev, uint16_t polynomial)
{
  uint8_t buf[2];

  buf[0] = (uint8_t) ((polynomial >> 8) & 0xFF);
  buf[1] = (uint8_t) (polynomial & 0xFF);

  switch (sx126x_get_packet_type (dev))
  {
    case PACKET_TYPE_GFSK:
      sx126x_write_registers (dev, REG_LR_CRCPOLYBASEADDR, buf, 2);
      break;

    default:
      break;
  }
}

void sx126x_set_whitening_seed (sx126x_dev *dev, uint16_t seed)
{
  uint8_t regValue = 0;

  switch (sx126x_get_packet_type (dev))
  {
    case PACKET_TYPE_GFSK:
      regValue = sx126x_read_register (dev, REG_LR_WHITSEEDBASEADDR_MSB) & 0xFE;
      regValue = ((seed >> 8) & 0x01) | regValue;
      sx126x_write_register (dev, REG_LR_WHITSEEDBASEADDR_MSB, regValue); // only 1 bit.
      sx126x_write_register (dev, REG_LR_WHITSEEDBASEADDR_LSB, (uint8_t) seed);
      break;

    default:
      break;
  }
}

uint32_t sx126x_get_random (sx126x_dev *dev)
{
  uint32_t number = 0;
  uint8_t regAnaLna = 0;
  uint8_t regAnaMixer = 0;

  regAnaLna = sx126x_read_register (dev, REG_ANA_LNA);
  sx126x_write_register (dev, REG_ANA_LNA, regAnaLna & ~(1 << 0));

  regAnaMixer = sx126x_read_register (dev, REG_ANA_MIXER);
  sx126x_write_register (dev, REG_ANA_MIXER, regAnaMixer & ~(1 << 7));

  // Set radio in continuous reception
  sx126x_set_rx (dev, 0xFFFFFF); // Rx Continuous

  sx126x_read_registers (dev, RANDOM_NUMBER_GENERATORBASEADDR,
                         (uint8_t*) &number, 4);

  sx126x_set_standby (dev, STDBY_RC);

  sx126x_write_register (dev, REG_ANA_LNA, regAnaLna);
  sx126x_write_register (dev, REG_ANA_MIXER, regAnaMixer);

  return number;
}

void sx126x_set_sleep_warm_start (sx126x_dev *dev)
{
  //SX126xAntSwOff( );//YP
  SleepParams_t params =
  { 0 };

  params.Fields.WarmStart = 1;

  sx126x_set_sleep (dev, params);
}

void sx126x_set_sleep_cold_start (sx126x_dev *dev)
{
  //SX126xAntSwOff( );//YP
  SleepParams_t params =
  { 0 };

  params.Fields.WarmStart = 0;

  sx126x_set_sleep (dev, params);
}

void sx126x_set_sleep (sx126x_dev *dev, SleepParams_t sleepConfig)
{
  uint8_t value = (((uint8_t) sleepConfig.Fields.WarmStart << 2)
      | ((uint8_t) sleepConfig.Fields.Reset << 1)
      | ((uint8_t) sleepConfig.Fields.WakeUpRTC));
  sx126x_write_command (dev, RADIO_SET_SLEEP, &value, 1);
  dev->op_mode = MODE_SLEEP;
}

void sx126x_set_standby (sx126x_dev *dev, RadioStandbyModes_t standbyConfig)
{
  sx126x_write_command (dev, RADIO_SET_STANDBY, (uint8_t*) &standbyConfig, 1);
  if (standbyConfig == STDBY_RC)
  {
    dev->op_mode = MODE_STDBY_RC;
  }
  else
  {
    dev->op_mode = MODE_STDBY_XOSC;
  }
}

void sx126x_set_fs (sx126x_dev *dev)
{
  sx126x_write_command (dev, RADIO_SET_FS, 0, 0);
  dev->op_mode = MODE_FS;
}

void sx126x_set_tx (sx126x_dev *dev, uint32_t timeout)
{
  uint8_t buf[3];

  dev->op_mode = MODE_TX;

  buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t) (timeout & 0xFF);
  if (dev->ant_sw_tx)
  {
    dev->ant_sw_tx ();
  }
  sx126x_write_command (dev, RADIO_SET_TX, buf, 3);
}

void sx126x_set_rx (sx126x_dev *dev, uint32_t timeout)
{
  uint8_t buf[3];

  dev->op_mode = MODE_RX;

  buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t) (timeout & 0xFF);
  if (dev->ant_sw_rx)
  {
    dev->ant_sw_rx ();
  }
  sx126x_write_command (dev, RADIO_SET_RX, buf, 3);
}

void sx126x_set_rx_boosted (sx126x_dev *dev, uint32_t timeout)
{
  uint8_t buf[3];

  dev->op_mode = MODE_RX;

  sx126x_write_register (dev, REG_RX_GAIN, 0x96); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

  buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t) (timeout & 0xFF);
  sx126x_write_command (dev, RADIO_SET_RX, buf, 3);
}

void sx126x_set_rx_duty_cycle (sx126x_dev *dev, uint32_t rxTime,
                               uint32_t sleepTime)
{
  uint8_t buf[6];

  buf[0] = (uint8_t) ((rxTime >> 16) & 0xFF);
  buf[1] = (uint8_t) ((rxTime >> 8) & 0xFF);
  buf[2] = (uint8_t) (rxTime & 0xFF);
  buf[3] = (uint8_t) ((sleepTime >> 16) & 0xFF);
  buf[4] = (uint8_t) ((sleepTime >> 8) & 0xFF);
  buf[5] = (uint8_t) (sleepTime & 0xFF);
  sx126x_write_command (dev, RADIO_SET_RXDUTYCYCLE, buf, 6);
  dev->op_mode = MODE_RX_DC;
}

void sx126x_set_cad (sx126x_dev *dev)
{
  sx126x_write_command (dev, RADIO_SET_CAD, 0, 0);
  dev->op_mode = MODE_CAD;
}

void sx126x_set_tx_continuous_wave (sx126x_dev *dev)
{
  sx126x_write_command (dev, RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
}

void sx126x_set_tx_infinite_preamble (sx126x_dev *dev)
{
  sx126x_write_command (dev, RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
}

void sx126x_set_stop_rx_timer_on_preamble_detect (sx126x_dev *dev, bool enable)
{
  sx126x_write_command (dev, RADIO_SET_STOPRXTIMERONPREAMBLE,
                        (uint8_t*) &enable, 1);
}

void sx126x_set_lora_symb_num_timeout (sx126x_dev *dev, uint8_t SymbNum)
{
  sx126x_write_command (dev, RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1);
}

void sx126x_set_regulator_mode (sx126x_dev *dev, RadioRegulatorMode_t mode)
{
  sx126x_write_command (dev, RADIO_SET_REGULATORMODE, (uint8_t*) &mode, 1);
  dev->regulator_mode = mode;
}

void sx126x_calibrate (sx126x_dev *dev, CalibrationParams_t calibParam)
{
  uint8_t value = (((uint8_t) calibParam.Fields.ImgEnable << 6)
      | ((uint8_t) calibParam.Fields.ADCBulkPEnable << 5)
      | ((uint8_t) calibParam.Fields.ADCBulkNEnable << 4)
      | ((uint8_t) calibParam.Fields.ADCPulseEnable << 3)
      | ((uint8_t) calibParam.Fields.PLLEnable << 2)
      | ((uint8_t) calibParam.Fields.RC13MEnable << 1)
      | ((uint8_t) calibParam.Fields.RC64KEnable));

  sx126x_write_command (dev, RADIO_CALIBRATE, &value, 1);
}

void sx126x_calibrate_image (sx126x_dev *dev, uint32_t freq)
{
  uint8_t calFreq[2];

  if (freq > 900000000)
  {
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
  }
  else if (freq > 850000000)
  {
    calFreq[0] = 0xD7;
    calFreq[1] = 0xDB;
  }
  else if (freq > 770000000)
  {
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
  }
  else if (freq > 460000000)
  {
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
  }
  else if (freq > 425000000)
  {
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
  }
  else if (freq > 350000000)
  {
    calFreq[0] = 0x57;
    calFreq[1] = 0x65;
  }
  sx126x_write_command (dev, RADIO_CALIBRATEIMAGE, calFreq, 2);
}

void sx126x_set_pa_config (sx126x_dev *dev, uint8_t paDutyCycle, uint8_t hpMax,
                           uint8_t deviceSel, uint8_t paLut)
{
  uint8_t buf[4];

  buf[0] = paDutyCycle;
  buf[1] = hpMax;
  buf[2] = deviceSel;
  buf[3] = paLut;
  sx126x_write_command (dev, RADIO_SET_PACONFIG, buf, 4);
}

void sx126x_set_rx_tx_fallback_mode (sx126x_dev *dev, uint8_t fallbackMode)
{
  sx126x_write_command (dev, RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1);
}

void sx126x_set_dio_irq_params (sx126x_dev *dev, uint16_t irqMask,
                                uint16_t dio1Mask, uint16_t dio2Mask,
                                uint16_t dio3Mask)
{
  uint8_t buf[8];

  buf[0] = (uint8_t) ((irqMask >> 8) & 0x00FF);
  buf[1] = (uint8_t) (irqMask & 0x00FF);
  buf[2] = (uint8_t) ((dio1Mask >> 8) & 0x00FF);
  buf[3] = (uint8_t) (dio1Mask & 0x00FF);
  buf[4] = (uint8_t) ((dio2Mask >> 8) & 0x00FF);
  buf[5] = (uint8_t) (dio2Mask & 0x00FF);
  buf[6] = (uint8_t) ((dio3Mask >> 8) & 0x00FF);
  buf[7] = (uint8_t) (dio3Mask & 0x00FF);
  sx126x_write_command (dev, RADIO_CFG_DIOIRQ, buf, 8);
}

uint16_t sx126x_get_irq_status (sx126x_dev *dev)
{
  uint8_t irqStatus[2];

  sx126x_read_command (dev, RADIO_GET_IRQSTATUS, irqStatus, 2);
  return (irqStatus[0] << 8) | irqStatus[1];
}

void sx126x_set_dio2_as_rf_switch_ctrl (sx126x_dev *dev, uint8_t enable)
{
  sx126x_write_command (dev, RADIO_SET_RFSWITCHMODE, &enable, 1);
}

void sx126x_set_dio3_as_tcxo_ctrl (sx126x_dev *dev,
                                   RadioTcxoCtrlVoltage_t tcxoVoltage,
                                   uint32_t timeout)
{
  uint8_t buf[4];

  buf[0] = tcxoVoltage & 0x07;
  buf[1] = (uint8_t) ((timeout >> 16) & 0xFF);
  buf[2] = (uint8_t) ((timeout >> 8) & 0xFF);
  buf[3] = (uint8_t) (timeout & 0xFF);

  sx126x_write_command (dev, RADIO_SET_TCXOMODE, buf, 4);
}

void sx126x_set_rf_frequency (sx126x_dev *dev, uint32_t frequency)
{
  uint8_t buf[4];
  uint32_t freq = 0;

  if ((dev->image_calibrated == true) || (dev->calibrated_freq == frequency))
  {
    // Do nothing
    return;
  }
  else if ((dev->image_calibrated == false)
      || (dev->calibrated_freq != frequency))
  {
    sx126x_calibrate_image (dev, frequency);
    dev->image_calibrated = true;
    dev->calibrated_freq = frequency;
  }

  freq = (uint32_t) ((double) frequency / (double) FREQ_STEP );
  buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
  buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
  buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
  buf[3] = (uint8_t) (freq & 0xFF);
  sx126x_write_command (dev, RADIO_SET_RFFREQUENCY, buf, 4);
}

void sx126x_set_packet_type (sx126x_dev *dev, RadioPacketTypes_t packetType)
{
  // Save packet type internally to avoid questioning the radio
  dev->packet_type = packetType;
  sx126x_write_command (dev, RADIO_SET_PACKETTYPE, (uint8_t*) &packetType, 1);
}

RadioPacketTypes_t sx126x_get_packet_type (sx126x_dev *dev)
{
  return dev->packet_type;
}

void sx126x_set_tx_params (sx126x_dev *dev, int8_t power,
                           RadioRampTimes_t rampTime)
{
  uint8_t buf[2];

  if (dev->device_id == SX1261_DEVICE_ID)
  {
    if (power == 15)
    {
      sx126x_set_pa_config (dev, 0x06, 0x00, 0x01, 0x01);
    }
    else
    {
      sx126x_set_pa_config (dev, 0x04, 0x00, 0x01, 0x01);
    }
    if (power >= 14)
    {
      power = 14;
    }
    else if (power < -17)
    {
      power = -17;
    }
    sx126x_write_register (dev, REG_OCP, 0x18); // current max is 80 mA for the whole device
  }
  else if (dev->device_id == SX1262_DEVICE_ID) // sx1262
  {
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    sx126x_write_register (dev, 0x08D8,
                           sx126x_read_register (dev, 0x08D8) | (0x0F << 1));
    // WORKAROUND END

    sx126x_set_pa_config (dev, 0x04, 0x07, 0x00, 0x01);
    if (power > 22)
    {
      power = 22;
    }
    else if (power < -9)
    {
      power = -9;
    }
    sx126x_write_register (dev, REG_OCP, 0x38); // current max 160mA for the whole device
  }
  else  // sx1268
  {
#if 1        
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    sx126x_write_register (dev, 0x08D8,
                           sx126x_read_register (dev, 0x08D8) | (0x0F << 1));
    // WORKAROUND END
#endif        
    sx126x_set_pa_config (dev, 0x04, 0x07, 0x00, 0x01);

    if (power > 22)
    {
      power = 22;
    }
    else if (power < -9)
    {
      power = -9;
    }
    sx126x_write_register (dev, REG_OCP, 0x38); // current max 160mA for the whole device

  }
  buf[0] = power;
  buf[1] = (uint8_t) rampTime;
  sx126x_write_command (dev, RADIO_SET_TXPARAMS, buf, 2);
}

void sx126x_set_modulation_params (sx126x_dev *dev,
                                   ModulationParams_t *modulationParams)
{
  uint8_t n;
  uint32_t tempVal = 0;
  uint8_t buf[8] =
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  // Check if required configuration corresponds to the stored packet type
  // If not, silently update radio packet type
  if (dev->packet_type != modulationParams->PacketType)
  {
    sx126x_set_packet_type (dev, modulationParams->PacketType);
  }

  switch (modulationParams->PacketType)
  {
    case PACKET_TYPE_GFSK:
      n = 8;
      tempVal =
          (uint32_t) (32
              * ((double) XTAL_FREQ
                  / (double) modulationParams->Params.Gfsk.BitRate));
      buf[0] = (tempVal >> 16) & 0xFF;
      buf[1] = (tempVal >> 8) & 0xFF;
      buf[2] = tempVal & 0xFF;
      buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
      buf[4] = modulationParams->Params.Gfsk.Bandwidth;
      tempVal = (uint32_t) ((double) modulationParams->Params.Gfsk.Fdev
          / (double) FREQ_STEP );
      buf[5] = (tempVal >> 16) & 0xFF;
      buf[6] = (tempVal >> 8) & 0xFF;
      buf[7] = (tempVal & 0xFF);
      sx126x_write_command (dev, RADIO_SET_MODULATIONPARAMS, buf, n);
      break;
    case PACKET_TYPE_LORA:
      n = 4;
      buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
      buf[1] = modulationParams->Params.LoRa.Bandwidth;
      buf[2] = modulationParams->Params.LoRa.CodingRate;
      buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

      sx126x_write_command (dev, RADIO_SET_MODULATIONPARAMS, buf, n);

      break;
    default:
    case PACKET_TYPE_NONE:
      return;
  }
}

void sx126x_set_packet_params (sx126x_dev *dev, PacketParams_t *packetParams)
{
  uint8_t n;
  uint8_t crcVal = 0;
  uint8_t buf[9] =
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  // Check if required configuration corresponds to the stored packet type
  // If not, silently update radio packet type
  if (dev->packet_type != packetParams->PacketType)
  {
    sx126x_set_packet_type (dev, packetParams->PacketType);
  }

  switch (packetParams->PacketType)
  {
    case PACKET_TYPE_GFSK:
      if (packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM)
      {
        sx126x_set_crc_seed (dev, CRC_IBM_SEED);
        sx126x_set_crc_polynomial (dev, CRC_POLYNOMIAL_IBM);
        crcVal = RADIO_CRC_2_BYTES;
      }
      else if (packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT)
      {
        sx126x_set_crc_seed (dev, CRC_CCITT_SEED);
        sx126x_set_crc_polynomial (dev, CRC_POLYNOMIAL_CCITT);
        crcVal = RADIO_CRC_2_BYTES_INV;
      }
      else
      {
        crcVal = packetParams->Params.Gfsk.CrcLength;
      }
      n = 9;
      buf[0] = (packetParams->Params.Gfsk.PreambleLength >> 8) & 0xFF;
      buf[1] = packetParams->Params.Gfsk.PreambleLength;
      buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
      buf[3] = (packetParams->Params.Gfsk.SyncWordLength /*<< 3*/); // convert from byte to bit
      buf[4] = packetParams->Params.Gfsk.AddrComp;
      buf[5] = packetParams->Params.Gfsk.HeaderType;
      buf[6] = packetParams->Params.Gfsk.PayloadLength;
      buf[7] = crcVal;
      buf[8] = packetParams->Params.Gfsk.DcFree;
      break;
    case PACKET_TYPE_LORA:
      n = 6;
      buf[0] = (packetParams->Params.LoRa.PreambleLength >> 8) & 0xFF;
      buf[1] = packetParams->Params.LoRa.PreambleLength;
      buf[2] = dev->lora_header_type = packetParams->Params.LoRa.HeaderType;
      buf[3] = packetParams->Params.LoRa.PayloadLength;
      buf[4] = packetParams->Params.LoRa.CrcMode;
      buf[5] = packetParams->Params.LoRa.InvertIQ;
      break;
    default:
    case PACKET_TYPE_NONE:
      return;
  }
  sx126x_write_command (dev, RADIO_SET_PACKETPARAMS, buf, n);
}

void sx126x_set_cad_params (sx126x_dev *dev, RadioLoRaCadSymbols_t cadSymbolNum,
                            uint8_t cadDetPeak, uint8_t cadDetMin,
                            RadioCadExitModes_t cadExitMode,
                            uint32_t cadTimeout)
{
  uint8_t buf[7];

  buf[0] = (uint8_t) cadSymbolNum;
  buf[1] = cadDetPeak;
  buf[2] = cadDetMin;
  buf[3] = (uint8_t) cadExitMode;
  buf[4] = (uint8_t) ((cadTimeout >> 16) & 0xFF);
  buf[5] = (uint8_t) ((cadTimeout >> 8) & 0xFF);
  buf[6] = (uint8_t) (cadTimeout & 0xFF);
  sx126x_write_command (dev, RADIO_SET_CADPARAMS, buf, 7);
  dev->op_mode = MODE_CAD;
}

void sx126x_set_buffer_base_address (sx126x_dev *dev, uint8_t txBaseAddress,
                                     uint8_t rxBaseAddress)
{
  uint8_t buf[2];

  buf[0] = txBaseAddress;
  buf[1] = rxBaseAddress;
  sx126x_write_command (dev, RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

RadioStatus_t sx126x_get_status (sx126x_dev *dev)
{
  uint8_t stat = 0;
  RadioStatus_t status =
  { .Value = 0 };

  stat = sx126x_read_command (dev, RADIO_GET_STATUS, NULL, 0);
  status.Fields.CmdStatus = (stat & (0x07 << 1)) >> 1;
  status.Fields.ChipMode = (stat & (0x07 << 4)) >> 4;
  return status;
}

int8_t sx126x_get_rssi_inst (sx126x_dev *dev)
{
  uint8_t buf[1];
  int8_t rssi = 0;

  sx126x_read_command (dev, RADIO_GET_RSSIINST, buf, 1);
  rssi = -buf[0] >> 1;
  return rssi;
}

void sx126x_get_rx_buffer_status (sx126x_dev *dev, uint8_t *payloadLength,
                                  uint8_t *rxStartBufferPointer)
{
  uint8_t status[2];

  sx126x_read_command (dev, RADIO_GET_RXBUFFERSTATUS, status, 2);

  // In case of LORA fixed header, the payloadLength is obtained by reading
  // the register REG_LR_PAYLOADLENGTH
  if ((sx126x_get_packet_type (dev) == PACKET_TYPE_LORA)
      && (dev->lora_header_type == LORA_PACKET_FIXED_LENGTH))
  {
    *payloadLength = sx126x_read_register (dev, REG_LR_PAYLOADLENGTH);
  }
  else
  {
    *payloadLength = status[0];
  }
  *rxStartBufferPointer = status[1];
}

void sx126x_get_packet_status (sx126x_dev *dev, PacketStatus_t *pktStatus)
{
  uint8_t status[3];

  sx126x_read_command (dev, RADIO_GET_PACKETSTATUS, status, 3);

  pktStatus->packetType = sx126x_get_packet_type (dev);
  switch (pktStatus->packetType)
  {
    case PACKET_TYPE_GFSK:
      pktStatus->Params.Gfsk.RxStatus = status[0];
      pktStatus->Params.Gfsk.RssiSync = -status[1] >> 1;
      pktStatus->Params.Gfsk.RssiAvg = -status[2] >> 1;
      pktStatus->Params.Gfsk.FreqError = 0;
      break;

    case PACKET_TYPE_LORA:
      pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
      // Returns SNR value [dB] rounded to the nearest integer value
      pktStatus->Params.LoRa.SnrPkt = (((int8_t) status[1]) + 2) >> 2;
      pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
      pktStatus->Params.LoRa.FreqError = dev->frequency_error;
      break;

    default:
    case PACKET_TYPE_NONE:
      // In that specific case, we set everything in the pktStatus to zeros
      // and reset the packet type accordingly
      memset (pktStatus, 0, sizeof(PacketStatus_t));
      pktStatus->packetType = PACKET_TYPE_NONE;
      break;
  }
}

RadioError_t sx126x_get_device_errors (sx126x_dev *dev)
{
  uint8_t err[] =
  { 0, 0 };
  RadioError_t error =
  { .Value = 0 };

  sx126x_read_command (dev, RADIO_GET_ERROR, (uint8_t*) err, 2);
  error.Fields.PaRamp = (err[0] & (1 << 0)) >> 0;
  error.Fields.PllLock = (err[1] & (1 << 6)) >> 6;
  error.Fields.XoscStart = (err[1] & (1 << 5)) >> 5;
  error.Fields.ImgCalib = (err[1] & (1 << 4)) >> 4;
  error.Fields.AdcCalib = (err[1] & (1 << 3)) >> 3;
  error.Fields.PllCalib = (err[1] & (1 << 2)) >> 2;
  error.Fields.Rc13mCalib = (err[1] & (1 << 1)) >> 1;
  error.Fields.Rc64kCalib = (err[1] & (1 << 0)) >> 0;
  return error;
}

void sx126x_clear_device_errors (sx126x_dev *dev)
{
  uint8_t buf[2] =
  { 0x00, 0x00 };
  sx126x_write_command (dev, RADIO_CLR_ERROR, buf, 2);
}

void sx126x_clear_irq_status (sx126x_dev *dev, uint16_t irq)
{
  uint8_t buf[2];

  buf[0] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
  buf[1] = (uint8_t) ((uint16_t) irq & 0x00FF);
  sx126x_write_command (dev, RADIO_CLR_IRQSTATUS, buf, 2);
}

void sx126x_write_command (sx126x_dev *dev, RadioCommands_t command,
                           uint8_t *buffer, uint16_t size)
{
  uint16_t i;
  uint8_t tx_buf[260] =
  { 0 };

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = (uint8_t) command;
  memcpy (tx_buf + 1, buffer, size);

  dev->write_buffer (tx_buf, size + 1);

  dev->cs_high ();

  if (command != RADIO_SET_SLEEP)
  {
    dev->wait_on_busy ();
  }
}

uint8_t debugStatus = 0;

uint8_t sx126x_read_command (sx126x_dev *dev, RadioCommands_t command,
                             uint8_t *buffer, uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };
  uint8_t rx_buf[260] =
  { 0 };

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = (uint8_t) command;

  dev->write_read_buffer (tx_buf, rx_buf, size + 2);

  memcpy (buffer, rx_buf + 2, size);

  dev->cs_high ();

  dev->wait_on_busy ();

  return rx_buf[1];
}

void sx126x_write_registers (sx126x_dev *dev, uint16_t address, uint8_t *buffer,
                             uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  /* 地址结构 */
  tx_buf[0] = RADIO_WRITE_REGISTER;
  tx_buf[1] = (address & 0xFF00) >> 8;
  tx_buf[2] = address & 0x00FF;
  memcpy (tx_buf + 3, buffer, size);

  dev->write_buffer (tx_buf, size + 3);

  dev->cs_high ();

  dev->wait_on_busy ();
}

void sx126x_write_register (sx126x_dev *dev, uint16_t address, uint8_t value)
{
  sx126x_write_registers (dev, address, &value, 1);
}

void sx126x_read_registers (sx126x_dev *dev, uint16_t address, uint8_t *buffer,
                            uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };
  uint8_t rx_buf[260] =
  { 0 };

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  /* 地址结构 */
  tx_buf[0] = RADIO_READ_REGISTER;
  tx_buf[1] = (address & 0xFF00) >> 8;
  tx_buf[2] = address & 0x00FF;

  dev->write_read_buffer (tx_buf, rx_buf, size + 4);
  memcpy (buffer, rx_buf + 4, size);

  dev->cs_high ();

  dev->wait_on_busy ();
}

uint8_t sx126x_read_register (sx126x_dev *dev, uint16_t address)
{
  uint8_t data = 0;

  sx126x_read_registers (dev, address, &data, 1);
  return data;
}

void sx126x_write_buffer (sx126x_dev *dev, uint8_t offset, uint8_t *buffer,
                          uint8_t size)
{
  uint8_t tx_buf[260] =
  { 0 };

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = RADIO_WRITE_BUFFER;
  tx_buf[1] = offset;
  memcpy (tx_buf + 2, buffer, size);
  dev->write_buffer (tx_buf, size + 2);

  dev->cs_high ();

  dev->wait_on_busy ();
}

void sx126x_read_buffer (sx126x_dev *dev, uint8_t offset, uint8_t *buffer,
                         uint8_t size)
{
  uint16_t i;
  uint8_t tx_buf[260];
  uint8_t rx_buf[260];

  sx126x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = RADIO_READ_BUFFER;
  tx_buf[1] = offset;

  dev->write_read_buffer (tx_buf, rx_buf, size + 3);

  dev->cs_high ();

  memcpy (buffer, rx_buf + 3, size);

  dev->wait_on_busy ();
}

void sx126x_set_rf_tx_power (sx126x_dev *dev, int8_t power)
{
  sx126x_set_tx_params (dev, power, RADIO_RAMP_40_US);
}

void sx126x_wakeup (sx126x_dev *dev)
{
  uint8_t tx_buf[2] =
  { RADIO_GET_STATUS, 0 };

  dev->cs_low ();
  dev->write_buffer (tx_buf, 2);
  dev->cs_high ();

  // Wait for chip to be ready.
  dev->wait_on_busy ();
}

void sx126x_dio1_process (sx126x_dev *dev)
{
  volatile uint16_t irq_status = 0;

  irq_status = sx126x_get_irq_status (dev);
  sx126x_clear_irq_status (
      dev, (IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT | IRQ_TX_DONE)); // This handler only handle these IRQ

  if (MODE_RX == dev->op_mode)
  {
    if (PACKET_TYPE_LORA == dev->modulation_params.PacketType)
    {
      if (dev->rx_continuous == false)
      {
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        dev->op_mode = MODE_STDBY_RC;
      }

      if ((irq_status & IRQ_RX_DONE) == IRQ_RX_DONE)
      {
        if ((irq_status & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
        {
          if ((dev != NULL) && (dev->callback.rxError != NULL))
          {
            dev->callback.rxError (IRQ_CRC_ERROR_CODE);
          }
        }
        else
        {
          //To Do check it need or not for SX1268!!!
          if (dev->rx_continuous == false)
          {
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            dev->op_mode = MODE_STDBY_RC;

            // WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
            // RegRtcControl = @address 0x0902
            sx126x_write_register (dev, 0x0902, 0x00);
            // RegEventMask = @address 0x0944
            sx126x_write_register (
                dev, 0x0944, sx126x_read_register (dev, 0x0944) | (1 << 1));
            // WORKAROUND END
          }

          memset (dev->radio_rx_payload, 0, sizeof(dev->radio_rx_payload));
          sx126x_get_payload (dev, dev->radio_rx_payload,
                              &dev->radio_rx_payload_len, 255);
          sx126x_get_packet_status (dev, &dev->packet_status);
          dev->rx_done_tick = dev->dio1_tick;
          if ((dev != NULL) && (dev->callback.rxDone != NULL))
          {
            dev->callback.rxDone (dev->radio_rx_payload,
                                  dev->radio_rx_payload_len,
                                  dev->packet_status.Params.LoRa.RssiPkt,
                                  dev->packet_status.Params.LoRa.SnrPkt);
          }

        }
      }

      if ((irq_status & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
      {
        if ((dev != NULL) && (dev->callback.rxTimeout != NULL))
        {
          dev->callback.rxTimeout ();
        }
      }


    }
    else if (PACKET_TYPE_GFSK == dev->modulation_params.PacketType)
    {
      //TODO
    }

  }
  else if (MODE_TX == dev->op_mode)
  {
    dev->op_mode = MODE_STDBY_RC;

    if ((irq_status & IRQ_TX_DONE) == IRQ_TX_DONE)
    {
      dev->tx_done_tick = dev->dio1_tick;
      if ((dev != NULL) && (dev->callback.txDone != NULL))
      {
        dev->callback.txDone ();
      }
    }

    if ((irq_status & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
    {
      if ((dev != NULL) && (dev->callback.txTimeout != NULL))
      {
        dev->callback.txTimeout ();
      }
    }
  }
}

void sx126x_dio2_process (sx126x_dev *dev)
{
  volatile uint16_t irq_status = 0;
  bool channelActivityDetected;

  irq_status = sx126x_get_irq_status (dev);
  sx126x_clear_irq_status (
      dev, (IRQ_HEADER_VALID | IRQ_HEADER_ERROR | IRQ_PREAMBLE_DETECTED)); //This handler only handle these IRQ

  if (PACKET_TYPE_LORA == dev->modulation_params.PacketType)
  {

    if ((irq_status & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED)
    {
      if ((dev != NULL) && (dev->callback.rxPreambleDetect != NULL))
      {
        dev->callback.rxPreambleDetect ();
      }
    }

    if ((irq_status & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID)
    {
      if ((dev != NULL) && (dev->callback.rxSyncWordDone != NULL))
      {
        dev->callback.rxSyncWordDone ();
      }
    }

    if ((irq_status & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
    {
      if ((dev != NULL) && (dev->callback.rxHeaderDone != NULL))
      {
        dev->callback.rxHeaderDone (true);
      }
    }

    if ((irq_status & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR)
    {
      if (dev->rx_continuous == false)
      {
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        dev->op_mode = MODE_STDBY_RC;
      }
      if ((dev != NULL) && (dev->callback.rxHeaderDone != NULL))
      {
        dev->callback.rxHeaderDone (false);
      }
    }

    if ((irq_status & IRQ_CAD_DONE) == IRQ_CAD_DONE)
    {

      if ((irq_status & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED)
      {
        channelActivityDetected = true;
      }
      else
      {
        channelActivityDetected = false;
      }

      if ((dev != NULL) && (dev->callback.cadDone != NULL))
      {
        dev->callback.cadDone (channelActivityDetected);
      }
    }

  }
  else if (PACKET_TYPE_GFSK == dev->modulation_params.PacketType)
  {
    //TODO
  }

}

void sx126x_dio3_process (sx126x_dev *dev)
{
  //TODO
}

