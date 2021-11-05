/*
 ______                              _
 / _____)             _              | |
 ( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
 (______/|_____)_|_|_| \__)_____)\____)_| |_|
 (C)2016 Semtech

 Description: Driver for SX1280 devices

 License: Revised BSD License, see LICENSE.TXT file include in the project

 Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
 */
#include <string.h>
#include <stdio.h>
#include <sx128x.h>
#include "RangingCorrection.h"

#ifndef DISABLE_IRQ
#define DISABLE_IRQ()
#endif

#ifndef ENABLE_IRQ
#define ENABLE_IRQ()
#endif

/*!
 * \brief Radio registers definition
 *
 */
typedef struct
{
  uint16_t Addr;                             //!< The address of the register
  uint8_t Value;                            //!< The value of the register
} SX128X_RadioRegisters_t;

/*!
 * \brief Radio hardware registers initialization definition
 */
// { Address, RegValue }
#define SX128X_RADIO_INIT_REGISTERS_VALUE  { NULL }

/*!
 * \brief Radio hardware registers initialization
 */
const SX128X_RadioRegisters_t RadioRegsInit[] =
{ SX128X_RADIO_INIT_REGISTERS_VALUE };


void sx128x_on_dio_irq (sx128x_dev *dev);

/*!
 * \brief Holds a flag raised on radio interrupt
 */

int32_t sx128x_complement2 (const uint32_t num, const uint8_t bitCnt)
{
  int32_t retVal = (int32_t) num;
  if (num >= 2 << (bitCnt - 2))
  {
    retVal -= 2 << (bitCnt - 1);
  }
  return retVal;
}

void sx128x_reset (sx128x_dev *dev)
{
  dev->rst_low ();
  dev->delay_ms (10);
  dev->rst_high ();
}

bool sx128x_init (sx128x_dev *dev)
{
  uint8_t tmp_reg = 0;
  uint16_t firmware_version;

  sx128x_reset (dev);

  sx128x_wakeup (dev);
  sx128x_set_standby (dev, SX128X_STDBY_RC);

  sx128x_set_regulator_mode (dev, dev->regulator_mode);
  // Initialize TCXO control
  if (dev->tcxo_enable)
  {
    dev->tcxo_enable ();
  }

  // 收发FIFO使用同一个起始地址
  sx128x_set_buffer_base_addresses (dev, 0x00, 0x00);

  sx128x_set_tx_params (dev, dev->tx_power, SX128X_RADIO_RAMP_20_US);

  // DIO中断使能配置
  sx128x_set_dio_irq_params (dev, SX128X_IRQ_RADIO_ALL, SX128X_IRQ_RADIO_NONE,
                             SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE,
                             SX128X_IRQ_RADIO_NONE);

  sx128x_set_modulation_params (dev, &dev->modulation_params);

  sx128x_set_packet_params (dev, &dev->packet_params);

  return true;
}

void sx128x_set_registers_default (sx128x_dev *dev)
{
  for (uint16_t i = 0;
      i < sizeof(RadioRegsInit) / sizeof(SX128X_RadioRegisters_t); i++)
  {
    sx128x_write_register (dev, RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
  }
}

uint16_t sx128x_get_firmware_version (sx128x_dev *dev)
{
  return (((sx128x_read_register (dev, SX128X_REG_LR_FIRMWARE_VERSION_MSB))
      << 8)
      | (sx128x_read_register (dev, SX128X_REG_LR_FIRMWARE_VERSION_MSB + 1)));
}

SX128X_RadioStatus_t sx128x_get_status (sx128x_dev *dev)
{
  uint8_t stat = 0;
  SX128X_RadioStatus_t status;

  sx128x_read_command (dev, SX128X_RADIO_GET_STATUS, (uint8_t*) &stat, 1);
  status.Value = stat;
  return status;
}

void sx128x_set_sleep (sx128x_dev *dev, SX128X_SleepParams_t sleepConfig)
{
  uint8_t sleep = (sleepConfig.WakeUpRTC << 3)
      | (sleepConfig.InstructionRamRetention << 2)
      | (sleepConfig.DataBufferRetention << 1) | (sleepConfig.DataRamRetention);

  dev->op_mode = SX128X_MODE_SLEEP;
  sx128x_write_command (dev, SX128X_RADIO_SET_SLEEP, &sleep, 1);
}

void sx128x_set_standby (sx128x_dev *dev,
                         SX128X_RadioStandbyModes_t standbyConfig)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_STANDBY,
                         (uint8_t*) &standbyConfig, 1);
  if (standbyConfig == SX128X_STDBY_RC)
  {
    dev->op_mode = SX128X_MODE_STDBY_RC;
  }
  else
  {
    dev->op_mode = SX128X_MODE_STDBY_XOSC;
  }
}

void sx128x_set_fs (sx128x_dev *dev)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_FS, 0, 0);
  dev->op_mode = SX128X_MODE_FS;
}

void sx128x_set_tx (sx128x_dev *dev, SX128X_TickTime_t timeout)
{
  uint8_t buf[3];
  buf[0] = timeout.Step;
  buf[1] = (uint8_t) ((timeout.NbSteps >> 8) & 0x00FF);
  buf[2] = (uint8_t) (timeout.NbSteps & 0x00FF);

  sx128x_clear_irq_status (dev, SX128X_IRQ_RADIO_ALL);

  // If the radio is doing ranging operations, then apply the specific calls
  // prior to SetTx
  if (sx128x_get_packet_type (dev) == SX128X_PACKET_TYPE_RANGING)
  {
    sx128x_set_ranging_role (dev, SX128X_RADIO_RANGING_ROLE_MASTER);
  }
  sx128x_write_command (dev, SX128X_RADIO_SET_TX, buf, 3);
  dev->op_mode = SX128X_MODE_TX;
}

void sx128x_set_rx (sx128x_dev *dev, SX128X_TickTime_t timeout)
{
  uint8_t buf[3];
  buf[0] = timeout.Step;
  buf[1] = (uint8_t) ((timeout.NbSteps >> 8) & 0x00FF);
  buf[2] = (uint8_t) (timeout.NbSteps & 0x00FF);

  sx128x_clear_irq_status (dev, SX128X_IRQ_RADIO_ALL);

  // If the radio is doing ranging operations, then apply the specific calls
  // prior to SetRx
  if (sx128x_get_packet_type (dev) == SX128X_PACKET_TYPE_RANGING)
  {
    sx128x_set_ranging_role (dev, SX128X_RADIO_RANGING_ROLE_SLAVE);
  }
  sx128x_write_command (dev, SX128X_RADIO_SET_RX, buf, 3);
  dev->op_mode = SX128X_MODE_RX;
}

void sx128x_set_rx_duty_cycle (sx128x_dev *dev, SX128X_RadioTickSizes_t Step,
                               uint16_t NbStepRx, uint16_t RxNbStepSleep)
{
  uint8_t buf[5];

  buf[0] = Step;
  buf[1] = (uint8_t) ((NbStepRx >> 8) & 0x00FF);
  buf[2] = (uint8_t) (NbStepRx & 0x00FF);
  buf[3] = (uint8_t) ((RxNbStepSleep >> 8) & 0x00FF);
  buf[4] = (uint8_t) (RxNbStepSleep & 0x00FF);
  sx128x_write_command (dev, SX128X_RADIO_SET_RXDUTYCYCLE, buf, 5);
  dev->op_mode = SX128X_MODE_RX;
}

void sx128x_set_cad (sx128x_dev *dev)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_CAD, 0, 0);
  dev->op_mode = SX128X_MODE_CAD;
}

void sx128x_set_tx_continuous_wave (sx128x_dev *dev)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
}

void sx128x_set_tx_continuous_preamble (sx128x_dev *dev)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
}

void sx128x_set_packet_type (sx128x_dev *dev,
                             SX128X_RadioPacketTypes_t packetType)
{
  // Save packet type internally to avoid questioning the radio
  dev->packet_type = packetType;

  sx128x_write_command (dev, SX128X_RADIO_SET_PACKETTYPE,
                         (uint8_t*) &packetType, 1);
}

SX128X_RadioPacketTypes_t sx128x_get_packet_type (sx128x_dev *dev)
{
  return dev->packet_type;
}

void sx128x_set_rf_frequency (sx128x_dev *dev, uint32_t frequency)
{
  uint8_t buf[3];
  uint32_t freq = 0;

  freq = (uint32_t) ((double) frequency / (double) SX128X_FREQ_STEP);
  buf[0] = (uint8_t) ((freq >> 16) & 0xFF);
  buf[1] = (uint8_t) ((freq >> 8) & 0xFF);
  buf[2] = (uint8_t) (freq & 0xFF);
  sx128x_write_command (dev, SX128X_RADIO_SET_RFFREQUENCY, buf, 3);
}

void sx128x_set_tx_params (sx128x_dev *dev, int8_t power,
                           SX128X_RadioRampTimes_t rampTime)
{
  uint8_t buf[2];

  // The power value to send on SPI/UART is in the range [0..31] and the
  // physical output power is in the range [-18..13]dBm
  buf[0] = power + 18;
  buf[1] = (uint8_t) rampTime;
  sx128x_write_command (dev, SX128X_RADIO_SET_TXPARAMS, buf, 2);
}

void sx128x_set_cad_params (sx128x_dev *dev,
                            SX128X_RadioLoRaCadSymbols_t cadSymbolNum)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_CADPARAMS,
                         (uint8_t*) &cadSymbolNum, 1);
  dev->op_mode = SX128X_MODE_CAD;
}

void sx128x_set_buffer_base_addresses (sx128x_dev *dev, uint8_t txBaseAddress,
                                       uint8_t rxBaseAddress)
{
  uint8_t buf[2];

  buf[0] = txBaseAddress;
  buf[1] = rxBaseAddress;
  sx128x_write_command (dev, SX128X_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx128x_set_modulation_params (sx128x_dev *dev,
                                   SX128X_ModulationParams_t *modulationParams)
{
  uint8_t buf[3];

  // Check if required configuration corresponds to the stored packet type
  // If not, silently update radio packet type
  if (dev->packet_type != modulationParams->PacketType)
  {
    sx128x_set_packet_type (dev, modulationParams->PacketType);
  }

  switch (modulationParams->PacketType)
  {
    case SX128X_PACKET_TYPE_GFSK:
      buf[0] = modulationParams->Params.Gfsk.BitrateBandwidth;
      buf[1] = modulationParams->Params.Gfsk.ModulationIndex;
      buf[2] = modulationParams->Params.Gfsk.ModulationShaping;
      break;

    case SX128X_PACKET_TYPE_LORA:
    case SX128X_PACKET_TYPE_RANGING:
      buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
      buf[1] = modulationParams->Params.LoRa.Bandwidth;
      buf[2] = modulationParams->Params.LoRa.CodingRate;
      dev->lora_bandwidth = modulationParams->Params.LoRa.Bandwidth;
      break;

    case SX128X_PACKET_TYPE_FLRC:
      buf[0] = modulationParams->Params.Flrc.BitrateBandwidth;
      buf[1] = modulationParams->Params.Flrc.CodingRate;
      buf[2] = modulationParams->Params.Flrc.ModulationShaping;
      break;

    case SX128X_PACKET_TYPE_BLE:
      buf[0] = modulationParams->Params.Ble.BitrateBandwidth;
      buf[1] = modulationParams->Params.Ble.ModulationIndex;
      buf[2] = modulationParams->Params.Ble.ModulationShaping;
      break;

    case SX128X_PACKET_TYPE_NONE:
      buf[0] = NULL;
      buf[1] = NULL;
      buf[2] = NULL;
      break;
  }
  sx128x_write_command (dev, SX128X_RADIO_SET_MODULATIONPARAMS, buf, 3);
}

void sx128x_set_packet_params (sx128x_dev *dev,
                            SX128X_PacketParams_t *packetParams)
{
  uint8_t buf[7];

  // Check if required configuration corresponds to the stored packet type
  // If not, silently update radio packet type
  if (dev->packet_type != packetParams->PacketType)
  {
    sx128x_set_packet_type (dev, packetParams->PacketType);
  }

  switch (packetParams->PacketType)
  {
    case SX128X_PACKET_TYPE_GFSK:
      buf[0] = packetParams->Params.Gfsk.PreambleLength;
      buf[1] = packetParams->Params.Gfsk.SyncWordLength;
      buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
      buf[3] = packetParams->Params.Gfsk.HeaderType;
      buf[4] = packetParams->Params.Gfsk.PayloadLength;
      buf[5] = packetParams->Params.Gfsk.CrcLength;
      buf[6] = packetParams->Params.Gfsk.Whitening;
      break;

    case SX128X_PACKET_TYPE_LORA:
    case SX128X_PACKET_TYPE_RANGING:
      buf[0] = packetParams->Params.LoRa.PreambleLength;
      buf[1] = packetParams->Params.LoRa.HeaderType;
      buf[2] = packetParams->Params.LoRa.PayloadLength;
      buf[3] = packetParams->Params.LoRa.CrcMode;
      buf[4] = packetParams->Params.LoRa.InvertIQ;
      buf[5] = NULL;
      buf[6] = NULL;
      break;

    case SX128X_PACKET_TYPE_FLRC:
      buf[0] = packetParams->Params.Flrc.PreambleLength;
      buf[1] = packetParams->Params.Flrc.SyncWordLength;
      buf[2] = packetParams->Params.Flrc.SyncWordMatch;
      buf[3] = packetParams->Params.Flrc.HeaderType;
      buf[4] = packetParams->Params.Flrc.PayloadLength;
      buf[5] = packetParams->Params.Flrc.CrcLength;
      buf[6] = packetParams->Params.Flrc.Whitening;
      break;

    case SX128X_PACKET_TYPE_BLE:
      buf[0] = packetParams->Params.Ble.ConnectionState;
      buf[1] = packetParams->Params.Ble.CrcField;
      buf[2] = packetParams->Params.Ble.BlePacketType;
      buf[3] = packetParams->Params.Ble.Whitening;
      buf[4] = NULL;
      buf[5] = NULL;
      buf[6] = NULL;
      break;

    case SX128X_PACKET_TYPE_NONE:
      buf[0] = NULL;
      buf[1] = NULL;
      buf[2] = NULL;
      buf[3] = NULL;
      buf[4] = NULL;
      buf[5] = NULL;
      buf[6] = NULL;
      break;
  }
  sx128x_write_command (dev, SX128X_RADIO_SET_PACKETPARAMS, buf, 7);
}

void sx128x_get_rx_nuffer_status (sx128x_dev *dev, uint8_t *payloadLength,
                              uint8_t *rxStartBufferPointer)
{
  uint8_t status[2];

  sx128x_read_command (dev, SX128X_RADIO_GET_RXBUFFERSTATUS, status, 2);

  // In case of LORA fixed header, the payloadLength is obtained by reading
  // the register REG_LR_PAYLOADLENGTH
  if ((sx128x_get_packet_type (dev) == SX128X_PACKET_TYPE_LORA)
      && (sx128x_read_register (dev, SX128X_REG_LR_PACKETPARAMS) >> 7 == 1))
  {
    *payloadLength = sx128x_read_register (dev, SX128X_REG_LR_PAYLOADLENGTH);
  }
  else if (sx128x_get_packet_type (dev) == SX128X_PACKET_TYPE_BLE)
  {
    // In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
    // so it is added there
    *payloadLength = status[0] + 2;
  }
  else
  {
    *payloadLength = status[0];
  }

  *rxStartBufferPointer = status[1];
}

void sx128x_get_packet_status (sx128x_dev *dev, SX128X_PacketStatus_t *pktStatus)
{
  uint8_t status[5];

  sx128x_read_command (dev, SX128X_RADIO_GET_PACKETSTATUS, status, 5);

  pktStatus->packetType = sx128x_get_packet_type (dev);
  switch (pktStatus->packetType)
  {
    case SX128X_PACKET_TYPE_GFSK:
      pktStatus->Params.Gfsk.RssiAvg = -status[0] / 2;
      pktStatus->Params.Gfsk.RssiSync = -status[1] / 2;

      pktStatus->Params.Gfsk.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.HeaderReceived = (status[2] >> 2)
          & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.PacketReceived = (status[2] >> 1)
          & 0x01;
      pktStatus->Params.Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

      pktStatus->Params.Gfsk.TxRxStatus.RxNoAck = (status[3] >> 5) & 0x01;
      pktStatus->Params.Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

      pktStatus->Params.Gfsk.SyncAddrStatus = status[4] & 0x07;
      break;

    case SX128X_PACKET_TYPE_LORA:
    case SX128X_PACKET_TYPE_RANGING:
      pktStatus->Params.LoRa.RssiPkt = -status[0] / 2;
      (status[1] < 128) ?
          (pktStatus->Params.LoRa.SnrPkt = status[1] / 4) :
          (pktStatus->Params.LoRa.SnrPkt = ((status[1] - 256) / 4));

      pktStatus->Params.LoRa.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = (status[2] >> 2)
          & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.PacketReceived = (status[2] >> 1)
          & 0x01;
      pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

      pktStatus->Params.LoRa.TxRxStatus.RxNoAck = (status[3] >> 5) & 0x01;
      pktStatus->Params.LoRa.TxRxStatus.PacketSent = status[3] & 0x01;

      pktStatus->Params.LoRa.SyncAddrStatus = status[4] & 0x07;
      break;

    case SX128X_PACKET_TYPE_FLRC:
      pktStatus->Params.Flrc.RssiAvg = -status[0] / 2;
      pktStatus->Params.Flrc.RssiSync = -status[1] / 2;

      pktStatus->Params.Flrc.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.HeaderReceived = (status[2] >> 2)
          & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.PacketReceived = (status[2] >> 1)
          & 0x01;
      pktStatus->Params.Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

      pktStatus->Params.Flrc.TxRxStatus.RxPid = (status[3] >> 6) & 0x03;
      pktStatus->Params.Flrc.TxRxStatus.RxNoAck = (status[3] >> 5) & 0x01;
      pktStatus->Params.Flrc.TxRxStatus.RxPidErr = (status[3] >> 4) & 0x01;
      pktStatus->Params.Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

      pktStatus->Params.Flrc.SyncAddrStatus = status[4] & 0x07;
      break;

    case SX128X_PACKET_TYPE_BLE:
      pktStatus->Params.Ble.RssiAvg = -status[0] / 2;
      pktStatus->Params.Ble.RssiSync = -status[1] / 2;

      pktStatus->Params.Ble.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
      pktStatus->Params.Ble.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
      pktStatus->Params.Ble.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
      pktStatus->Params.Ble.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
      pktStatus->Params.Ble.ErrorStatus.HeaderReceived = (status[2] >> 2)
          & 0x01;
      pktStatus->Params.Ble.ErrorStatus.PacketReceived = (status[2] >> 1)
          & 0x01;
      pktStatus->Params.Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

      pktStatus->Params.Ble.TxRxStatus.PacketSent = status[3] & 0x01;

      pktStatus->Params.Ble.SyncAddrStatus = status[4] & 0x07;
      break;

    case SX128X_PACKET_TYPE_NONE:
      // In that specific case, we set everything in the pktStatus to zeros
      // and reset the packet type accordingly
      memset (pktStatus, 0, sizeof(SX128X_PacketStatus_t));
      pktStatus->packetType = SX128X_PACKET_TYPE_NONE;
      break;
  }
}

int8_t sx128x_get_rssi_inst (sx128x_dev *dev)
{
  uint8_t raw = 0;

  sx128x_read_command (dev, SX128X_RADIO_GET_RSSIINST, &raw, 1);

  return (int8_t) (-raw / 2);
}

void sx128x_set_dio_irq_params (sx128x_dev *dev, uint16_t irqMask,
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
  sx128x_write_command (dev, SX128X_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

uint16_t sx128x_get_irq_status (sx128x_dev *dev)
{
  uint8_t irqStatus[2];

  sx128x_read_command (dev, SX128X_RADIO_GET_IRQSTATUS, irqStatus, 2);

  return (irqStatus[0] << 8) | irqStatus[1];
}

void sx128x_clear_irq_status (sx128x_dev *dev, uint16_t irq)
{
  uint8_t buf[2];

  buf[0] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
  buf[1] = (uint8_t) ((uint16_t) irq & 0x00FF);
  sx128x_write_command (dev, SX128X_RADIO_CLR_IRQSTATUS, buf, 2);
}

void sx128x_calibrate (sx128x_dev *dev, SX128X_CalibrationParams_t calibParam)
{
  uint8_t cal = (calibParam.ADCBulkPEnable << 5)
      | (calibParam.ADCBulkNEnable << 4) | (calibParam.ADCPulseEnable << 3)
      | (calibParam.PLLEnable << 2) | (calibParam.RC13MEnable << 1)
      | (calibParam.RC64KEnable);

  sx128x_write_command (dev, SX128X_RADIO_CALIBRATE, &cal, 1);
}

void sx128x_set_regulator_mode (sx128x_dev *dev, SX128X_RadioRegulatorModes_t mode)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_REGULATORMODE, (uint8_t*) &mode,
                         1);
}

void sx128x_set_save_context (sx128x_dev *dev)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_SAVECONTEXT, 0, 0);
}

void sx128x_set_auto_tx (sx128x_dev *dev, uint16_t time)
{
  uint16_t compensatedTime = time - (uint16_t) SX128X_AUTO_RX_TX_OFFSET;
  uint8_t buf[2];

  buf[0] = (uint8_t) ((compensatedTime >> 8) & 0x00FF);
  buf[1] = (uint8_t) (compensatedTime & 0x00FF);
  sx128x_write_command (dev, SX128X_RADIO_SET_AUTOTX, buf, 2);
}

void sx128x_stop_auto_tx (sx128x_dev *dev)
{
  uint8_t buf[2] =
  { 0x00, 0x00 };
  sx128x_write_command (dev, SX128X_RADIO_SET_AUTOTX, buf, 2);
}

void sx128x_set_auto_fs (sx128x_dev *dev, uint8_t enable)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_AUTOFS, &enable, 1);
}

void sx128x_set_long_preamble (sx128x_dev *dev, uint8_t enable)
{
  sx128x_write_command (dev, SX128X_RADIO_SET_LONGPREAMBLE, &enable, 1);
}

void sx128x_set_payload (sx128x_dev *dev, uint8_t *buffer, uint8_t size)
{
  sx128x_write_buffer (dev, 0x00, buffer, size);
}

uint8_t sx128x_get_payload (sx128x_dev *dev, uint8_t *buffer, uint8_t *size,
                            uint8_t maxSize)
{
  uint8_t offset;

  sx128x_get_rx_nuffer_status (dev, size, &offset);
  if (*size > maxSize)
  {
    return 1;
  }
  sx128x_read_buffer (dev, offset, buffer, *size);
  return 0;
}

void sx128x_send_payload (sx128x_dev *dev, uint8_t *payload, uint8_t size,
                          SX128X_TickTime_t timeout)
{
  sx128x_set_payload (dev, payload, size);
  sx128x_set_tx (dev, timeout);
}

uint8_t sx128x_set_sync_word (sx128x_dev *dev, uint8_t syncWordIdx,
                              uint8_t *syncWord)
{
  uint16_t addr;
  uint8_t syncwordSize = 0;

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_GFSK:
      syncwordSize = 5;
      switch (syncWordIdx)
      {
        case 1:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1;
          break;

        case 2:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS2;
          break;

        case 3:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS3;
          break;

        default:
          return 1;
      }
      break;

    case SX128X_PACKET_TYPE_FLRC:
      // For FLRC packet type, the SyncWord is one byte shorter and
      // the base address is shifted by one byte
      syncwordSize = 4;
      switch (syncWordIdx)
      {
        case 1:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1 + 1;
          break;

        case 2:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS2 + 1;
          break;

        case 3:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS3 + 1;
          break;

        default:
          return 1;
      }
      break;

    case SX128X_PACKET_TYPE_BLE:
      // For Ble packet type, only the first SyncWord is used and its
      // address is shifted by one byte
      syncwordSize = 4;
      switch (syncWordIdx)
      {
        case 1:
          addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1 + 1;
          break;

        default:
          return 1;
      }
      break;

    default:
      return 1;
  }
  sx128x_write_registers (dev, addr, syncWord, syncwordSize);
  return 0;
}

void sx128x_set_sync_word_error_tolerance (sx128x_dev *dev, uint8_t ErrorBits)
{
  ErrorBits = (sx128x_read_register (dev, SX128X_REG_LR_SYNCWORDTOLERANCE)
      & 0xF0) | (ErrorBits & 0x0F);
  sx128x_write_register (dev, SX128X_REG_LR_SYNCWORDTOLERANCE, ErrorBits);
}

void sx128x_set_crc_seed (sx128x_dev *dev, uint16_t seed)
{
  uint8_t val[2];

  val[0] = (uint8_t) (seed >> 8) & 0xFF;
  val[1] = (uint8_t) (seed & 0xFF);

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_GFSK:
    case SX128X_PACKET_TYPE_FLRC:
      sx128x_write_registers (dev, SX128X_REG_LR_CRCSEEDBASEADDR, val, 2);
      break;

    default:
      break;
  }
}

void sx128x_set_ble_access_address (sx128x_dev *dev, uint32_t accessAddress)
{
  sx128x_write_register (dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS,
                          (accessAddress >> 24) & 0x000000FF);
  sx128x_write_register (dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 1,
                          (accessAddress >> 16) & 0x000000FF);
  sx128x_write_register (dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 2,
                          (accessAddress >> 8) & 0x000000FF);
  sx128x_write_register (dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 3,
                          accessAddress & 0x000000FF);
}

void sx128x_set_ble_advertizer_access_address (sx128x_dev *dev)
{
  sx128x_set_ble_access_address (dev, SX128X_BLE_ADVERTIZER_ACCESS_ADDRESS);
}

void sx128x_set_crc_polynomial (sx128x_dev *dev, uint16_t polynomial)
{
  uint8_t val[2];

  val[0] = (uint8_t) (polynomial >> 8) & 0xFF;
  val[1] = (uint8_t) (polynomial & 0xFF);

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_GFSK:
    case SX128X_PACKET_TYPE_FLRC:
      sx128x_write_registers (dev, SX128X_REG_LR_CRCPOLYBASEADDR, val, 2);
      break;

    default:
      break;
  }
}

void sx128x_set_whitening_seed (sx128x_dev *dev, uint8_t seed)
{
  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_GFSK:
    case SX128X_PACKET_TYPE_FLRC:
    case SX128X_PACKET_TYPE_BLE:
      sx128x_write_register (dev, SX128X_REG_LR_WHITSEEDBASEADDR, seed);
      break;

    default:
      break;
  }
}

void sx128x_enable_manual_gain (sx128x_dev *dev)
{
  sx128x_write_register (
      dev,
      SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL,
      sx128x_read_register (dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL)
          | SX128X_MASK_MANUAL_GAIN_CONTROL);
  sx128x_write_register (
      dev, SX128X_REG_DEMOD_DETECTION,
      sx128x_read_register (dev, SX128X_REG_DEMOD_DETECTION) & SX128X_MASK_DEMOD_DETECTION);
}

void sx128x_disable_manual_gain (sx128x_dev *dev)
{
  sx128x_write_register (
      dev,
      SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL,
      sx128x_read_register (dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL)
          & ~SX128X_MASK_MANUAL_GAIN_CONTROL);
  sx128x_write_register (
      dev, SX128X_REG_DEMOD_DETECTION,
      sx128x_read_register (dev, SX128X_REG_DEMOD_DETECTION) | ~SX128X_MASK_DEMOD_DETECTION);
}

void sx128x_set_manual_gain_value (sx128x_dev *dev, uint8_t gain)
{
  sx128x_write_register (
      dev,
      SX128X_REG_MANUAL_GAIN_VALUE,
      (sx128x_read_register (dev, SX128X_REG_MANUAL_GAIN_VALUE)
          & SX128X_MASK_MANUAL_GAIN_VALUE) | gain);
}

void sx128x_set_lna_gain_setting (sx128x_dev *dev,
                                  const SX128X_RadioLnaSettings_t lnaSetting)
{
  switch (lnaSetting)
  {
    case SX128X_LNA_HIGH_SENSITIVITY_MODE:
    {
      sx128x_write_register (
          dev, SX128X_REG_LNA_REGIME,
          sx128x_read_register (dev, SX128X_REG_LNA_REGIME) | SX128X_MASK_LNA_REGIME);
      break;
    }
    case SX128X_LNA_LOW_POWER_MODE:
    {
      sx128x_write_register (
          dev, SX128X_REG_LNA_REGIME,
          sx128x_read_register (dev, SX128X_REG_LNA_REGIME) & ~SX128X_MASK_LNA_REGIME);
      break;
    }
  }
}

void sx128x_set_ranging_id_length (sx128x_dev *dev,
                                   SX128X_RadioRangingIdCheckLengths_t length)
{
  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_RANGING:
      sx128x_write_register (
          dev,
          SX128X_REG_LR_RANGINGIDCHECKLENGTH,
          ((((uint8_t) length) & 0x03) << 6)
              | (sx128x_read_register (dev, SX128X_REG_LR_RANGINGIDCHECKLENGTH) & 0x3F));
      break;

    default:
      break;
  }
}

void sx128x_set_device_ranging_address (sx128x_dev *dev, uint32_t address)
{
  uint8_t addrArray[] =
  { address >> 24, address >> 16, address >> 8, address };

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_RANGING:
      sx128x_write_registers (dev, SX128X_REG_LR_DEVICERANGINGADDR, addrArray, 4);
      break;

    default:
      break;
  }
}

void sx128x_set_ranging_request_address (sx128x_dev *dev, uint32_t address)
{
  uint8_t addrArray[] =
  { address >> 24, address >> 16, address >> 8, address };

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_RANGING:
      sx128x_write_registers (dev, SX128X_REG_LR_REQUESTRANGINGADDR, addrArray, 4);
      break;

    default:
      break;
  }
}

double sx128x_get_ranging_result (sx128x_dev *dev,
                                  SX128X_RadioRangingResultTypes_t resultType)
{
  uint32_t valLsb = 0;
  double val = 0.0;

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_RANGING:
      sx128x_set_standby (dev, SX128X_STDBY_XOSC);
      sx128x_write_register (dev, 0x97F,
                              sx128x_read_register (dev, 0x97F) | (1 << 1)); // enable LORA modem clock
      sx128x_write_register (
          dev,
          SX128X_REG_LR_RANGINGRESULTCONFIG,
          (sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTCONFIG)
              & SX128X_MASK_RANGINGMUXSEL) | ((((uint8_t) resultType) & 0x03) << 4));
      valLsb =
          ((sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTBASEADDR) << 16)
              | (sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTBASEADDR + 1)
                  << 8)
              | (sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTBASEADDR + 2)));
      sx128x_set_standby (dev, SX128X_STDBY_RC);

      // Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
      switch (resultType)
      {
        case SX128X_RANGING_RESULT_RAW:
          // Convert the ranging LSB to distance in meter
          val = (double) sx128x_complement2 (valLsb, 24)
              / (double) sx128x_get_lora_bandwidth (dev) * 36621.09375;
          break;

        case SX128X_RANGING_RESULT_AVERAGED:
        case SX128X_RANGING_RESULT_DEBIASED:
        case SX128X_RANGING_RESULT_FILTERED:
          val = (double) valLsb * 20.0 / 100.0;
          break;

        default:
          val = 0.0;
      }
      break;

    default:
      break;
  }
  return val;
}

uint8_t sx128x_get_ranging_power_delta_threshold_indicator (sx128x_dev *dev)
{
  sx128x_set_standby (dev, SX128X_STDBY_XOSC);
  sx128x_write_register (dev, 0x97F,
                          sx128x_read_register (dev, 0x97F) | (1 << 1)); // enable LoRa modem clock
  sx128x_write_register (
      dev,
      SX128X_REG_LR_RANGINGRESULTCONFIG,
      (sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTCONFIG)
          & SX128X_MASK_RANGINGMUXSEL)
          | ((((uint8_t) SX128X_RANGING_RESULT_RAW) & 0x03) << 4)); // Select raw results
  return sx128x_read_register (dev, SX128X_REG_RANGING_RSSI);
}

void sx128x_set_ranging_calibration (sx128x_dev *dev, uint16_t cal)
{
  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_RANGING:
      sx128x_write_register (dev, SX128X_REG_LR_RANGINGRERXTXDELAYCAL,
                              (uint8_t) ((cal >> 8) & 0xFF));
      sx128x_write_register (dev, SX128X_REG_LR_RANGINGRERXTXDELAYCAL + 1,
                              (uint8_t) ((cal) & 0xFF));
      break;

    default:
      break;
  }
}

void sx128x_ranging_clear_filter_result (sx128x_dev *dev)
{
  uint8_t regVal = sx128x_read_register (dev, SX128X_REG_LR_RANGINGRESULTCLEARREG);

  // To clear result, set bit 5 to 1 then to 0
  sx128x_write_register (dev, SX128X_REG_LR_RANGINGRESULTCLEARREG, regVal | (1 << 5));
  sx128x_write_register (dev, SX128X_REG_LR_RANGINGRESULTCLEARREG,
                          regVal & (~(1 << 5)));
}

void sx128x_ranging_set_filter_num_samples (sx128x_dev *dev, uint8_t num)
{
  // Silently set 8 as minimum value
  sx128x_write_register (
      dev, SX128X_REG_LR_RANGINGFILTERWINDOWSIZE,
      (num < SX128X_DEFAULT_RANGING_FILTER_SIZE) ? SX128X_DEFAULT_RANGING_FILTER_SIZE : num);
}

int8_t sx128x_parse_hex_file_line (sx128x_dev *dev, char *line)
{
  uint16_t addr;
  uint16_t n;
  uint8_t code;
  uint8_t bytes[256];

  if (sx128x_get_hex_file_line_fields (dev, line, bytes, &addr, &n, &code) != 0)
  {
    if (code == 0)
    {
      sx128x_write_registers (dev, addr, bytes, n);
    }
    if (code == 1)
    { // end of file
      //return 2;
    }
    if (code == 2)
    { // begin of file
      //return 3;
    }
  }
  else
  {
    return 0;
  }
  return 1;
}

void sx128x_set_ranging_role (sx128x_dev *dev, SX128X_RadioRangingRoles_t role)
{
  uint8_t buf[1];

  buf[0] = role;
  sx128x_write_command (dev, SX128X_RADIO_SET_RANGING_ROLE, &buf[0], 1);
}

int8_t sx128x_get_hex_file_line_fields (sx128x_dev *dev, char *line,
                                        uint8_t *bytes, uint16_t *addr,
                                        uint16_t *num, uint8_t *code)
{
  uint16_t sum, len, cksum;
  char *ptr;

  *num = 0;
  if (line[0] != ':')
  {
    return 0;
  }
  if (strlen (line) < 11)
  {
    return 0;
  }
  ptr = line + 1;
  if (!sscanf (ptr, "%02hx", &len))
  {
    return 0;
  }
  ptr += 2;
  if (strlen (line) < (11 + (len * 2)))
  {
    return 0;
  }
  if (!sscanf (ptr, "%04hx", addr))
  {
    return 0;
  }
  ptr += 4;
  if (!sscanf (ptr, "%02hhx", code))
  {
    return 0;
  }
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255)
      + ((*code >> 8) & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf (ptr, "%02hhx", &bytes[*num]))
    {
      return 0;
    }
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256)
    {
      return 0;
    }
  }
  if (!sscanf (ptr, "%02hx", &cksum))
  {
    return 0;
  }
  if (((sum & 255) + (cksum & 255)) & 255)
  {
    return 0; // checksum error
  }

  return 1;
}

double sx128x_get_frequency_error (sx128x_dev *dev)
{
  uint8_t efeRaw[3] =
  { 0 };
  uint32_t efe = 0;
  double efeHz = 0.0;

  switch (sx128x_get_packet_type (dev))
  {
    case SX128X_PACKET_TYPE_LORA:
    case SX128X_PACKET_TYPE_RANGING:
      efeRaw[0] = sx128x_read_register (
          dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
      efeRaw[1] = sx128x_read_register (
          dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
      efeRaw[2] = sx128x_read_register (
          dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
      efe = (efeRaw[0] << 16) | (efeRaw[1] << 8) | efeRaw[2];
      efe &= SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

      efeHz = 1.55 * (double) sx128x_complement2 (efe, 20)
          / (1600.0 / (double) sx128x_get_lora_bandwidth (dev) * 1000.0);
      break;

    case SX128X_PACKET_TYPE_NONE:
    case SX128X_PACKET_TYPE_BLE:
    case SX128X_PACKET_TYPE_FLRC:
    case SX128X_PACKET_TYPE_GFSK:
      break;
  }

  return efeHz;
}

int32_t sx128x_get_lora_bandwidth (sx128x_dev *dev)
{
  int32_t bwValue = 0;

  switch (dev->lora_bandwidth)
  {
    case SX128X_LORA_BW_0200:
      bwValue = 203125;
      break;

    case SX128X_LORA_BW_0400:
      bwValue = 406250;
      break;

    case SX128X_LORA_BW_0800:
      bwValue = 812500;
      break;

    case SX128X_LORA_BW_1600:
      bwValue = 1625000;
      break;

    default:
      bwValue = 0;
  }
  return bwValue;
}

double sx128x_get_ranging_correction_per_sf_bw_gain (
    sx128x_dev *dev, const SX128X_RadioLoRaSpreadingFactors_t sf,
    const SX128X_RadioLoRaBandwidths_t bw, const int8_t gain)
{
  uint8_t sf_index, bw_index;

  switch (sf)
  {
    case SX128X_LORA_SF5:
      sf_index = 0;
      break;
    case SX128X_LORA_SF6:
      sf_index = 1;
      break;
    case SX128X_LORA_SF7:
      sf_index = 2;
      break;
    case SX128X_LORA_SF8:
      sf_index = 3;
      break;
    case SX128X_LORA_SF9:
      sf_index = 4;
      break;
    case SX128X_LORA_SF10:
      sf_index = 5;
      break;
    case SX128X_LORA_SF11:
      sf_index = 6;
      break;
    case SX128X_LORA_SF12:
      sf_index = 7;
      break;
  }
  switch (bw)
  {
    case SX128X_LORA_BW_0400:
      bw_index = 0;
      break;
    case SX128X_LORA_BW_0800:
      bw_index = 1;
      break;
    case SX128X_LORA_BW_1600:
      bw_index = 2;
      break;
  }

  double correction = RangingCorrectionPerSfBwGain[sf_index][bw_index][gain];
  return correction;
}

double sx128x_compute_ranging_correction_polynome (
    const SX128X_RadioLoRaSpreadingFactors_t sf,
    const SX128X_RadioLoRaBandwidths_t bw, const double median)
{
  uint8_t sf_index, bw_index;

  switch (sf)
  {
    case SX128X_LORA_SF5:
      sf_index = 0;
      break;
    case SX128X_LORA_SF6:
      sf_index = 1;
      break;
    case SX128X_LORA_SF7:
      sf_index = 2;
      break;
    case SX128X_LORA_SF8:
      sf_index = 3;
      break;
    case SX128X_LORA_SF9:
      sf_index = 4;
      break;
    case SX128X_LORA_SF10:
      sf_index = 5;
      break;
    case SX128X_LORA_SF11:
      sf_index = 6;
      break;
    case SX128X_LORA_SF12:
      sf_index = 7;
      break;
  }
  switch (bw)
  {
    case SX128X_LORA_BW_0400:
      bw_index = 0;
      break;
    case SX128X_LORA_BW_0800:
      bw_index = 1;
      break;
    case SX128X_LORA_BW_1600:
      bw_index = 2;
      break;
  }
  const RangingCorrectionPolynomes_t *polynome =
      RangingCorrectionPolynomesPerSfBw[sf_index][bw_index];
  double correctedValue = 0.0;
  double correctionCoeff = 0;
  for (uint8_t order = 0; order < polynome->order; order++)
  {
    correctionCoeff = polynome->coefficients[order]
        * pow (median, polynome->order - order - 1);
    correctedValue += correctionCoeff;
  }
  return correctedValue;
}

void sx128x_set_interrupt_mode (sx128x_dev *dev)
{
  dev->polling_mode = false;
}

void sx128x_on_dio_irq (sx128x_dev *dev)
{
  /*
   * When polling mode is activated, it is up to the application to call
   * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
   * on radio interrupt.
   */
  if (dev->polling_mode == true)
  {
    dev->irq_state = true;
  }
  else
  {
    sx128x_process_irqs (dev);
  }
}

void sx128x_process_irqs (sx128x_dev *dev)
{
  SX128X_RadioPacketTypes_t packetType = SX128X_PACKET_TYPE_NONE;

  if (dev->op_mode == SX128X_MODE_SLEEP)
  {
    return; // DIO glitch on V2b :-)
  }

  if (dev->polling_mode == true)
  {
    if (dev->irq_state == true)
    {
      DISABLE_IRQ( );
      dev->irq_state = false;
      ENABLE_IRQ( );
    }
    else
    {
      return;
    }
  }

  packetType = sx128x_get_packet_type (dev);
  uint16_t irqRegs = sx128x_get_irq_status (dev);
  sx128x_clear_irq_status (dev, SX128X_IRQ_RADIO_ALL);

  switch (packetType)
  {
    case SX128X_PACKET_TYPE_GFSK:
    case SX128X_PACKET_TYPE_FLRC:
    case SX128X_PACKET_TYPE_BLE:
      switch (dev->op_mode)
      {
        case SX128X_MODE_RX:
          if ((irqRegs & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE)
          {
            if ((irqRegs & SX128X_IRQ_CRC_ERROR) == SX128X_IRQ_CRC_ERROR)
            {
              if ((dev != NULL) && (dev->callbacks.rxError != NULL))
              {
                dev->callbacks.rxError (SX128X_IRQ_CRC_ERROR_CODE);
              }
            }
            else if ((irqRegs & SX128X_IRQ_SYNCWORD_ERROR)
                == SX128X_IRQ_SYNCWORD_ERROR)
            {
              if ((dev != NULL) && (dev->callbacks.rxError != NULL))
              {
                dev->callbacks.rxError (SX128X_IRQ_SYNCWORD_ERROR_CODE);
              }
            }
            else
            {
              memset (dev->radio_rx_payload, 0, sizeof(dev->radio_rx_payload));
              sx128x_get_payload (dev, dev->radio_rx_payload,
                                  &dev->radio_rx_payload_len, 255);
              sx128x_get_packet_status (dev, &dev->packet_status);
              if ((dev != NULL) && (dev->callbacks.rxDone != NULL))
              {
                dev->callbacks.rxDone (dev->radio_rx_payload,
                                        dev->radio_rx_payload_len,
                                        dev->packet_status.Params.LoRa.RssiPkt,
                                        dev->packet_status.Params.LoRa.SnrPkt);
              }
            }
          }
          if ((irqRegs & SX128X_IRQ_SYNCWORD_VALID)
              == SX128X_IRQ_SYNCWORD_VALID)
          {
            if ((dev != NULL) && (dev->callbacks.rxSyncWordDone != NULL))
            {
              dev->callbacks.rxSyncWordDone ();
            }
          }
          if ((irqRegs & SX128X_IRQ_SYNCWORD_ERROR)
              == SX128X_IRQ_SYNCWORD_ERROR)
          {
            if ((dev != NULL) && (dev->callbacks.rxError != NULL))
            {
              dev->callbacks.rxError (SX128X_IRQ_SYNCWORD_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.rxTimeout != NULL))
            {
              dev->callbacks.rxTimeout ();
            }
          }
          if ((irqRegs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
          {
            if ((dev != NULL) && (dev->callbacks.txDone != NULL))
            {
              dev->callbacks.txDone ();
            }
          }
          break;
        case SX128X_MODE_TX:
          if ((irqRegs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
          {
            if ((dev != NULL) && (dev->callbacks.txDone != NULL))
            {
              dev->callbacks.txDone ();
            }
          }
          if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.txTimeout != NULL))
            {
              dev->callbacks.txTimeout ();
            }
          }
          break;
        default:
          // Unexpected IRQ: silently returns
          break;
      }
      break;
    case SX128X_PACKET_TYPE_LORA:
      switch (dev->op_mode)
      {
        case SX128X_MODE_RX:
          if ((irqRegs & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE)
          {
            if ((irqRegs & SX128X_IRQ_CRC_ERROR) == SX128X_IRQ_CRC_ERROR)
            {
              if ((dev != NULL) && (dev->callbacks.rxError != NULL))
              {
                dev->callbacks.rxError (SX128X_IRQ_CRC_ERROR_CODE);
              }
            }
            else
            {
              memset (dev->radio_rx_payload, 0, sizeof(dev->radio_rx_payload));
              sx128x_get_payload (dev, dev->radio_rx_payload,
                                  &dev->radio_rx_payload_len, 255);
              sx128x_get_packet_status (dev, &dev->packet_status);
              if ((dev != NULL) && (dev->callbacks.rxDone != NULL))
              {
                dev->callbacks.rxDone (dev->radio_rx_payload,
                                        dev->radio_rx_payload_len,
                                        dev->packet_status.Params.LoRa.RssiPkt,
                                        dev->packet_status.Params.LoRa.SnrPkt);
              }
            }
          }
          if ((irqRegs & SX128X_IRQ_HEADER_VALID) == SX128X_IRQ_HEADER_VALID)
          {
            if ((dev != NULL) && (dev->callbacks.rxHeaderDone != NULL))
            {
              dev->callbacks.rxHeaderDone ();
            }
          }
          if ((irqRegs & SX128X_IRQ_HEADER_ERROR) == SX128X_IRQ_HEADER_ERROR)
          {
            if ((dev != NULL) && (dev->callbacks.rxError != NULL))
            {
              dev->callbacks.rxError (SX128X_IRQ_HEADER_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.rxTimeout != NULL))
            {
              dev->callbacks.rxTimeout ();
            }
          }
          if ((irqRegs & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
              == SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
          {
            if ((dev != NULL) && (dev->callbacks.rxError != NULL))
            {
              dev->callbacks.rxError (SX128X_IRQ_RANGING_ON_LORA_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
          {
            if ((dev != NULL) && (dev->callbacks.txDone != NULL))
            {
              dev->callbacks.txDone ();
            }
          }
          break;
        case SX128X_MODE_TX:
          if ((irqRegs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE)
          {
            if ((dev != NULL) && (dev->callbacks.txDone != NULL))
            {
              dev->callbacks.txDone ();
            }
          }
          if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.txTimeout != NULL))
            {
              dev->callbacks.txTimeout ();
            }
          }
          break;
        case SX128X_MODE_CAD:
          if ((irqRegs & SX128X_IRQ_CAD_DONE) == SX128X_IRQ_CAD_DONE)
          {
            if ((irqRegs & SX128X_IRQ_CAD_ACTIVITY_DETECTED)
                == SX128X_IRQ_CAD_ACTIVITY_DETECTED)
            {
              if ((dev != NULL) && (dev->callbacks.cadDone != NULL))
              {
                dev->callbacks.cadDone (true);
              }
            }
            else
            {
              if ((dev != NULL) && (dev->callbacks.cadDone != NULL))
              {
                dev->callbacks.cadDone (false);
              }
            }
          }
          else if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT)
              == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.rxTimeout != NULL))
            {
              dev->callbacks.rxTimeout ();
            }
          }
          break;
        default:
          // Unexpected IRQ: silently returns
          break;
      }
      break;
    case SX128X_PACKET_TYPE_RANGING:
      switch (dev->op_mode)
      {
        // SX128x_MODE_RX indicates an IRQ on the Slave side
        case SX128X_MODE_RX:
          if ((irqRegs & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
              == SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_SLAVE_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID)
              == SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_SLAVE_VALID_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE)
              == SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_SLAVE_VALID_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_SLAVE_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_HEADER_VALID) == SX128X_IRQ_HEADER_VALID)
          {
            if ((dev != NULL) && (dev->callbacks.rxHeaderDone != NULL))
            {
              dev->callbacks.rxHeaderDone ();
            }
          }
          if ((irqRegs & SX128X_IRQ_HEADER_ERROR) == SX128X_IRQ_HEADER_ERROR)
          {
            if ((dev != NULL) && (dev->callbacks.rxError != NULL))
            {
              dev->callbacks.rxError (SX128X_IRQ_HEADER_ERROR_CODE);
            }
          }
          break;
          // SX128x_MODE_TX indicates an IRQ on the Master side
        case SX128X_MODE_TX:
          if ((irqRegs & SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT)
              == SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_MASTER_ERROR_CODE);
            }
          }
          if ((irqRegs & SX128X_IRQ_RANGING_MASTER_RESULT_VALID)
              == SX128X_IRQ_RANGING_MASTER_RESULT_VALID)
          {
            if ((dev != NULL) && (dev->callbacks.rangingDone != NULL))
            {
              dev->callbacks.rangingDone (SX128X_IRQ_RANGING_MASTER_VALID_CODE);
            }
          }
          break;
        default:
          // Unexpected IRQ: silently returns
          break;
      }
      break;
    default:
      // Unexpected IRQ: silently returns
      break;
  }
}

void sx128x_clear_instruction_ram (sx128x_dev *dev)
{
  // Clearing the instruction RAM is writing 0x00s on every bytes of the
  // instruction RAM
  uint8_t cmd_iram[SX128X_IRAM_SIZE+3] = {0};
  memset (cmd_iram, 0x00, SX128X_IRAM_SIZE+3);
  cmd_iram[0] = SX128X_RADIO_WRITE_REGISTER;
  cmd_iram[1] = (SX128X_IRAM_START_ADDRESS >> 8) & 0x00FF;
  cmd_iram[2] = SX128X_IRAM_START_ADDRESS & 0x00FF;

  dev->cs_low ();
  dev->write_buffer (cmd_iram, sizeof(cmd_iram));
  dev->cs_high ();

  // Wait for chip to be ready.
  dev->wait_on_busy ();
}

void sx128x_wakeup (sx128x_dev *dev)
{
  uint8_t tx_buf[2] =
  { SX128X_RADIO_GET_STATUS, 0 };

  dev->cs_low ();
  dev->write_buffer (tx_buf, 2);
  dev->cs_high ();

  // Wait for chip to be ready.
  dev->wait_on_busy ();
}

void sx128x_check_device_ready (sx128x_dev *dev)
{
  if (dev->op_mode == SX128X_MODE_SLEEP)
  {
    sx128x_wakeup (dev);
    // Switch is turned off when device is in sleep mode and turned on is all other modes
  }
  dev->wait_on_busy ();
}

void sx128x_write_command (sx128x_dev *dev, uint8_t command, uint8_t *buffer,
                            uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = (uint8_t) command;
  memcpy (tx_buf + 1, buffer, size);

  dev->write_buffer (tx_buf, size + 1);

  dev->cs_high ();

  if (command != SX128X_RADIO_SET_SLEEP)
  {
    dev->wait_on_busy ();
  }
}

uint8_t sx128x_read_command (sx128x_dev *dev, uint8_t command,
                             uint8_t *buffer, uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };
  uint8_t rx_buf[260] =
  { 0 };

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = (uint8_t) command;

  dev->write_read_buffer (tx_buf, rx_buf, size + 2);

  memcpy (buffer, rx_buf + 2, size);

  dev->cs_high ();

  dev->wait_on_busy ();

  return rx_buf[1];
}

void sx128x_write_registers (sx128x_dev *dev, uint16_t address,
                              uint8_t *buffer, uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  /* 地址结构 */
  tx_buf[0] = SX128X_RADIO_WRITE_REGISTER;
  tx_buf[1] = (address & 0xFF00) >> 8;
  tx_buf[2] = address & 0x00FF;
  memcpy (tx_buf + 3, buffer, size);

  dev->write_buffer (tx_buf, size + 3);

  dev->cs_high ();

  dev->wait_on_busy ();
}

void sx128x_write_register (sx128x_dev *dev, uint16_t address, uint8_t value)
{
  uint8_t data = value;

  sx128x_write_registers (dev, address, &data, 1);
}

void sx128x_read_registers (sx128x_dev *dev, uint16_t address, uint8_t *buffer,
                             uint16_t size)
{
  uint8_t tx_buf[260] =
  { 0 };
  uint8_t rx_buf[260] =
  { 0 };

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  /* 地址结构 */
  tx_buf[0] = SX128X_RADIO_READ_REGISTER;
  tx_buf[1] = (address & 0xFF00) >> 8;
  tx_buf[2] = address & 0x00FF;

  dev->write_read_buffer (tx_buf, rx_buf, size + 4);
  memcpy (buffer, rx_buf + 4, size);

  dev->cs_high ();

  dev->wait_on_busy ();
}

uint8_t sx128x_read_register (sx128x_dev *dev, uint16_t address)
{
  uint8_t data;

  sx128x_read_registers (dev, address, &data, 1);

  return data;
}

void sx128x_write_buffer (sx128x_dev *dev, uint8_t offset, uint8_t *buffer,
                           uint8_t size)
{
  uint8_t tx_buf[260] =
  { 0 };

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = SX128X_RADIO_WRITE_BUFFER;
  tx_buf[1] = offset;
  memcpy (tx_buf + 2, buffer, size);
  dev->write_buffer (tx_buf, size + 2);

  dev->cs_high ();

  dev->wait_on_busy ();
}

void sx128x_read_buffer (sx128x_dev *dev, uint8_t offset, uint8_t *buffer,
                         uint8_t size)
{
  uint16_t i;
  uint8_t tx_buf[260];
  uint8_t rx_buf[260];

  sx128x_check_device_ready (dev);
  dev->cs_low ();

  tx_buf[0] = SX128X_RADIO_READ_BUFFER;
  tx_buf[1] = offset;

  dev->write_read_buffer (tx_buf, rx_buf, size + 3);

  dev->cs_high ();

  memcpy (buffer, rx_buf + 3, size);

  dev->wait_on_busy ();
}

