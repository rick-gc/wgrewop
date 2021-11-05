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

 Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
 */
#ifndef __SX1280_H__
#define __SX1280_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /*!
   * \brief Provides the frequency of the chip running on the radio and the frequency step
   *
   * \remark These defines are used for computing the frequency divider to set the RF frequency
   */
#define SX128X_XTAL_FREQ                                   52000000
#define SX128X_FREQ_STEP                                   ( ( double )( SX128X_XTAL_FREQ / pow( 2.0, 18.0 ) ) )

  /*!
   * \brief Compensation delay for SetAutoTx/Rx functions in microseconds
   */
#define SX128X_AUTO_RX_TX_OFFSET                           33

  /*!
   * \brief The address of the register holding the firmware version MSB
   */
#define SX128X_REG_LR_FIRMWARE_VERSION_MSB                 0x0153

  /*!
   * \brief The address of the register holding the first byte defining the CRC seed
   *
   * \remark Only used for packet types GFSK and Flrc
   */
#define SX128X_REG_LR_CRCSEEDBASEADDR                      0x09C8

  /*!
   * \brief The address of the register holding the first byte defining the CRC polynomial
   *
   * \remark Only used for packet types GFSK and Flrc
   */
#define SX128X_REG_LR_CRCPOLYBASEADDR                      0x09C6

  /*!
   * \brief The address of the register holding the first byte defining the whitening seed
   *
   * \remark Only used for packet types GFSK, FLRC and BLE
   */
#define SX128X_REG_LR_WHITSEEDBASEADDR                     0x09C5

  /*!
   * \brief The address of the register holding the ranging id check length
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGIDCHECKLENGTH                 0x0931

  /*!
   * \brief The address of the register holding the device ranging id
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_DEVICERANGINGADDR                    0x0916

  /*!
   * \brief The address of the register holding the device ranging id
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_REQUESTRANGINGADDR                   0x0912

  /*!
   * \brief The address of the register holding ranging results configuration
   * and the corresponding mask
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGRESULTCONFIG                  0x0924
#define SX128X_MASK_RANGINGMUXSEL                          0xCF

  /*!
   * \brief The address of the register holding the first byte of ranging results
   * Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGRESULTBASEADDR                0x0961

  /*!
   * \brief The address of the register allowing to read ranging results
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGRESULTSFREEZE                 0x097F

  /*!
   * \brief The address of the register holding the first byte of ranging calibration
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGRERXTXDELAYCAL                0x092C

  /*!
   *\brief The address of the register holding the ranging filter window size
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGFILTERWINDOWSIZE              0x091E

  /*!
   *\brief The address of the register to reset for clearing ranging filter
   *
   * \remark Only used for packet type Ranging
   */
#define SX128X_REG_LR_RANGINGRESULTCLEARREG                0x0923

#define SX128X_REG_RANGING_RSSI                            0x0964

  /*!
   * \brief The default number of samples considered in built-in ranging filter
   */
#define SX128X_DEFAULT_RANGING_FILTER_SIZE                 127

  /*!
   * \brief The address of the register holding LORA packet parameters
   */
#define SX128X_REG_LR_PACKETPARAMS                         0x903

  /*!
   * \brief The address of the register holding payload length
   *
   * \remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
   */
#define SX128X_REG_LR_PAYLOADLENGTH                        0x901

  /*!
   * \brief The address of the instruction RAM and its size
   */
#define SX128X_IRAM_START_ADDRESS                          0x8000
#define SX128X_IRAM_SIZE                                   0x4000

  /*!
   * \brief The addresses of the registers holding SyncWords values
   *
   * \remark The addresses depends on the Packet Type in use, and not all
   *         SyncWords are available for every Packet Type
   */
#define SX128X_REG_LR_SYNCWORDBASEADDRESS1                 0x09CE
#define SX128X_REG_LR_SYNCWORDBASEADDRESS2                 0x09D3
#define SX128X_REG_LR_SYNCWORDBASEADDRESS3                 0x09D8

  /*!
   * \brief The MSB address and mask used to read the estimated frequency
   * error
   */
#define SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB        0x0954
#define SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK       0x0FFFFF

  /*!
   * \brief Defines how many bit errors are tolerated in sync word detection
   */
#define SX128X_REG_LR_SYNCWORDTOLERANCE                    0x09CD

  /*!
   * \brief Register for MSB Access Address (BLE)
   */
#define SX128X_REG_LR_BLE_ACCESS_ADDRESS                   0x09CF
#define SX128X_BLE_ADVERTIZER_ACCESS_ADDRESS               0x8E89BED6

  /*!
   * \brief Register address and mask for LNA regime selection
   */
#define SX128X_REG_LNA_REGIME                              0x0891
#define SX128X_MASK_LNA_REGIME                             0xC0

  /*!
   * \brief Register and mask enabling manual gain control
   */
#define SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL              0x089F
#define SX128X_MASK_MANUAL_GAIN_CONTROL                    0x80

  /*!
   * \brief Register and mask controlling demodulation detection
   */
#define SX128X_REG_DEMOD_DETECTION                         0x0895
#define SX128X_MASK_DEMOD_DETECTION                        0xFE

  /*!
   * \brief Register and mask setting manual gain value
   */
#define SX128X_REG_MANUAL_GAIN_VALUE                       0x089E
#define SX128X_MASK_MANUAL_GAIN_VALUE                      0xF0

  /*!
   * \brief Selector values to configure LNA regime
   */
  typedef enum
  {
    SX128X_LNA_LOW_POWER_MODE,        //!< Allow maximum efficiency of sx1280 (default)
    SX128X_LNA_HIGH_SENSITIVITY_MODE, //!< Allow to use highest three steps of LNA gain and increase current consumption
  } SX128X_RadioLnaSettings_t;

  /*!
   * \brief Structure describing the radio status
   */
  typedef union
  {
    /*!
     * \brief Structure of the radio status
     */
    struct
    {
      uint8_t CpuBusy :1;  //!< Flag for CPU radio busy
      uint8_t DmaBusy :1;  //!< Flag for DMA busy
      uint8_t CmdStatus :3;  //!< Command status
      uint8_t ChipMode :3;  //!< Chip mode
    } Fields;

    /*!
     * \brief Serialized radio status
     */
    uint8_t Value;
  } SX128X_RadioStatus_t;

  /*!
   * \brief Represents the states of the radio
   */
  typedef enum
  {
    SX128X_RF_IDLE = 0x00,         //!< The radio is idle
    SX128X_RF_RX_RUNNING,                   //!< The radio is in reception state
    SX128X_RF_TX_RUNNING,                //!< The radio is in transmission state
    SX128X_RF_CAD,           //!< The radio is doing channel activity detection
  } SX128X_RadioStates_t;

  /*!
   * \brief Represents the operating mode the radio is actually running
   */
  typedef enum
  {
    SX128X_MODE_SLEEP = 0x00,         //! The radio is in sleep mode
    SX128X_MODE_STDBY_RC,    //! The radio is in standby mode with RC oscillator
    SX128X_MODE_STDBY_XOSC, //! The radio is in standby mode with XOSC oscillator
    SX128X_MODE_FS,                 //! The radio is in frequency synthesis mode
    SX128X_MODE_TX,                            //! The radio is in transmit mode
    SX128X_MODE_RX,                             //! The radio is in receive mode
    SX128X_MODE_CAD          //! The radio is in channel activity detection mode
  } SX128X_RadioOperatingModes_t;

  /*!
   * \brief Declares the oscillator in use while in standby mode
   *
   * Using the SX128x_STDBY_RC standby mode allow to reduce the energy consumption
   * SX128x_STDBY_XOSC should be used for time critical applications
   */
  typedef enum
  {
    SX128X_STDBY_RC = 0x00, SX128X_STDBY_XOSC = 0x01,
  } SX128X_RadioStandbyModes_t;

  /*!
   * \brief Declares the power regulation used to power the device
   *
   * This command allows the user to specify if DC-DC or LDO is used for power regulation.
   * Using only LDO implies that the Rx or Tx current is doubled
   */
  typedef enum
  {
    SX128X_USE_LDO = 0x00,           //! Use LDO (default value)
    SX128X_USE_DCDC = 0x01,           //! Use DCDC
  } SX128X_RadioRegulatorModes_t;

  /*!
   * \brief Represents the possible packet type (i.e. modem) used
   */
  typedef enum
  {
    SX128X_PACKET_TYPE_GFSK = 0x00,
    SX128X_PACKET_TYPE_LORA,
    SX128X_PACKET_TYPE_RANGING,
    SX128X_PACKET_TYPE_FLRC,
    SX128X_PACKET_TYPE_BLE,
    SX128X_PACKET_TYPE_NONE = 0x0F,
  } SX128X_RadioPacketTypes_t;

  /*!
   * \brief Represents the ramping time for power amplifier
   */
  typedef enum
  {
    SX128X_RADIO_RAMP_02_US = 0x00,
    SX128X_RADIO_RAMP_04_US = 0x20,
    SX128X_RADIO_RAMP_06_US = 0x40,
    SX128X_RADIO_RAMP_08_US = 0x60,
    SX128X_RADIO_RAMP_10_US = 0x80,
    SX128X_RADIO_RAMP_12_US = 0xA0,
    SX128X_RADIO_RAMP_16_US = 0xC0,
    SX128X_RADIO_RAMP_20_US = 0xE0,
  } SX128X_RadioRampTimes_t;

  /*!
   * \brief Represents the number of symbols to be used for channel activity detection operation
   */
  typedef enum
  {
    SX128X_LORA_CAD_01_SYMBOL = 0x00,
    SX128X_LORA_CAD_02_SYMBOL = 0x20,
    SX128X_LORA_CAD_04_SYMBOL = 0x40,
    SX128X_LORA_CAD_08_SYMBOL = 0x60,
    SX128X_LORA_CAD_16_SYMBOL = 0x80,
  } SX128X_RadioLoRaCadSymbols_t;

  /*!
   * \brief Represents the possible combinations of bitrate and bandwidth for
   *        GFSK and BLE packet types
   *
   * The bitrate is expressed in Mb/s and the bandwidth in MHz
   */
  typedef enum
  {
    SX1280X_GFSK_BLE_BR_2_000_BW_2_4 = 0x04,
    SX1280X_GFSK_BLE_BR_1_600_BW_2_4 = 0x28,
    SX1280X_GFSK_BLE_BR_1_000_BW_2_4 = 0x4C,
    SX1280X_GFSK_BLE_BR_1_000_BW_1_2 = 0x45,
    SX1280X_GFSK_BLE_BR_0_800_BW_2_4 = 0x70,
    SX1280X_GFSK_BLE_BR_0_800_BW_1_2 = 0x69,
    SX1280X_GFSK_BLE_BR_0_500_BW_1_2 = 0x8D,
    SX1280X_GFSK_BLE_BR_0_500_BW_0_6 = 0x86,
    SX1280X_GFSK_BLE_BR_0_400_BW_1_2 = 0xB1,
    SX1280X_GFSK_BLE_BR_0_400_BW_0_6 = 0xAA,
    SX1280X_GFSK_BLE_BR_0_250_BW_0_6 = 0xCE,
    SX1280X_GFSK_BLE_BR_0_250_BW_0_3 = 0xC7,
    SX1280X_GFSK_BLE_BR_0_125_BW_0_3 = 0xEF,
  } SX128X_RadioGfskBleBitrates_t;

  /*!
   * \brief Represents the modulation index used in GFSK and BLE packet
   *        types
   */
  typedef enum
  {
    SX128X_GFSK_BLE_MOD_IND_0_35 = 0,
    SX128X_GFSK_BLE_MOD_IND_0_50 = 1,
    SX128X_GFSK_BLE_MOD_IND_0_75 = 2,
    SX128X_GFSK_BLE_MOD_IND_1_00 = 3,
    SX128X_GFSK_BLE_MOD_IND_1_25 = 4,
    SX128X_GFSK_BLE_MOD_IND_1_50 = 5,
    SX128X_GFSK_BLE_MOD_IND_1_75 = 6,
    SX128X_GFSK_BLE_MOD_IND_2_00 = 7,
    SX128X_GFSK_BLE_MOD_IND_2_25 = 8,
    SX128X_GFSK_BLE_MOD_IND_2_50 = 9,
    SX128X_GFSK_BLE_MOD_IND_2_75 = 10,
    SX128X_GFSK_BLE_MOD_IND_3_00 = 11,
    SX128X_GFSK_BLE_MOD_IND_3_25 = 12,
    SX128X_GFSK_BLE_MOD_IND_3_50 = 13,
    SX128X_GFSK_BLE_MOD_IND_3_75 = 14,
    SX128X_GFSK_BLE_MOD_IND_4_00 = 15,
  } SX128X_RadioGfskBleModIndexes_t;

  /*!
   * \brief Represents the possible combination of bitrate and bandwidth for FLRC
   *        packet type
   *
   * The bitrate is in Mb/s and the bitrate in MHz
   */
  typedef enum
  {
    SX128X_FLRC_BR_2_600_BW_2_4 = 0x04,
    SX128X_FLRC_BR_2_080_BW_2_4 = 0x28,
    SX128X_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX128X_FLRC_BR_1_040_BW_1_2 = 0x69,
    SX128X_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX128X_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX128X_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX128X_FLRC_BR_0_260_BW_0_3 = 0xEB,
  } SX128X_RadioFlrcBitrates_t;

  /*!
   * \brief Represents the possible values for coding rate parameter in FLRC
   *        packet type
   */
  typedef enum
  {
    SX128X_FLRC_CR_1_2 = 0x00,
    SX128X_FLRC_CR_3_4 = 0x02,
    SX128X_FLRC_CR_1_0 = 0x04,
  } SX128X_RadioFlrcCodingRates_t;

  /*!
   * \brief Represents the modulation shaping parameter for GFSK, FLRC and BLE
   *        packet types
   */
  typedef enum
  {
    SX128X_RADIO_MOD_SHAPING_BT_OFF = 0x00,         //! No filtering
    SX128X_RADIO_MOD_SHAPING_BT_1_0 = 0x10,
    SX128X_RADIO_MOD_SHAPING_BT_0_5 = 0x20,
  } SX128X_RadioModShapings_t;

  /*!
   * \brief Represents the possible spreading factor values in LORA packet types
   */
  typedef enum
  {
    SX128X_LORA_SF5 = 0x50,
    SX128X_LORA_SF6 = 0x60,
    SX128X_LORA_SF7 = 0x70,
    SX128X_LORA_SF8 = 0x80,
    SX128X_LORA_SF9 = 0x90,
    SX128X_LORA_SF10 = 0xA0,
    SX128X_LORA_SF11 = 0xB0,
    SX128X_LORA_SF12 = 0xC0,
  } SX128X_RadioLoRaSpreadingFactors_t;

  /*!
   * \brief Represents the bandwidth values for LORA packet type
   */
  typedef enum
  {
    SX128X_LORA_BW_0200 = 0x34,
    SX128X_LORA_BW_0400 = 0x26,
    SX128X_LORA_BW_0800 = 0x18,
    SX128X_LORA_BW_1600 = 0x0A,
  } SX128X_RadioLoRaBandwidths_t;

  /*!
   * \brief Represents the coding rate values for LORA packet type
   */
  typedef enum
  {
    SX128X_LORA_CR_4_5 = 0x01,
    SX128X_LORA_CR_4_6 = 0x02,
    SX128X_LORA_CR_4_7 = 0x03,
    SX128X_LORA_CR_4_8 = 0x04,
    SX128X_LORA_CR_LI_4_5 = 0x05,
    SX128X_LORA_CR_LI_4_6 = 0x06,
    SX128X_LORA_CR_LI_4_7 = 0x07,
  } SX128X_RadioLoRaCodingRates_t;

  /*!
   * \brief Represents the preamble length values for GFSK and FLRC packet
   *        types
   */
  typedef enum
  {
    SX128X_PREAMBLE_LENGTH_04_BITS = 0x00,         //!< Preamble length: 04 bits
    SX128X_PREAMBLE_LENGTH_08_BITS = 0x10,         //!< Preamble length: 08 bits
    SX128X_PREAMBLE_LENGTH_12_BITS = 0x20,         //!< Preamble length: 12 bits
    SX128X_PREAMBLE_LENGTH_16_BITS = 0x30,         //!< Preamble length: 16 bits
    SX128X_PREAMBLE_LENGTH_20_BITS = 0x40,         //!< Preamble length: 20 bits
    SX128X_PREAMBLE_LENGTH_24_BITS = 0x50,         //!< Preamble length: 24 bits
    SX128X_PREAMBLE_LENGTH_28_BITS = 0x60,         //!< Preamble length: 28 bits
    SX128X_PREAMBLE_LENGTH_32_BITS = 0x70,        //!< Preamble length: 32 bits
  } SX128X_RadioPreambleLengths_t;

  /*!
   * \brief Represents the SyncWord length for FLRC packet type
   */
  typedef enum
  {
    SX128X_FLRC_NO_SYNCWORD = 0x00, SX128X_FLRC_SYNCWORD_LENGTH_4_BYTE = 0x04,
  } SX128X_RadioFlrcSyncWordLengths_t;

  /*!
   * \brief The length of sync words for GFSK packet type
   */
  typedef enum
  {
    SX128X_GFSK_SYNCWORD_LENGTH_1_BYTE = 0x00,     //!< Sync word length: 1 byte
    SX128X_GFSK_SYNCWORD_LENGTH_2_BYTE = 0x02,    //!< Sync word length: 2 bytes
    SX128X_GFSK_SYNCWORD_LENGTH_3_BYTE = 0x04,    //!< Sync word length: 3 bytes
    SX128X_GFSK_SYNCWORD_LENGTH_4_BYTE = 0x06,    //!< Sync word length: 4 bytes
    SX128X_GFSK_SYNCWORD_LENGTH_5_BYTE = 0x08,   //!< Sync word length: 5 bytes
  } SX128X_RadioSyncWordLengths_t;

  /*!
   * \brief Represents the possible combinations of SyncWord correlators
   *        activated for GFSK and FLRC packet types
   */
  typedef enum
  {
    RADIO_RX_MATCH_SYNCWORD_OFF = 0x00, //!< No correlator turned on, i.e. do not search for SyncWord
    RADIO_RX_MATCH_SYNCWORD_1 = 0x10,
    RADIO_RX_MATCH_SYNCWORD_2 = 0x20,
    RADIO_RX_MATCH_SYNCWORD_1_2 = 0x30,
    RADIO_RX_MATCH_SYNCWORD_3 = 0x40,
    RADIO_RX_MATCH_SYNCWORD_1_3 = 0x50,
    RADIO_RX_MATCH_SYNCWORD_2_3 = 0x60,
    RADIO_RX_MATCH_SYNCWORD_1_2_3 = 0x70,
  } SX128X_RadioSyncWordRxMatchs_t;

  /*!
   *  \brief Radio packet length mode for GFSK and FLRC packet types
   */
  typedef enum
  {
    SX128X_RADIO_PACKET_FIXED_LENGTH = 0x00, //!< The packet is known on both sides, no header included in the packet
    SX128X_RADIO_PACKET_VARIABLE_LENGTH = 0x20, //!< The packet is on variable size, header included
  } SX128X_RadioPacketLengthModes_t;

  /*!
   * \brief Represents the CRC length for GFSK and FLRC packet types
   *
   * \warning Not all configurations are available for both GFSK and FLRC
   *          packet type. Refer to the datasheet for possible configuration.
   */
  typedef enum
  {
    SX128X_RADIO_CRC_OFF = 0x00,         //!< No CRC in use
    SX128X_RADIO_CRC_1_BYTES = 0x10,
    SX128X_RADIO_CRC_2_BYTES = 0x20,
    SX128X_RADIO_CRC_3_BYTES = 0x30,
  } SX128X_RadioCrcTypes_t;

  /*!
   * \brief Radio whitening mode activated or deactivated for GFSK, FLRC and
   *        BLE packet types
   */
  typedef enum
  {
    SX128X_RADIO_WHITENING_ON = 0x00, SX128X_RADIO_WHITENING_OFF = 0x08,
  } SX128X_RadioWhiteningModes_t;

  /*!
   * \brief Holds the packet length mode of a LORA packet type
   */
  typedef enum
  {
    SX128X_LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
    SX128X_LORA_PACKET_FIXED_LENGTH = 0x80, //!< The packet is known on both sides, no header included in the packet
    SX128X_LORA_PACKET_EXPLICIT = SX128X_LORA_PACKET_VARIABLE_LENGTH,
    SX128X_LORA_PACKET_IMPLICIT = SX128X_LORA_PACKET_FIXED_LENGTH,
  } SX128X_RadioLoRaPacketLengthsModes_t;

  /*!
   * \brief Represents the CRC mode for LORA packet type
   */
  typedef enum
  {
    SX128X_LORA_CRC_ON = 0x20,         //!< CRC activated
    SX128X_LORA_CRC_OFF = 0x00,         //!< CRC not used
  } SX128X_RadioLoRaCrcModes_t;

  /*!
   * \brief Represents the IQ mode for LORA packet type
   */
  typedef enum
  {
    SX128X_LORA_IQ_NORMAL = 0x40, SX128X_LORA_IQ_INVERTED = 0x00,
  } SX128X_RadioLoRaIQModes_t;

  /*!
   * \brief Represents the length of the ID to check in ranging operation
   */
  typedef enum
  {
    SX128X_RANGING_IDCHECK_LENGTH_08_BITS = 0x00,
    SX128X_RANGING_IDCHECK_LENGTH_16_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_24_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_32_BITS,
  } SX128X_RadioRangingIdCheckLengths_t;

  /*!
   * \brief Represents the result type to be used in ranging operation
   */
  typedef enum
  {
    SX128X_RANGING_RESULT_RAW = 0x00,
    SX128X_RANGING_RESULT_AVERAGED = 0x01,
    SX128X_RANGING_RESULT_DEBIASED = 0x02,
    SX128X_RANGING_RESULT_FILTERED = 0x03,
  } SX128X_RadioRangingResultTypes_t;

  /*!
   * \brief Represents the connection state for BLE packet type
   */
  typedef enum
  {
    SX128X_BLE_PAYLOAD_LENGTH_MAX_31_BYTES = 0x00,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_37_BYTES = 0x20,
    SX128X_BLE_TX_TEST_MODE = 0x40,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_255_BYTES = 0x80,
  } SX128X_RadioBleConnectionStates_t;

  /*!
   * \brief Represents the CRC field length for BLE packet type
   */
  typedef enum
  {
    SX128X_BLE_CRC_OFF = 0x00, SX128X_BLE_CRC_3B = 0x10,
  } SX128X_RadioBleCrcFields_t;

  /*!
   * \brief Represents the specific packets to use in BLE packet type
   */
  typedef enum
  {
    SX128X_BLE_PRBS_9 = 0x00, //!< Pseudo Random Binary Sequence based on 9th degree polynomial
    SX128X_BLE_PRBS_15 = 0x0C, //!< Pseudo Random Binary Sequence based on 15th degree polynomial
    SX128X_BLE_EYELONG_1_0 = 0x04,         //!< Repeated '11110000' sequence
    SX128X_BLE_EYELONG_0_1 = 0x18,         //!< Repeated '00001111' sequence
    SX128X_BLE_EYESHORT_1_0 = 0x08,         //!< Repeated '10101010' sequence
    SX128X_BLE_EYESHORT_0_1 = 0x1C,         //!< Repeated '01010101' sequence
    SX128X_BLE_ALL_1 = 0x10,         //!< Repeated '11111111' sequence
    SX128X_BLE_ALL_0 = 0x14,         //!< Repeated '00000000' sequence
  } SX128X_RadioBlePacketTypes_t;

  /*!
   * \brief Represents the interruption masks available for the radio
   *
   * \remark Note that not all these interruptions are available for all packet types
   */
  typedef enum
  {
    SX128X_IRQ_RADIO_NONE = 0x0000,
    SX128X_IRQ_TX_DONE = 0x0001,
    SX128X_IRQ_RX_DONE = 0x0002,
    SX128X_IRQ_SYNCWORD_VALID = 0x0004,
    SX128X_IRQ_SYNCWORD_ERROR = 0x0008,
    SX128X_IRQ_HEADER_VALID = 0x0010,
    SX128X_IRQ_HEADER_ERROR = 0x0020,
    SX128X_IRQ_CRC_ERROR = 0x0040,
    SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE = 0x0080,
    SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED = 0x0100,
    SX128X_IRQ_RANGING_MASTER_RESULT_VALID = 0x0200,
    SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT = 0x0400,
    SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID = 0x0800,
    SX128X_IRQ_CAD_DONE = 0x1000,
    SX128X_IRQ_CAD_ACTIVITY_DETECTED = 0x2000,
    SX128X_IRQ_RX_TX_TIMEOUT = 0x4000,
    SX128X_IRQ_PREAMBLE_DETECTED = 0x8000,
    SX128X_IRQ_RADIO_ALL = 0xFFFF,
  } SX128X_RadioIrqMasks_t;

  /*!
   * \brief Represents the digital input/output of the radio
   */
  typedef enum
  {
    SX128X_RADIO_DIO1 = 0x02, SX128X_RADIO_DIO2 = 0x04, SX128X_RADIO_DIO3 = 0x08,
  } SX128X_RadioDios_t;

  /*!
   * \brief Represents the tick size available for Rx/Tx timeout operations
   */
  typedef enum
  {
    SX128X_RADIO_TICK_SIZE_0015_US = 0x00,
    SX128X_RADIO_TICK_SIZE_0062_US = 0x01,
    SX128X_RADIO_TICK_SIZE_1000_US = 0x02,
    SX128X_RADIO_TICK_SIZE_4000_US = 0x03,
  } SX128X_RadioTickSizes_t;

  /*!
   * \brief Represents the role of the radio during ranging operations
   */
  typedef enum
  {
    SX128X_RADIO_RANGING_ROLE_SLAVE = 0x00,
    SX128X_RADIO_RANGING_ROLE_MASTER = 0x01,
  } SX128X_RadioRangingRoles_t;

  /*!
   * \brief Represents all possible opcode understood by the radio
   */
  typedef enum RadioCommands_u
  {
    SX128X_RADIO_GET_STATUS = 0xC0,
    SX128X_RADIO_WRITE_REGISTER = 0x18,
    SX128X_RADIO_READ_REGISTER = 0x19,
    SX128X_RADIO_WRITE_BUFFER = 0x1A,
    SX128X_RADIO_READ_BUFFER = 0x1B,
    SX128X_RADIO_SET_SLEEP = 0x84,
    SX128X_RADIO_SET_STANDBY = 0x80,
    SX128X_RADIO_SET_FS = 0xC1,
    SX128X_RADIO_SET_TX = 0x83,
    SX128X_RADIO_SET_RX = 0x82,
    SX128X_RADIO_SET_RXDUTYCYCLE = 0x94,
    SX128X_RADIO_SET_CAD = 0xC5,
    SX128X_RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
    SX128X_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX128X_RADIO_SET_PACKETTYPE = 0x8A,
    SX128X_RADIO_GET_PACKETTYPE = 0x03,
    SX128X_RADIO_SET_RFFREQUENCY = 0x86,
    SX128X_RADIO_SET_TXPARAMS = 0x8E,
    SX128X_RADIO_SET_CADPARAMS = 0x88,
    SX128X_RADIO_SET_BUFFERBASEADDRESS = 0x8F,
    SX128X_RADIO_SET_MODULATIONPARAMS = 0x8B,
    SX128X_RADIO_SET_PACKETPARAMS = 0x8C,
    SX128X_RADIO_GET_RXBUFFERSTATUS = 0x17,
    SX128X_RADIO_GET_PACKETSTATUS = 0x1D,
    SX128X_RADIO_GET_RSSIINST = 0x1F,
    SX128X_RADIO_SET_DIOIRQPARAMS = 0x8D,
    SX128X_RADIO_GET_IRQSTATUS = 0x15,
    SX128X_RADIO_CLR_IRQSTATUS = 0x97,
    SX128X_RADIO_CALIBRATE = 0x89,
    SX128X_RADIO_SET_REGULATORMODE = 0x96,
    SX128X_RADIO_SET_SAVECONTEXT = 0xD5,
    SX128X_RADIO_SET_AUTOTX = 0x98,
    SX128X_RADIO_SET_AUTOFS = 0x9E,
    SX128X_RADIO_SET_LONGPREAMBLE = 0x9B,
    SX128X_RADIO_SET_UARTSPEED = 0x9D,
    SX128X_RADIO_SET_RANGING_ROLE = 0xA3,
  } SX128X_RadioCommands_t;

  /*!
   * \brief Represents an amount of time measurable by the radio clock
   *
   * @code
   * Time = Step * NbSteps
   * Example:
   * Step = RADIO_TICK_SIZE_4000_US( 4 ms )
   * NbSteps = 1000
   * Time = 4e-3 * 1000 = 4 seconds
   * @endcode
   */
  typedef struct TickTime_s
  {
    SX128X_RadioTickSizes_t Step;                      //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
  } SX128X_TickTime_t;

  /*!
   * \brief RX_TX_CONTINUOUS and RX_TX_SINGLE are two particular values for TickTime.
   * The former keep the radio in Rx or Tx mode, even after successfull reception
   * or transmission. It should never generate Timeout interrupt.
   * The later let the radio enought time to make one reception or transmission.
   * No Timeout interrupt is generated, and the radio fall in StandBy mode after
   * reception or transmission.
   */
#define SX128X_RX_TX_CONTINUOUS ( SX128X_TickTime_t ){ SX128X_RADIO_TICK_SIZE_0015_US, 0xFFFF }
#define SX128X_RX_TX_SINGLE     ( SX128X_TickTime_t ){ SX128X_RADIO_TICK_SIZE_0015_US, 0 }

  /*!
   * \brief The type describing the modulation parameters for every packet types
   */
  typedef struct
  {
    SX128X_RadioPacketTypes_t PacketType; //!< Packet to which the modulation parameters are referring to.
//    union
    struct
    {
      /*!
       * \brief Holds the GFSK modulation parameters
       *
       * In GFSK modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
       */
      struct
      {
        SX128X_RadioGfskBleBitrates_t BitrateBandwidth; //!< The bandwidth and bit-rate values for BLE and GFSK modulations
        SX128X_RadioGfskBleModIndexes_t ModulationIndex; //!< The coding rate for BLE and GFSK modulations
        SX128X_RadioModShapings_t ModulationShaping; //!< The modulation shaping for BLE and GFSK modulations
      } Gfsk;
      /*!
       * \brief Holds the LORA modulation parameters
       *
       * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
       */
      struct
      {
        SX128X_RadioLoRaSpreadingFactors_t SpreadingFactor; //!< Spreading Factor for the LORA modulation
        SX128X_RadioLoRaBandwidths_t Bandwidth; //!< Bandwidth for the LORA modulation
        SX128X_RadioLoRaCodingRates_t CodingRate; //!< Coding rate for the LORA modulation
      } LoRa;
      /*!
       * \brief Holds the FLRC modulation parameters
       *
       * In FLRC modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
       */
      struct
      {
        SX128X_RadioFlrcBitrates_t BitrateBandwidth; //!< The bandwidth and bit-rate values for FLRC modulation
        SX128X_RadioFlrcCodingRates_t CodingRate; //!< The coding rate for FLRC modulation
        SX128X_RadioModShapings_t ModulationShaping; //!< The modulation shaping for FLRC modulation
      } Flrc;
      /*!
       * \brief Holds the BLE modulation parameters
       *
       * In BLE modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
       */
      struct
      {
        SX128X_RadioGfskBleBitrates_t BitrateBandwidth; //!< The bandwidth and bit-rate values for BLE and GFSK modulations
        SX128X_RadioGfskBleModIndexes_t ModulationIndex; //!< The coding rate for BLE and GFSK modulations
        SX128X_RadioModShapings_t ModulationShaping; //!< The modulation shaping for BLE and GFSK modulations
      } Ble;
    } Params;                     //!< Holds the modulation parameters structure
  } SX128X_ModulationParams_t;

  /*!
   * \brief Structure describing the error codes for callback functions
   */
  typedef enum
  {
    SX128X_IRQ_HEADER_ERROR_CODE = 0x00,
    SX128X_IRQ_SYNCWORD_ERROR_CODE,
    SX128X_IRQ_CRC_ERROR_CODE,
    SX128X_IRQ_RANGING_ON_LORA_ERROR_CODE,
  } SX128X_IrqErrorCode_t;

  /*!
   * \brief Structure describing the ranging codes for callback functions
   */
  typedef enum
  {
    SX128X_IRQ_RANGING_SLAVE_ERROR_CODE = 0x00,
    SX128X_IRQ_RANGING_SLAVE_VALID_CODE,
    SX128X_IRQ_RANGING_MASTER_ERROR_CODE,
    SX128X_IRQ_RANGING_MASTER_VALID_CODE,
  } SX128X_IrqRangingCode_t;

  /*!
   * \brief The radio callbacks structure
   * Holds function pointers to be called on radio interrupts
   */
  typedef struct
  {
    void (*txDone) (); //!< Pointer to a function run on successful transmission
    void (*rxDone) (uint8_t*, uint16_t, int8_t, int8_t); //!< Pointer to a function run on successful reception
    void (*rxSyncWordDone) (); //!< Pointer to a function run on successful SyncWord reception
    void (*rxHeaderDone) (); //!< Pointer to a function run on successful Header reception
    void (*txTimeout) (); //!< Pointer to a function run on transmission timeout
    void (*rxTimeout) (); //!< Pointer to a function run on reception timeout
    void (*rxError) (SX128X_IrqErrorCode_t errCode); //!< Pointer to a function run on reception error
    void (*rangingDone) (SX128X_IrqRangingCode_t val); //!< Pointer to a function run on ranging terminated
    void (*cadDone) (bool cadFlag); //!< Pointer to a function run on channel activity detected
  } SX128X_Callbacks_t;

  /*!
   * \brief The type describing the packet parameters for every packet types
   */
  typedef struct
  {
    SX128X_RadioPacketTypes_t PacketType; //!< Packet to which the packet parameters are referring to.
//    union
    struct
    {
      /*!
       * \brief Holds the GFSK packet parameters
       */
      struct
      {
        SX128X_RadioPreambleLengths_t PreambleLength; //!< The preamble length for GFSK packet type
        SX128X_RadioSyncWordLengths_t SyncWordLength; //!< The synchronization word length for GFSK packet type
        SX128X_RadioSyncWordRxMatchs_t SyncWordMatch; //!< The synchronization correlator to use to check synchronization word
        SX128X_RadioPacketLengthModes_t HeaderType; //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
        uint8_t PayloadLength;     //!< Size of the payload in the GFSK packet
        SX128X_RadioCrcTypes_t CrcLength; //!< Size of the CRC block in the GFSK packet
        SX128X_RadioWhiteningModes_t Whitening; //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
      } Gfsk;
      /*!
       * \brief Holds the LORA packet parameters
       */
      struct
      {
        uint8_t PreambleLength; //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
        SX128X_RadioLoRaPacketLengthsModes_t HeaderType; //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted
        uint8_t PayloadLength;     //!< Size of the payload in the LORA packet
        SX128X_RadioLoRaCrcModes_t CrcMode; //!< Size of CRC block in LORA packet
        SX128X_RadioLoRaIQModes_t InvertIQ; //!< Allows to swap IQ for LORA packet
      } LoRa;
      /*!
       * \brief Holds the FLRC packet parameters
       */
      struct
      {
        SX128X_RadioPreambleLengths_t PreambleLength; //!< The preamble length for FLRC packet type
        SX128X_RadioFlrcSyncWordLengths_t SyncWordLength; //!< The synchronization word length for FLRC packet type
        SX128X_RadioSyncWordRxMatchs_t SyncWordMatch; //!< The synchronization correlator to use to check synchronization word
        SX128X_RadioPacketLengthModes_t HeaderType; //!< If the header is explicit, it will be transmitted in the FLRC packet. If the header is implicit, it will not be transmitted.
        uint8_t PayloadLength;     //!< Size of the payload in the FLRC packet
        SX128X_RadioCrcTypes_t CrcLength; //!< Size of the CRC block in the FLRC packet
        SX128X_RadioWhiteningModes_t Whitening; //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
      } Flrc;
      /*!
       * \brief Holds the BLE packet parameters
       */
      struct
      {
        SX128X_RadioBleConnectionStates_t ConnectionState;   //!< The BLE state
        SX128X_RadioBleCrcFields_t CrcField; //!< Size of the CRC block in the BLE packet
        SX128X_RadioBlePacketTypes_t BlePacketType; //!< Special BLE packet types
        SX128X_RadioWhiteningModes_t Whitening; //!< Usage of whitening on PDU and CRC blocks of BLE packet
      } Ble;
    } Params;                         //!< Holds the packet parameters structure
  } SX128X_PacketParams_t;

  /*!
   * \brief Represents the packet status for every packet type
   */
  typedef struct
  {
    SX128X_RadioPacketTypes_t packetType; //!< Packet to which the packet status are referring to.
    union
    {
      struct
      {
        int8_t RssiAvg;                                 //!< The averaged RSSI
        int8_t RssiSync;                   //!< The RSSI measured on last packet
        struct
        {
          bool SyncError :1;                  //!< SyncWord error on last packet
          bool LengthError :1;                  //!< Length error on last packet
          bool CrcError :1;                        //!< CRC error on last packet
          bool AbortError :1;                    //!< Abort error on last packet
          bool HeaderReceived :1;            //!< Header received on last packet
          bool PacketReceived :1;                     //!< Packet received
          bool PacketControlerBusy :1;               //!< Packet controller busy
        } ErrorStatus;                                //!< The error status Byte
        struct
        {
          bool RxNoAck :1; //!< No acknowledgment received for Rx with variable length packets
          bool PacketSent :1;         //!< Packet sent, only relevant in Tx mode
        } TxRxStatus;                                 //!< The Tx/Rx status Byte
        uint8_t SyncAddrStatus :3; //!< The id of the correlator who found the packet
      } Gfsk;
      struct
      {
        int8_t RssiPkt;                         //!< The RSSI of the last packet
        int8_t SnrPkt;                           //!< The SNR of the last packet
        struct
        {
          bool SyncError :1;                  //!< SyncWord error on last packet
          bool LengthError :1;                  //!< Length error on last packet
          bool CrcError :1;                        //!< CRC error on last packet
          bool AbortError :1;                    //!< Abort error on last packet
          bool HeaderReceived :1;            //!< Header received on last packet
          bool PacketReceived :1;                     //!< Packet received
          bool PacketControlerBusy :1;               //!< Packet controller busy
        } ErrorStatus;                                //!< The error status Byte
        struct
        {
          bool RxNoAck :1; //!< No acknowledgment received for Rx with variable length packets
          bool PacketSent :1;         //!< Packet sent, only relevant in Tx mode
        } TxRxStatus;                                 //!< The Tx/Rx status Byte
        uint8_t SyncAddrStatus :3; //!< The id of the correlator who found the packet
      } LoRa;
      struct
      {
        int8_t RssiAvg;                                 //!< The averaged RSSI
        int8_t RssiSync;                        //!< The RSSI of the last packet
        struct
        {
          bool SyncError :1;                  //!< SyncWord error on last packet
          bool LengthError :1;                  //!< Length error on last packet
          bool CrcError :1;                        //!< CRC error on last packet
          bool AbortError :1;                    //!< Abort error on last packet
          bool HeaderReceived :1;            //!< Header received on last packet
          bool PacketReceived :1;                     //!< Packet received
          bool PacketControlerBusy :1;               //!< Packet controller busy
        } ErrorStatus;                                //!< The error status Byte
        struct
        {
          uint8_t RxPid :2;                           //!< PID of the Rx
          bool RxNoAck :1; //!< No acknowledgment received for Rx with variable length packets
          bool RxPidErr :1;                           //!< Received PID error
          bool PacketSent :1;         //!< Packet sent, only relevant in Tx mode
        } TxRxStatus;                                 //!< The Tx/Rx status Byte
        uint8_t SyncAddrStatus :3; //!< The id of the correlator who found the packet
      } Flrc;
      struct
      {
        int8_t RssiAvg;                                 //!< The averaged RSSI
        int8_t RssiSync;                        //!< The RSSI of the last packet
        struct
        {
          bool SyncError :1;                  //!< SyncWord error on last packet
          bool LengthError :1;                  //!< Length error on last packet
          bool CrcError :1;                        //!< CRC error on last packet
          bool AbortError :1;                    //!< Abort error on last packet
          bool HeaderReceived :1;            //!< Header received on last packet
          bool PacketReceived :1;                     //!< Packet received
          bool PacketControlerBusy :1;               //!< Packet controller busy
        } ErrorStatus;                                //!< The error status Byte
        struct
        {
          bool PacketSent :1;         //!< Packet sent, only relevant in Tx mode
        } TxRxStatus;                                 //!< The Tx/Rx status Byte
        uint8_t SyncAddrStatus :3; //!< The id of the correlator who found the packet
      } Ble;
    } Params;
  } SX128X_PacketStatus_t;

  /*!
   * \brief Represents the Rx internal counters values when GFSK or LORA packet type is used
   */
  typedef struct
  {
    SX128X_RadioPacketTypes_t packetType; //!< Packet to which the packet status are referring to.
    union
    {
      struct
      {
        uint16_t PacketReceived;                 //!< Number of received packets
        uint16_t CrcError;                             //!< Number of CRC errors
        uint16_t LengthError;                       //!< Number of length errors
        uint16_t SyncwordError;                  //!< Number of sync-word errors
      } Gfsk;
      struct
      {
        uint16_t PacketReceived;                 //!< Number of received packets
        uint16_t CrcError;                             //!< Number of CRC errors
        uint16_t HeaderValid;                       //!< Number of valid headers
      } LoRa;
    } Params;
  } SX128X_RxCounter_t;

  /*!
   * \brief Represents a calibration configuration
   */
  typedef struct
  {
    uint8_t RC64KEnable :1;                           //!< Calibrate RC64K clock
    uint8_t RC13MEnable :1;                           //!< Calibrate RC13M clock
    uint8_t PLLEnable :1;                             //!< Calibrate PLL
    uint8_t ADCPulseEnable :1;                          //!< Calibrate ADC Pulse
    uint8_t ADCBulkNEnable :1;                          //!< Calibrate ADC bulkN
    uint8_t ADCBulkPEnable :1;                          //!< Calibrate ADC bulkP
  } SX128X_CalibrationParams_t;

  typedef uint32_t (*sx128x_dev_write) (uint8_t*, uint16_t);
  typedef uint32_t (*sx128x_dev_read) (uint8_t*, uint16_t);
  typedef uint32_t (*sx128x_dev_write_read) (uint8_t*, uint8_t*, uint16_t);
  typedef void (*sx128x_dev_delay) (uint32_t);
  typedef void (*sx128x_gpio_operation) (void);

  typedef struct
  {
    /*
     *  配置
     */
    sx128x_dev_write write_buffer;
    sx128x_dev_read read_buffer;
    sx128x_dev_write_read write_read_buffer;
    sx128x_dev_delay delay_ms;
    sx128x_gpio_operation wait_on_busy;
    sx128x_gpio_operation cs_low;
    sx128x_gpio_operation cs_high;
    sx128x_gpio_operation rst_low;
    sx128x_gpio_operation rst_high;
    sx128x_gpio_operation tcxo_enable;
    sx128x_gpio_operation tcxo_disable;
    sx128x_gpio_operation ant_sw_tx;
    sx128x_gpio_operation ant_sw_rx;
    SX128X_PacketParams_t packet_params;
    SX128X_ModulationParams_t modulation_params;
    SX128X_RadioRegulatorModes_t regulator_mode;
    int8_t tx_power;
    /*
     *  状态
     */
    SX128X_RadioPacketTypes_t packet_type;
    SX128X_PacketStatus_t packet_status;
    SX128X_RadioOperatingModes_t op_mode;
    SX128X_RadioLoRaPacketLengthsModes_t lora_header_type;
    SX128X_RadioLoRaBandwidths_t lora_bandwidth;

    /*!
     * \brief Holds the polling state of the driver
     */
    bool polling_mode;
    bool irq_state;
    SX128X_Callbacks_t callbacks;

    uint32_t frequency_error;
    uint32_t calibrated_freq;
    uint8_t radio_rx_payload[256];
    uint16_t radio_rx_payload_len;
    uint32_t dio1_tick;
    uint32_t dio2_tick;
    uint32_t dio3_tick;
    uint32_t tx_done_tick;
    uint32_t rx_done_tick;
  } sx128x_dev;

  /*!
   * \brief Represents a sleep mode configuration
   */
  typedef struct
  {
    uint8_t WakeUpRTC :1; //!< Get out of sleep mode if wakeup signal received from RTC
    uint8_t InstructionRamRetention :1; //!< InstructionRam is conserved during sleep
    uint8_t DataBufferRetention :1;   //!< Data buffer is conserved during sleep
    uint8_t DataRamRetention :1;         //!< Data ram is conserved during sleep
  } SX128X_SleepParams_t;

  /*!
   * \brief Compute the two's complement for a register of size lower than
   *        32bits
   *
   * \param [in]  num            The register to be two's complemented
   * \param [in]  bitCnt         The position of the sign bit
   */
  int32_t sx128x_complement2 (const uint32_t num, const uint8_t bitCnt);

  /*!
   * \brief Returns the value of LoRa bandwidth from driver's value
   *
   * The value is returned in Hz so that it can be represented as an integer
   * type. Most computation should be done as integer to reduce floating
   * point related errors.
   *
   * \retval loRaBw              The value of the current bandwidth in Hz
   */
  int32_t sx128x_get_lora_bandwidth (sx128x_dev *dev);

  /*!
   * \brief Returns the corrected raw value of ranging
   *
   * \retval correction              Corrected ranging raw value
   */
  double sx128x_get_ranging_correction_per_sf_bw_gain (
      sx128x_dev *dev, const SX128X_RadioLoRaSpreadingFactors_t sf,
      const SX128X_RadioLoRaBandwidths_t bw, const int8_t gain);

  /*!
   * \brief Returns the short range corrected distance
   *
   * \retval Corrected Distance              corrected ditance
   */
  double sx128x_compute_ranging_correction_polynome (
      const SX128X_RadioLoRaSpreadingFactors_t sf,
      const SX128X_RadioLoRaBandwidths_t bw, const double median);

  /*!
   * \brief DIOs interrupt callback
   *
   * \remark Called to handle all 3 DIOs pins
   */
  void sx128x_on_dio_irq (sx128x_dev *dev);

  /*!
   * \brief Set the role of the radio during ranging operations
   *
   * \param [in]  role          Role of the radio
   */
  void sx128x_set_ranging_role (sx128x_dev *dev,
                                SX128X_RadioRangingRoles_t role);

  void sx128x_reset (sx128x_dev *dev);

  /*!
   * \brief Initializes the radio driver
   */
  bool sx128x_init (sx128x_dev *dev);

  /*!
   * \brief Set the driver in interrupt mode.
   *
   * In interrupt mode, the driver communicate with the radio during the
   * interruption by direct calls to ProcessIrqs( ). The main advantage is
   * the possibility to have low power application architecture.
   * This is the default mode.
   * @code
   * // Initializations and callbacks declaration/definition
   * radio = SX1280( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
   * radio.Init( );
   * radio.SetInterruptMode( );   // Optionnal. Driver default behavior
   *
   * while( true )
   * {
   *     // Do some applicative work
   * }
   * @endcode
   *
   * \see SX1280SetPollingMode
   */
  void sx128x_set_interrupt_mode (sx128x_dev *dev);

  /*!
   * \brief Initializes the radio registers to the recommended default values
   */
  void sx128x_set_registers_default (sx128x_dev *dev);

  /*!
   * \brief Returns the current device firmware version
   *
   * \retval      version       Firmware version
   */
  uint16_t sx128x_get_firmware_version (sx128x_dev *dev);

  /*!
   * \brief Gets the current radio status
   *
   * \retval      status        Radio status
   */
  SX128X_RadioStatus_t sx128x_get_status (sx128x_dev *dev);

  /*!
   * \brief Sets the radio in sleep mode
   *
   * \param [in]  sleepConfig   The sleep configuration describing data
   *                            retention and RTC wake-up
   */
  void sx128x_set_sleep (sx128x_dev *dev, SX128X_SleepParams_t sleepConfig);

  /*!
   * \brief Sets the radio in configuration mode
   *
   * \param [in]  mode          The standby mode to put the radio into
   */
  void sx128x_set_standby (sx128x_dev *dev, SX128X_RadioStandbyModes_t mode);

  /*!
   * \brief Sets the radio in FS mode
   */
  void sx128x_set_fs (sx128x_dev *dev);

  /*!
   * \brief Sets the radio in transmission mode
   *
   * \param [in]  timeout       Structure describing the transmission timeout value
   */
  void sx128x_set_tx (sx128x_dev *dev, SX128X_TickTime_t timeout);

  /*!
   * \brief Sets the radio in reception mode
   *
   * \param [in]  timeout       Structure describing the reception timeout value
   */
  void sx128x_set_rx (sx128x_dev *dev, SX128X_TickTime_t timeout);

  /*!
   * \brief Sets the Rx duty cycle management parameters
   *
   * \param [in]  rxTime        Structure describing reception timeout value
   * \param [in]  sleepTime     Structure describing sleep timeout value
   */
  void sx128x_set_rx_duty_cycle (sx128x_dev *dev, SX128X_RadioTickSizes_t Step,
                                 uint16_t NbStepRx, uint16_t RxNbStepSleep);

  /*!
   * \brief Sets the radio in CAD mode
   *
   * \see SX1280::SetCadParams
   */
  void sx128x_set_cad (sx128x_dev *dev);

  /*!
   * \brief Sets the radio in continuous wave transmission mode
   */
  void sx128x_set_tx_continuous_wave (sx128x_dev *dev);

  /*!
   * \brief Sets the radio in continuous preamble transmission mode
   */
  void sx128x_set_tx_continuous_preamble (sx128x_dev *dev);

  /*!
   * \brief Sets the radio for the given protocol
   *
   * \param [in]  packetType    [SX128x_PACKET_TYPE_GFSK, SX128x_PACKET_TYPE_LORA,
   *                             SX128x_PACKET_TYPE_RANGING, SX128x_PACKET_TYPE_FLRC,
   *                             SX128x_PACKET_TYPE_BLE]
   *
   * \remark This method has to be called before SetRfFrequency,
   *         SetModulationParams and SetPacketParams
   */
  void sx128x_set_packet_type (sx128x_dev *dev,
                               SX128X_RadioPacketTypes_t packetType);

  /*!
   * \brief Gets the current radio protocol
   *
   * \retval      packetType    [SX128x_PACKET_TYPE_GFSK, SX128x_PACKET_TYPE_LORA,
   *                             SX128x_PACKET_TYPE_RANGING, SX128x_PACKET_TYPE_FLRC,
   *                             SX128x_PACKET_TYPE_BLE, SX128x_PACKET_TYPE_NONE]
   */
  SX128X_RadioPacketTypes_t sx128x_get_packet_type (sx128x_dev *dev);

  /*!
   * \brief Sets the RF frequency
   *
   * \param [in]  frequency     RF frequency [Hz]
   */
  void sx128x_set_rf_frequency (sx128x_dev *dev, uint32_t frequency);

  /*!
   * \brief Sets the transmission parameters
   *
   * \param [in]  power         RF output power [-18..13] dBm
   * \param [in]  rampTime      Transmission ramp up time
   */
  void sx128x_set_tx_params (sx128x_dev *dev, int8_t power,
                             SX128X_RadioRampTimes_t rampTime);

  /*!
   * \brief Sets the number of symbols to be used for Channel Activity
   *        Detection operation
   *
   * \param [in]  cadSymbolNum  The number of symbol to use for Channel Activity
   *                            Detection operations [SX128x_LORA_CAD_01_SYMBOL, SX128x_LORA_CAD_02_SYMBOL,
   *                            SX128x_LORA_CAD_04_SYMBOL, SX128x_LORA_CAD_08_SYMBOL, SX128x_LORA_CAD_16_SYMBOL]
   */
  void sx128x_set_cad_params (sx128x_dev *dev,
                              SX128X_RadioLoRaCadSymbols_t cadSymbolNum);

  /*!
   * \brief Sets the data buffer base address for transmission and reception
   *
   * \param [in]  txBaseAddress Transmission base address
   * \param [in]  rxBaseAddress Reception base address
   */
  void sx128x_set_buffer_base_addresses (sx128x_dev *dev, uint8_t txBaseAddress,
                                         uint8_t rxBaseAddress);

  /*!
   * \brief Set the modulation parameters
   *
   * \param [in]  modParams     A structure describing the modulation parameters
   */
  void sx128x_set_modulation_params (sx128x_dev *dev,
                                     SX128X_ModulationParams_t *modParams);

  /*!
   * \brief Sets the packet parameters
   *
   * \param [in]  packetParams  A structure describing the packet parameters
   */
  void sx128x_set_packet_params (sx128x_dev *dev,
                              SX128X_PacketParams_t *packetParams);

  /*!
   * \brief Gets the last received packet buffer status
   *
   * \param [out] payloadLength Last received packet payload length
   * \param [out] rxStartBuffer Last received packet buffer address pointer
   */
  void sx128x_get_rx_nuffer_status (sx128x_dev *dev, uint8_t *payloadLength,
                                uint8_t *rxStartBuffer);

  /*!
   * \brief Gets the last received packet payload length
   *
   * \param [out] pktStatus     A structure of packet status
   */
  void sx128x_get_packet_status (sx128x_dev *dev,
                              SX128X_PacketStatus_t *pktStatus);

  /*!
   * \brief Returns the instantaneous RSSI value for the last packet received
   *
   * \retval      rssiInst      Instantaneous RSSI
   */
  int8_t sx128x_get_rssi_inst (sx128x_dev *dev);

  /*!
   * \brief   Sets the IRQ mask and DIO masks
   *
   * \param [in]  irqMask       General IRQ mask
   * \param [in]  dio1Mask      DIO1 mask
   * \param [in]  dio2Mask      DIO2 mask
   * \param [in]  dio3Mask      DIO3 mask
   */
  void sx128x_set_dio_irq_params (sx128x_dev *dev, uint16_t irqMask,
                              uint16_t dio1Mask, uint16_t dio2Mask,
                              uint16_t dio3Mask);

  /*!
   * \brief Returns the current IRQ status
   *
   * \retval      irqStatus     IRQ status
   */
  uint16_t sx128x_get_irq_status (sx128x_dev *dev);

  /*!
   * \brief Clears the IRQs
   *
   * \param [in]  irq           IRQ(s) to be cleared
   */
  void sx128x_clear_irq_status (sx128x_dev *dev, uint16_t irq);

  /*!
   * \brief Calibrates the given radio block
   *
   * \param [in]  calibParam    The description of blocks to be calibrated
   */
  void sx128x_calibrate (sx128x_dev *dev, SX128X_CalibrationParams_t calibParam);

  /*!
   * \brief Sets the power regulators operating mode
   *
   * \param [in]  mode          [0: LDO, 1:DC_DC]
   */
  void sx128x_set_regulator_mode (sx128x_dev *dev,
                               SX128X_RadioRegulatorModes_t mode);

  /*!
   * \brief Saves the current selected modem configuration into data RAM
   */
  void sx128x_set_save_context (sx128x_dev *dev);

  /*!
   * \brief Sets the chip to automatically send a packet after the end of a packet reception
   *
   * \remark The offset is automatically compensated inside the function
   *
   * \param [in]  time          The delay in us after which a Tx is done
   */
  void sx128x_set_auto_tx (sx128x_dev *dev, uint16_t time);

  /*!
   * \brief Stop the chip from automatically sending a packet after the end of a packet reception
   * if previously activated with SX1280SetAutoTx command
   *
   * \see SX1280SetAutoTx
   */
  void sx128x_stop_auto_tx (sx128x_dev *dev);

  /*!
   * \brief Sets the chip to automatically receive a packet after the end of a packet transmission
   *
   * \remark The offset is automatically compensated inside the function
   *
   * \param [in]  time          The delay in us after which a Rx is done
   */
  void sx128x_set_auto_fs (sx128x_dev *dev, uint8_t enable);

  /*!
   * \brief Enables or disables long preamble detection mode
   *
   * \param [in]  enable        [0: Disable, 1: Enable]
   */
  void sx128x_set_long_preamble (sx128x_dev *dev, uint8_t enable);

  /*!
   * \brief Saves the payload to be send in the radio buffer
   *
   * \param [in]  payload       A pointer to the payload
   * \param [in]  size          The size of the payload
   */
  void sx128x_set_payload (sx128x_dev *dev, uint8_t *payload, uint8_t size);

  /*!
   * \brief Reads the payload received. If the received payload is longer
   * than maxSize, then the method returns 1 and do not set size and payload.
   *
   * \param [out] payload       A pointer to a buffer into which the payload will be copied
   * \param [out] size          A pointer to the size of the payload received
   * \param [in]  maxSize       The maximal size allowed to copy into the buffer
   */
  uint8_t sx128x_get_payload (sx128x_dev *dev, uint8_t *payload, uint8_t *size,
                              uint8_t maxSize);

  /*!
   * \brief Sends a payload
   *
   * \param [in]  payload       A pointer to the payload to send
   * \param [in]  size          The size of the payload to send
   * \param [in]  timeout       The timeout for Tx operation
   */
  void sx128x_send_payload (sx128x_dev *dev, uint8_t *payload, uint8_t size,
                            SX128X_TickTime_t timeout);

  /*!
   * \brief Sets the Sync Word given by index used in GFSK, FLRC and BLE protocols
   *
   * \remark 5th byte isn't used in FLRC and BLE protocols
   *
   * \param [in]  syncWordIdx   Index of SyncWord to be set [1..3]
   * \param [in]  syncWord      SyncWord bytes ( 5 bytes )
   *
   * \retval      status        [0: OK, 1: NOK]
   */
  uint8_t sx128x_set_sync_word (sx128x_dev *dev, uint8_t syncWordIdx,
                                uint8_t *syncWord);

  /*!
   * \brief Defines how many error bits are tolerated in sync word detection
   *
   * \param [in]  errorBits     Number of error bits supported to validate the Sync word detection
   *                            ( default is 4 bit, minimum is 1 bit )
   */
  void sx128x_set_sync_word_error_tolerance (sx128x_dev *dev,
                                             uint8_t errorBits);

  /*!
   * \brief Sets the Initial value for the LFSR used for the CRC calculation
   *
   * \param [in]  seed          Initial LFSR value ( 4 bytes )
   *
   */
  void sx128x_set_crc_seed (sx128x_dev *dev, uint16_t seed);

  /*!
   * \brief Set the Access Address field of BLE packet
   *
   * \param [in]  accessAddress The access address to be used for next BLE packet sent
   */
  void sx128x_set_ble_access_address (sx128x_dev *dev, uint32_t accessAddress);

  /*!
   * \brief Set the Access Address for Advertizer BLE packets
   *
   * All advertizer BLE packets must use a particular value for Access
   * Address field. This method sets it.
   *
   * \see SX1280::SetBleAccessAddress
   */
  void sx128x_set_ble_advertizer_access_address (sx128x_dev *dev);

  /*!
   * \brief Sets the seed used for the CRC calculation
   *
   * \param [in]  seed          The seed value
   *
   */
  void sx128x_set_crc_polynomial (sx128x_dev *dev, uint16_t seed);

  /*!
   * \brief Sets the Initial value of the LFSR used for the whitening in GFSK, FLRC and BLE protocols
   *
   * \param [in]  seed          Initial LFSR value
   */
  void sx128x_set_whitening_seed (sx128x_dev *dev, uint8_t seed);

  /*!
   * \brief Enable manual gain and disable AGC
   *
   * \see SX1280SetManualGainValue, SX1280DisableManualGain
   */
  void sx128x_enable_manual_gain (sx128x_dev *dev);

  /*!
   * \brief Disable the manual gain control and enable AGC
   *
   * \see SX1280EnableManualGain
   */
  void sx128x_disable_manual_gain (sx128x_dev *dev);

  /*!
   * \brief Set the gain for LNA
   *
   * SX1280EnableManualGain must be call before using this function
   *
   * \param [in]  gain          The value of gain to set, refer to datasheet for value meaning
   *
   * \see SX1280EnableManualGain, SX1280DisableManualGain
   */
  void sx128x_set_manual_gain_value (sx128x_dev *dev, uint8_t gain);

  /*!
   * \brief Configure the LNA regime of operation
   *
   * \param [in]  lnaSetting    The LNA setting. Possible values are
   *                            LNA_LOW_POWER_MODE and
   *                            LNA_HIGH_SENSITIVITY_MODE
   */
  void sx128x_set_lna_gain_setting (sx128x_dev *dev,
                                    const SX128X_RadioLnaSettings_t lnaSetting);

  /*!
   * \brief Sets the number of bits used to check that ranging request match ranging ID
   *
   * \param [in]  length        [0: 8 bits, 1: 16 bits,
   *                             2: 24 bits, 3: 32 bits]
   */
  void sx128x_set_ranging_id_length (
      sx128x_dev *dev, SX128X_RadioRangingIdCheckLengths_t length);

  /*!
   * \brief Sets ranging device id
   *
   * \param [in]  address       Device address
   */
  void sx128x_set_device_ranging_address (sx128x_dev *dev, uint32_t address);

  /*!
   * \brief Sets the device id to ping in a ranging request
   *
   * \param [in]  address       Address of the device to ping
   */
  void sx128x_set_ranging_request_address (sx128x_dev *dev, uint32_t address);

  /*!
   * \brief Return the ranging result value
   *
   * \param [in]  resultType    Specifies the type of result.
   *                            [0: RAW, 1: Averaged,
   *                             2: De-biased, 3:Filtered]
   *
   * \retval      ranging       The ranging measure filtered according to resultType [m]
   */
  double sx128x_get_ranging_result (
      sx128x_dev *dev, SX128X_RadioRangingResultTypes_t resultType);

  /*!
   * \brief Sets the standard processing delay between Master and Slave
   *
   * \param [in]  cal           RxTx delay offset for correcting ranging bias.
   *
   * The calibration value reflects the group delay of the radio front end and
   * must be re-performed for each new SX1280 PCB design. The value is obtained
   * empirically by either conducted measurement in a known electrical length
   * coaxial RF cable (where the design is connectorised) or by radiated
   * measurement, at a known distance, where an antenna is present.
   * The result of the calibration process is that the SX1280 ranging result
   * accurately reflects the physical range, the calibration procedure therefore
   * removes the average timing error from the time-of-flight measurement for a
   * given design.
   *
   * The values are Spreading Factor dependents, and depend also of the board
   * design. Some typical values are provided in the next table.
   *
   * Spreading Factor | Calibration Value
   * ---------------- | -----------------
   *   SF5            |  12200
   *   SF6            |  12200
   *   SF7            |  12400
   *   SF8            |  12650
   *   SF9            |  12940
   *   SF10           |  13000
   *   SF11           |  13060
   *   SF12           |  13120
   */
  void sx128x_set_ranging_calibration (sx128x_dev *dev, uint16_t cal);

  /*!
   * \brief Return the last ranging result power indicator
   *
   * The value returned is not an absolute power measurement. It is
   * a relative power measurement.
   *
   * \retval      deltaThreshold  A relative power indicator
   */
  uint8_t sx128x_get_ranging_power_delta_threshold_indicator (sx128x_dev *dev);

  /*!
   * \brief Clears the ranging filter
   */
  void sx128x_ranging_clear_filter_result (sx128x_dev *dev);

  /*!
   * \brief Set the number of samples considered in the built-in filter
   *
   * \param [in]  numSample     The number of samples to use built-in filter
   *                            [8..255]
   *
   * \remark Value inferior to 8 will be silently set to 8
   */
  void sx128x_ranging_set_filter_num_samples (sx128x_dev *dev,
                                              uint8_t numSample);

  /*!
   * \brief Return the Estimated Frequency Error in LORA and RANGING operations
   *
   * \retval efe                The estimated frequency error [Hz]
   */
  double sx128x_get_frequency_error (sx128x_dev *dev);

  /*!
   * \brief Process the analysis of radio IRQs and calls callback functions
   *        depending on radio state
   */
  void sx128x_process_irqs (sx128x_dev *dev);

  /*!
   * \brief Clears the instruction RAM
   */
  void SX128XClearInstructionRam (sx128x_dev *dev);

  /*!
   * \brief Parses 1 HEX file line and writes the content to the instruction memory
   *
   * \param [in]  line          HEX file line string
   *
   * \retval      status        [0: ERROR, 1:OK]
   */
  int8_t sx128x_parse_hex_file_line (sx128x_dev *dev, char *line);

  /*!
   * \brief Gets individual fields for the given HEX file line
   *
   * \param [in]  line          HEX file line string
   * \param [out] bytes         Bytes array to be written to the instruction memory
   * \param [out] addr          Instruction memory address where to write the bytes array
   * \param [out] num           Number of bytes in Bytes array
   * \param [out] code          HEX file line type [0: instruction, 1: end of file, 2: begin of file]
   *
   * \retval      status        [0: ERROR, 1:OK]
   */
  int8_t sx128x_get_hex_file_line_fields (sx128x_dev *dev, char *line,
                                          uint8_t *bytes, uint16_t *addr,
                                          uint16_t *num, uint8_t *code);

  /*!
   * \brief Clears the instruction ram memory block
   */
  void sx128x_clear_instruction_ram (sx128x_dev *sx1280);

  /*!
   * \brief Wakes up the radio
   */
  void sx128x_wakeup (sx128x_dev *sx1280);

  /*!
   * \brief Send a command that write data to the radio
   *
   * \param [in]  opcode        Opcode of the command
   * \param [in]  buffer        Buffer to be send to the radio
   * \param [in]  size          Size of the buffer to send
   */
  void sx128x_write_command (sx128x_dev *sx1280, uint8_t opcode,
                              uint8_t *buffer, uint16_t size);

  /*!
   * \brief Send a command that read data from the radio
   *
   * \param [in]  opcode        Opcode of the command
   * \param [out] buffer        Buffer holding data from the radio
   * \param [in]  size          Size of the buffer
   */
  uint8_t sx128x_read_command (sx128x_dev *sx1280, uint8_t opcode,
                             uint8_t *buffer, uint16_t size);

  /*!
   * \brief Write data to the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the radio
   * \param [in]  buffer        The data to be written in radio's memory
   * \param [in]  size          The number of bytes to write in radio's memory
   */
  void sx128x_write_registers (sx128x_dev *sx1280, uint16_t address,
                                uint8_t *buffer, uint16_t size);

  /*!
   * \brief Write a single byte of data to the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the radio
   * \param [in]  value         The data to be written in radio's memory
   */
  void sx128x_write_register (sx128x_dev *sx1280, uint16_t address,
                               uint8_t value);

  /*!
   * \brief Read data from the radio memory
   *
   * \param [in]  address       The address of the first byte to read from the radio
   * \param [out] buffer        The buffer that holds data read from radio
   * \param [in]  size          The number of bytes to read from radio's memory
   */
  void sx128x_read_registers (sx128x_dev *sx1280, uint16_t address,
                               uint8_t *buffer, uint16_t size);

  /*!
   * \brief Read a single byte of data from the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the
   *                            radio
   *
   * \retval      value         The value of the byte at the given address in
   *                            radio's memory
   */
  uint8_t sx128x_read_register (sx128x_dev *sx1280, uint16_t address);

  /*!
   * \brief Write data to the buffer holding the payload in the radio
   *
   * \param [in]  offset        The offset to start writing the payload
   * \param [in]  buffer        The data to be written (the payload)
   * \param [in]  size          The number of byte to be written
   */
  void sx128x_write_buffer (sx128x_dev *sx1280, uint8_t offset,
                             uint8_t *buffer, uint8_t size);

  /*!
   * \brief Read data from the buffer holding the payload in the radio
   *
   * \param [in]  offset        The offset to start reading the payload
   * \param [out] buffer        A pointer to a buffer holding the data from the radio
   * \param [in]  size          The number of byte to be read
   */
  void sx128x_read_buffer (sx128x_dev *sx1280, uint8_t offset, uint8_t *buffer,
                           uint8_t size);


#ifdef __cplusplus
}
#endif

#endif // __SX1280_H__
