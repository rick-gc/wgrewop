/*!
 * \file      sx126x.h
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
#ifndef __SX126x_H__
#define __SX126x_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef  enum
    {
      SX1261_DEVICE_ID = 1,
      SX1262_DEVICE_ID,
      SX1268_DEVICE_ID
    }sx126x_device_id;

  /*!
   * Radio complete Wake-up Time with margin for temperature compensation
   */
#define RADIO_WAKEUP_TIME                           3 // [ms]

  /*!
   * \brief Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
   */
#define AUTO_RX_TX_OFFSET                           2

  /*!
   * \brief LFSR initial value to compute IBM type CRC
   */
#define CRC_IBM_SEED                                0xFFFF

  /*!
   * \brief LFSR initial value to compute CCIT type CRC
   */
#define CRC_CCITT_SEED                              0x1D0F

  /*!
   * \brief Polynomial used to compute IBM CRC
   */
#define CRC_POLYNOMIAL_IBM                          0x8005

  /*!
   * \brief Polynomial used to compute CCIT CRC
   */
#define CRC_POLYNOMIAL_CCITT                        0x1021

  /*!
   * \brief The address of the register holding the first byte defining the CRC seed
   *
   */
#define REG_LR_CRCSEEDBASEADDR                      0x06BC

  /*!
   * \brief The address of the register holding the first byte defining the CRC polynomial
   */
#define REG_LR_CRCPOLYBASEADDR                      0x06BE

  /*!
   * \brief The address of the register holding the first byte defining the whitening seed
   */
#define REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9

  /*!
   * \brief The address of the register holding the packet configuration
   */
#define REG_LR_PACKETPARAMS                         0x0704

  /*!
   * \brief The address of the register holding the payload size
   */
#define REG_LR_PAYLOADLENGTH                        0x0702

  /*!
   * \brief The addresses of the registers holding SyncWords values
   */
#define REG_LR_SYNCWORDBASEADDRESS                  0x06C0

  /*!
   * \brief The addresses of the register holding LoRa Modem SyncWord value
   */
#define REG_LR_SYNCWORD                             0x0740

  /*!
   * Syncword for Private LoRa networks
   */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x1424

  /*!
   * Syncword for Public LoRa networks
   */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x3444

  /*!
   * The address of the register giving a 32-bit random number
   */
#define RANDOM_NUMBER_GENERATORBASEADDR             0x0819

  /*!
   * The address of the register used to disable the LNA
   */
#define REG_ANA_LNA                                 0x08E2

  /*!
   * The address of the register used to disable the mixer
   */
#define REG_ANA_MIXER                               0x08E5

  /*!
   * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
   */
#define REG_RX_GAIN                                 0x08AC

  /*!
   * Change the value on the device internal trimming capacitor
   */
#define REG_XTA_TRIM                                0x0911

  /*!
   * Set the current max value in the over current protection
   */
#define REG_OCP                                     0x08E7

  /*!
   * \brief Structure describing the radio status
   */
  typedef union RadioStatus_u
    {
      uint8_t Value;
      struct
        {   //bit order is lsb -> msb
          uint8_t : 1;//!< Reserved
          uint8_t CmdStatus : 3;//!< Command status
          uint8_t ChipMode : 3;//!< Chip mode
          uint8_t : 1;//!< Reserved
        }Fields;
    }RadioStatus_t;

  /*!
   * \brief Structure describing the error codes for callback functions
   */
  typedef enum
    {
      IRQ_HEADER_ERROR_CODE = 0x01,
      IRQ_SYNCWORD_ERROR_CODE = 0x02,
      IRQ_CRC_ERROR_CODE = 0x04,
    }IrqErrorCode_t;

  enum IrqPblSyncHeaderCode_t
    {
      IRQ_PBL_DETECT_CODE = 0x01,
      IRQ_SYNCWORD_VALID_CODE = 0x02,
      IRQ_HEADER_VALID_CODE = 0x04,
    };

  /*!
   * \brief Represents the operating mode the radio is actually running
   */
  typedef enum
    {
      MODE_SLEEP = 0x00,         //! The radio is in sleep mode
      MODE_STDBY_RC,//! The radio is in standby mode with RC oscillator
      MODE_STDBY_XOSC,//! The radio is in standby mode with XOSC oscillator
      MODE_FS,//! The radio is in frequency synthesis mode
      MODE_TX,//! The radio is in transmit mode
      MODE_RX,//! The radio is in receive mode
      MODE_RX_DC,//! The radio is in receive duty cycle mode
      MODE_CAD//! The radio is in channel activity detection mode
    }RadioOperatingModes_t;

  /*!
   * \brief Declares the oscillator in use while in standby mode
   *
   * Using the STDBY_RC standby mode allow to reduce the energy consumption
   * STDBY_XOSC should be used for time critical applications
   */
  typedef enum
    {
      STDBY_RC = 0x00,
      STDBY_XOSC = 0x01,
    }RadioStandbyModes_t;

  /*!
   * \brief Declares the power regulation used to power the device
   *
   * This command allows the user to specify if DC-DC or LDO is used for power regulation.
   * Using only LDO implies that the Rx or Tx current is doubled
   */
  typedef enum
    {
      USE_LDO = 0x00, // default
      USE_DCDC = 0x01,
    }RadioRegulatorMode_t;

  /*!
   * \brief Represents the possible packet type (i.e. modem) used
   */
  typedef enum
    {
      PACKET_TYPE_GFSK = 0x00,
      PACKET_TYPE_LORA = 0x01,
      PACKET_TYPE_NONE = 0xFF,
    }RadioPacketTypes_t;

  /*!
   * \brief Represents the ramping time for power amplifier
   */
  typedef enum
    {
      RADIO_RAMP_10_US = 0x00,
      RADIO_RAMP_20_US = 0x01,
      RADIO_RAMP_40_US = 0x02,
      RADIO_RAMP_80_US = 0x03,
      RADIO_RAMP_200_US = 0x04,
      RADIO_RAMP_800_US = 0x05,
      RADIO_RAMP_1700_US = 0x06,
      RADIO_RAMP_3400_US = 0x07,
    }RadioRampTimes_t;

  /*!
   * \brief Represents the number of symbols to be used for channel activity detection operation
   */
  typedef enum
    {
      LORA_CAD_01_SYMBOL = 0x00,
      LORA_CAD_02_SYMBOL = 0x01,
      LORA_CAD_04_SYMBOL = 0x02,
      LORA_CAD_08_SYMBOL = 0x03,
      LORA_CAD_16_SYMBOL = 0x04,
    }RadioLoRaCadSymbols_t;

  /*!
   * \brief Represents the Channel Activity Detection actions after the CAD operation is finished
   */
  typedef enum
    {
      LORA_CAD_ONLY = 0x00,
      LORA_CAD_RX = 0x01,
      LORA_CAD_LBT = 0x10,
    }RadioCadExitModes_t;

  /*!
   * \brief Represents the modulation shaping parameter
   */
  typedef enum
    {
      MOD_SHAPING_OFF = 0x00,
      MOD_SHAPING_G_BT_03 = 0x08,
      MOD_SHAPING_G_BT_05 = 0x09,
      MOD_SHAPING_G_BT_07 = 0x0A,
      MOD_SHAPING_G_BT_1 = 0x0B,
    }RadioModShapings_t;

  /*!
   * \brief Represents the modulation shaping parameter
   */
  typedef enum
    {
      RX_BW_4800 = 0x1F,
      RX_BW_5800 = 0x17,
      RX_BW_7300 = 0x0F,
      RX_BW_9700 = 0x1E,
      RX_BW_11700 = 0x16,
      RX_BW_14600 = 0x0E,
      RX_BW_19500 = 0x1D,
      RX_BW_23400 = 0x15,
      RX_BW_29300 = 0x0D,
      RX_BW_39000 = 0x1C,
      RX_BW_46900 = 0x14,
      RX_BW_58600 = 0x0C,
      RX_BW_78200 = 0x1B,
      RX_BW_93800 = 0x13,
      RX_BW_117300 = 0x0B,
      RX_BW_156200 = 0x1A,
      RX_BW_187200 = 0x12,
      RX_BW_234300 = 0x0A,
      RX_BW_312000 = 0x19,
      RX_BW_373600 = 0x11,
      RX_BW_467000 = 0x09,
    }RadioRxBandwidth_t;

  /*!
   * \brief Represents the possible spreading factor values in LoRa packet types
   */
  typedef enum
    {
      LORA_SF5 = 0x05,
      LORA_SF6 = 0x06,
      LORA_SF7 = 0x07,
      LORA_SF8 = 0x08,
      LORA_SF9 = 0x09,
      LORA_SF10 = 0x0A,
      LORA_SF11 = 0x0B,
      LORA_SF12 = 0x0C,
    }RadioLoRaSpreadingFactors_t;

  /*!
   * \brief Represents the bandwidth values for LoRa packet type
   */
  typedef enum
    {
      LORA_BW_500 = 6,
      LORA_BW_250 = 5,
      LORA_BW_125 = 4,
      LORA_BW_062 = 3,
      LORA_BW_041 = 10,
      LORA_BW_031 = 2,
      LORA_BW_020 = 9,
      LORA_BW_015 = 1,
      LORA_BW_010 = 8,
      LORA_BW_007 = 0,
    }RadioLoRaBandwidths_t;

  /*!
   * \brief Represents the coding rate values for LoRa packet type
   */
  typedef enum
    {
      LORA_CR_4_5 = 0x01,
      LORA_CR_4_6 = 0x02,
      LORA_CR_4_7 = 0x03,
      LORA_CR_4_8 = 0x04,
    }RadioLoRaCodingRates_t;

  /*!
   * \brief Represents the preamble length used to detect the packet on Rx side
   */
  typedef enum
    {
      RADIO_PREAMBLE_DETECTOR_OFF = 0x00,     //!< Preamble detection length off
      RADIO_PREAMBLE_DETECTOR_08_BITS = 0x04,//!< Preamble detection length 8 bits
      RADIO_PREAMBLE_DETECTOR_16_BITS = 0x05,//!< Preamble detection length 16 bits
      RADIO_PREAMBLE_DETECTOR_24_BITS = 0x06,//!< Preamble detection length 24 bits
      RADIO_PREAMBLE_DETECTOR_32_BITS = 0x07,//!< Preamble detection length 32 bit
    }RadioPreambleDetection_t;

  /*!
   * \brief Represents the possible combinations of SyncWord correlators activated
   */
  typedef enum
    {
      RADIO_ADDRESSCOMP_FILT_OFF = 0x00, //!< No correlator turned on, i.e. do not search for SyncWord
      RADIO_ADDRESSCOMP_FILT_NODE = 0x01,
      RADIO_ADDRESSCOMP_FILT_NODE_BROAD = 0x02,
    }RadioAddressComp_t;

  /*!
   *  \brief Radio GFSK packet length mode
   */
  typedef enum
    {
      RADIO_PACKET_FIXED_LENGTH = 0x00, //!< The packet is known on both sides, no header included in the packet
      RADIO_PACKET_VARIABLE_LENGTH = 0x01,//!< The packet is on variable size, header included
    }RadioPacketLengthModes_t;

  /*!
   * \brief Represents the CRC length
   */
  typedef enum
    {
      RADIO_CRC_OFF = 0x01,         //!< No CRC in use
      RADIO_CRC_1_BYTES = 0x00,
      RADIO_CRC_2_BYTES = 0x02,
      RADIO_CRC_1_BYTES_INV = 0x04,
      RADIO_CRC_2_BYTES_INV = 0x06,
      RADIO_CRC_2_BYTES_IBM = 0xF1,
      RADIO_CRC_2_BYTES_CCIT = 0xF2,
    }RadioCrcTypes_t;

  /*!
   * \brief Radio whitening mode activated or deactivated
   */
  typedef enum
    {
      RADIO_DC_FREE_OFF = 0x00,
      RADIO_DC_FREEWHITENING = 0x01,
    }RadioDcFree_t;

  /*!
   * \brief Holds the Radio lengths mode for the LoRa packet type
   */
  typedef enum
    {
      LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
      LORA_PACKET_FIXED_LENGTH = 0x01,//!< The packet is known on both sides, no header included in the packet
      LORA_PACKET_EXPLICIT = LORA_PACKET_VARIABLE_LENGTH,
      LORA_PACKET_IMPLICIT = LORA_PACKET_FIXED_LENGTH,
    }RadioLoRaPacketLengthsMode_t;

  /*!
   * \brief Represents the CRC mode for LoRa packet type
   */
  typedef enum
    {
      LORA_CRC_ON = 0x01,         //!< CRC activated
      LORA_CRC_OFF = 0x00,//!< CRC not used
    }RadioLoRaCrcModes_t;

  /*!
   * \brief Represents the IQ mode for LoRa packet type
   */
  typedef enum
    {
      LORA_IQ_NORMAL = 0x00,
      LORA_IQ_INVERTED = 0x01,
    }RadioLoRaIQModes_t;

  /*!
   * \brief Represents the voltage used to control the TCXO on/off from DIO3
   */
  typedef enum
    {
      TCXO_CTRL_1_6V = 0x00,
      TCXO_CTRL_1_7V = 0x01,
      TCXO_CTRL_1_8V = 0x02,
      TCXO_CTRL_2_2V = 0x03,
      TCXO_CTRL_2_4V = 0x04,
      TCXO_CTRL_2_7V = 0x05,
      TCXO_CTRL_3_0V = 0x06,
      TCXO_CTRL_3_3V = 0x07,
    }RadioTcxoCtrlVoltage_t;

  /*!
   * \brief Represents the interruption masks available for the radio
   *
   * \remark Note that not all these interruptions are available for all packet types
   */
  typedef enum
    {
      IRQ_RADIO_NONE = 0x0000,
      IRQ_TX_DONE = 0x0001,
      IRQ_RX_DONE = 0x0002,
      IRQ_PREAMBLE_DETECTED = 0x0004,
      IRQ_SYNCWORD_VALID = 0x0008,
      IRQ_HEADER_VALID = 0x0010,
      IRQ_HEADER_ERROR = 0x0020,
      IRQ_CRC_ERROR = 0x0040,
      IRQ_CAD_DONE = 0x0080,
      IRQ_CAD_ACTIVITY_DETECTED = 0x0100,
      IRQ_RX_TX_TIMEOUT = 0x0200,
      IRQ_RADIO_ALL = 0xFFFF,
    }RadioIrqMasks_t;

  /*!
   * \brief Represents all possible opcode understood by the radio
   */
  typedef enum RadioCommands_e
    {
      RADIO_GET_STATUS = 0xC0,
      RADIO_WRITE_REGISTER = 0x0D,
      RADIO_READ_REGISTER = 0x1D,
      RADIO_WRITE_BUFFER = 0x0E,
      RADIO_READ_BUFFER = 0x1E,
      RADIO_SET_SLEEP = 0x84,
      RADIO_SET_STANDBY = 0x80,
      RADIO_SET_FS = 0xC1,
      RADIO_SET_TX = 0x83,
      RADIO_SET_RX = 0x82,
      RADIO_SET_RXDUTYCYCLE = 0x94,
      RADIO_SET_CAD = 0xC5,
      RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
      RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
      RADIO_SET_PACKETTYPE = 0x8A,
      RADIO_GET_PACKETTYPE = 0x11,
      RADIO_SET_RFFREQUENCY = 0x86,
      RADIO_SET_TXPARAMS = 0x8E,
      RADIO_SET_PACONFIG = 0x95,
      RADIO_SET_CADPARAMS = 0x88,
      RADIO_SET_BUFFERBASEADDRESS = 0x8F,
      RADIO_SET_MODULATIONPARAMS = 0x8B,
      RADIO_SET_PACKETPARAMS = 0x8C,
      RADIO_GET_RXBUFFERSTATUS = 0x13,
      RADIO_GET_PACKETSTATUS = 0x14,
      RADIO_GET_RSSIINST = 0x15,
      RADIO_GET_STATS = 0x10,
      RADIO_RESET_STATS = 0x00,
      RADIO_CFG_DIOIRQ = 0x08,
      RADIO_GET_IRQSTATUS = 0x12,
      RADIO_CLR_IRQSTATUS = 0x02,
      RADIO_CALIBRATE = 0x89,
      RADIO_CALIBRATEIMAGE = 0x98,
      RADIO_SET_REGULATORMODE = 0x96,
      RADIO_GET_ERROR = 0x17,
      RADIO_CLR_ERROR = 0x07,
      RADIO_SET_TCXOMODE = 0x97,
      RADIO_SET_TXFALLBACKMODE = 0x93,
      RADIO_SET_RFSWITCHMODE = 0x9D,
      RADIO_SET_STOPRXTIMERONPREAMBLE = 0x9F,
      RADIO_SET_LORASYMBTIMEOUT = 0xA0,
    }RadioCommands_t;

  /*!
   * \brief The type describing the modulation parameters for every packet types
   */
  typedef struct
    {
      RadioPacketTypes_t PacketType; //!< Packet to which the modulation parameters are referring to.
      struct
        {
          struct
            {
              uint32_t BitRate;
              uint32_t Fdev;
              RadioModShapings_t ModulationShaping;
              uint8_t Bandwidth;
            }Gfsk;
          struct
            {
              RadioLoRaSpreadingFactors_t SpreadingFactor; //!< Spreading Factor for the LoRa modulation
              RadioLoRaBandwidths_t Bandwidth;//!< Bandwidth for the LoRa modulation
              RadioLoRaCodingRates_t CodingRate;//!< Coding rate for the LoRa modulation
              uint8_t LowDatarateOptimize;//!< Indicates if the modem uses the low datarate optimization
            }LoRa;
        }Params;                  //!< Holds the modulation parameters structure
    }ModulationParams_t;

  /*!
   * \brief The type describing the packet parameters for every packet types
   */
  typedef struct
    {
      RadioPacketTypes_t PacketType; //!< Packet to which the packet parameters are referring to.
      struct
        {
          /*!
           * \brief Holds the GFSK packet parameters
           */
          struct
            {
              uint16_t PreambleLength; //!< The preamble Tx length for GFSK packet type in bit
              RadioPreambleDetection_t PreambleMinDetect;//!< The preamble Rx length minimal for GFSK packet type
              uint8_t SyncWordLength;//!< The synchronization word length for GFSK packet type
              RadioAddressComp_t AddrComp;//!< Activated SyncWord correlators
              RadioPacketLengthModes_t HeaderType;//!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
              uint8_t PayloadLength;//!< Size of the payload in the GFSK packet
              RadioCrcTypes_t CrcLength;//!< Size of the CRC block in the GFSK packet
              RadioDcFree_t DcFree;
            }Gfsk;
          /*!
           * \brief Holds the LoRa packet parameters
           */
          struct
            {
              uint16_t PreambleLength; //!< The preamble length is the number of LoRa symbols in the preamble
              RadioLoRaPacketLengthsMode_t HeaderType;//!< If the header is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
              uint8_t PayloadLength;//!< Size of the payload in the LoRa packet
              RadioLoRaCrcModes_t CrcMode;//!< Size of CRC block in LoRa packet
              RadioLoRaIQModes_t InvertIQ;//!< Allows to swap IQ for LoRa packet
            }LoRa;
        }Params;                      //!< Holds the packet parameters structure
    }PacketParams_t;

  /*!
   * \brief Represents the packet status for every packet type
   */
  typedef struct
    {
      RadioPacketTypes_t packetType; //!< Packet to which the packet status are referring to.
      struct
        {
          struct
            {
              uint8_t RxStatus;
              int8_t RssiAvg;                             //!< The averaged RSSI
              int8_t RssiSync;//!< The RSSI measured on last packet
              uint32_t FreqError;
            }Gfsk;
          struct
            {
              int8_t RssiPkt;                   //!< The RSSI of the last packet
              int8_t SnrPkt;//!< The SNR of the last packet
              int8_t SignalRssiPkt;
              uint32_t FreqError;
            }LoRa;
        }Params;
    }PacketStatus_t;

  /*!
   * \brief Represents the Rx internal counters values when GFSK or LoRa packet type is used
   */
  typedef struct
    {
      RadioPacketTypes_t packetType; //!< Packet to which the packet status are referring to.
      uint16_t PacketReceived;
      uint16_t CrcOk;
      uint16_t LengthError;
    }RxCounter_t;

  /*!
   * \brief Represents a calibration configuration
   */
  typedef union
    {
      struct
        {
          uint8_t RC64KEnable : 1;                    //!< Calibrate RC64K clock
          uint8_t RC13MEnable : 1;//!< Calibrate RC13M clock
          uint8_t PLLEnable : 1;//!< Calibrate PLL
          uint8_t ADCPulseEnable : 1;//!< Calibrate ADC Pulse
          uint8_t ADCBulkNEnable : 1;//!< Calibrate ADC bulkN
          uint8_t ADCBulkPEnable : 1;//!< Calibrate ADC bulkP
          uint8_t ImgEnable : 1;
          uint8_t : 1;
        }Fields;
      uint8_t Value;
    }CalibrationParams_t;

  /*!
   * \brief Represents a sleep mode configuration
   */
  typedef union
    {
      struct
        {
          uint8_t WakeUpRTC : 1; //!< Get out of sleep mode if wakeup signal received from RTC
          uint8_t Reset : 1;
          uint8_t WarmStart : 1;
          uint8_t Reserved : 5;
        }Fields;
      uint8_t Value;
    }SleepParams_t;

  /*!
   * \brief Represents the possible radio system error states
   */
  typedef union
    {
      struct
        {
          uint8_t Rc64kCalib : 1;    //!< RC 64kHz oscillator calibration failed
          uint8_t Rc13mCalib : 1;//!< RC 13MHz oscillator calibration failed
          uint8_t PllCalib : 1;//!< PLL calibration failed
          uint8_t AdcCalib : 1;//!< ADC calibration failed
          uint8_t ImgCalib : 1;//!< Image calibration failed
          uint8_t XoscStart : 1;//!< XOSC oscillator failed to start
          uint8_t PllLock : 1;//!< PLL lock failed
          uint8_t : 1;//!< Buck converter failed to start
          uint8_t PaRamp : 1;//!< PA ramp failed
          uint8_t : 7;//!< Reserved
        }Fields;
      uint16_t Value;
    }RadioError_t;

  /*!
   * SX126x definitions
   */

  /*!
   * \brief The radio callbacks structure
   * Holds function pointers to be called on radio interrupts
   */
  typedef struct
    {
      void ( *txDone )( void ); //!< Pointer to a function run on successful transmission
      void ( *rxDone )( uint8_t*, uint16_t, int8_t, int8_t );//!< Pointer to a function run on successful reception
      void ( *rxPreambleDetect )( void );//!< Pointer to a function run on successful Preamble detection
      void ( *rxSyncWordDone )( void );//!< Pointer to a function run on successful SyncWord reception
      void ( *rxHeaderDone )( bool isOk );//!< Pointer to a function run on successful Header reception
      void ( *txTimeout )( void );//!< Pointer to a function run on transmission timeout
      void ( *rxTimeout )( void );//!< Pointer to a function run on reception timeout
      void ( *rxError )( IrqErrorCode_t errCode );//!< Pointer to a function run on reception error
      void ( *cadDone )( bool cadFlag );//!< Pointer to a function run on channel activity detected
    }SX126xCallbacks_t;

  typedef uint32_t (*sx126x_dev_write)(uint8_t*, uint16_t);
  typedef uint32_t (*sx126x_dev_read) (uint8_t*, uint16_t);
  typedef uint32_t (*sx126x_dev_write_read) (uint8_t*, uint8_t*, uint16_t);
  typedef void (*sx126x_dev_delay) (uint32_t);
  typedef void (*sx126x_gpio_operation)(void);

  typedef struct
    {
      /*
       *  配置
       */
      sx126x_device_id device_id;
      sx126x_dev_write write_buffer;
      sx126x_dev_read read_buffer;
      sx126x_dev_write_read write_read_buffer;
      sx126x_dev_delay delay_ms;
      sx126x_gpio_operation wait_on_busy;
      sx126x_gpio_operation cs_low;
      sx126x_gpio_operation cs_high;
      sx126x_gpio_operation rst_low;
      sx126x_gpio_operation rst_high;
      sx126x_gpio_operation tcxo_enable;
      sx126x_gpio_operation tcxo_disable;
      sx126x_gpio_operation ant_sw_tx;
      sx126x_gpio_operation ant_sw_rx;
      PacketParams_t packet_params;
      ModulationParams_t modulation_params;
      SX126xCallbacks_t callback;
      RadioRegulatorMode_t regulator_mode;
      bool rx_continuous;
      bool dio2_as_rf_switch_ctrl_flag;
      int8_t tx_power;
      uint16_t symble_timeout;
      /*
       *  状态
       */
      RadioPacketTypes_t packet_type;
      PacketStatus_t packet_status;
      RadioOperatingModes_t op_mode;
      RadioLoRaPacketLengthsMode_t lora_header_type;
      uint32_t frequency_error;
      bool image_calibrated;
      uint32_t calibrated_freq;
      uint8_t radio_rx_payload[256];
      uint16_t radio_rx_payload_len;
      uint32_t dio1_tick;
      uint32_t dio2_tick;
      uint32_t dio3_tick;
      uint32_t tx_done_tick;
      uint32_t rx_done_tick;
    }sx126x_dev;

  /*!
   * ============================================================================
   * Public functions prototypes
   * ============================================================================
   */

  /*!
   * \brief Initializes the radio driver
   */
  bool sx126x_init(sx126x_dev *dev);

  void sx126x_tx(sx126x_dev *dev, uint8_t* data, uint16_t len, uint32_t timeout_ms);

  void sx126x_rx(sx126x_dev *dev, uint32_t timeout_ms);

  /*!
   * \brief Wakeup the radio if it is in Sleep mode and check that Busy is low
   */
  void sx126x_check_device_ready( sx126x_dev *dev );

  /*!
   * \brief Saves the payload to be send in the radio buffer
   *
   * \param [in]  payload       A pointer to the payload
   * \param [in]  size          The size of the payload
   */
  void sx126x_set_payload( sx126x_dev *dev, uint8_t *payload, uint8_t size );

  /*!
   * \brief Reads the payload received. If the received payload is longer
   * than maxSize, then the method returns 1 and do not set size and payload.
   *
   * \param [out] payload       A pointer to a buffer into which the payload will be copied
   * \param [out] size          A pointer to the size of the payload received
   * \param [in]  maxSize       The maximal size allowed to copy into the buffer
   */
  uint8_t sx126x_get_payload( sx126x_dev *dev, uint8_t *payload, uint8_t *size, uint8_t maxSize );

  /*!
   * \brief Sends a payload
   *
   * \param [in]  payload       A pointer to the payload to send
   * \param [in]  size          The size of the payload to send
   * \param [in]  timeout       The timeout for Tx operation
   */
  void sx126x_send_payload( sx126x_dev *dev, uint8_t *payload, uint8_t size, uint32_t timeout );

  /*!
   * \brief Sets the Sync Word given by index used in GFSK
   *
   * \param [in]  syncWord      SyncWord bytes ( 8 bytes )
   *
   * \retval      status        [0: OK, 1: NOK]
   */
  uint8_t sx126x_set_sync_word( sx126x_dev *dev, uint8_t *syncWord );

  /*!
   * \brief Sets the Initial value for the LFSR used for the CRC calculation
   *
   * \param [in]  seed          Initial LFSR value ( 2 bytes )
   *
   */
  void sx126x_set_crc_seed( sx126x_dev *dev, uint16_t seed );

  /*!
   * \brief Sets the seed used for the CRC calculation
   *
   * \param [in]  seed          The seed value
   *
   */
  void sx126x_set_crc_polynomial( sx126x_dev *dev, uint16_t polynomial );

  /*!
   * \brief Sets the Initial value of the LFSR used for the whitening in GFSK protocols
   *
   * \param [in]  seed          Initial LFSR value
   */
  void sx126x_set_whitening_seed( sx126x_dev *dev, uint16_t seed );

  /*!
   * \brief Gets a 32-bit random value generated by the radio
   *
   * \remark A valid packet type must have been configured with \ref SX126xSetPacketType
   *         before using this command.
   *
   * \remark The radio must be in reception mode before executing this function
   *         This code can potentially result in interrupt generation. It is the responsibility of
   *         the calling code to disable radio interrupts before calling this function,
   *         and re-enable them afterwards if necessary, or be certain that any interrupts
   *         generated during this process will not cause undesired side-effects in the software.
   *
   *         Please note that the random numbers produced by the generator do not have a uniform or Gaussian distribution. If
   *         uniformity is needed, perform appropriate software post-processing.
   *
   * \retval randomValue    32 bits random value
   */
  uint32_t sx126x_get_random( sx126x_dev *dev );

  void
  sx126x_set_sleep_warm_start (sx126x_dev *dev);

  void
  sx126x_set_sleep_cold_start (sx126x_dev *dev);

  /*!
   * \brief Sets the radio in sleep mode
   *
   * \param [in]  sleepConfig   The sleep configuration describing data
   *                            retention and RTC wake-up
   */
  void sx126x_set_sleep( sx126x_dev *dev, SleepParams_t sleepConfig );

  /*!
   * \brief Sets the radio in configuration mode
   *
   * \param [in]  mode          The standby mode to put the radio into
   */
  void sx126x_set_standby( sx126x_dev *dev, RadioStandbyModes_t mode );

  /*!
   * \brief Sets the radio in FS mode
   */
  void sx126x_set_fs( sx126x_dev *dev );

  /*!
   * \brief Sets the radio in transmission mode
   *
   * \param [in]  timeout       Structure describing the transmission timeout value
   */
  void sx126x_set_tx( sx126x_dev *dev, uint32_t timeout );

  /*!
   * \brief Sets the radio in reception mode
   *
   * \param [in]  timeout       Structure describing the reception timeout value
   */
  void sx126x_set_rx( sx126x_dev *dev, uint32_t timeout );

  /*!
   * \brief Sets the radio in reception mode with Boosted LNA gain
   *
   * \param [in]  timeout       Structure describing the reception timeout value
   */
  void sx126x_set_rx_boosted( sx126x_dev *dev, uint32_t timeout );

  /*!
   * \brief Sets the Rx duty cycle management parameters
   *
   * \param [in]  rxTime        Structure describing reception timeout value
   * \param [in]  sleepTime     Structure describing sleep timeout value
   */
  void sx126x_set_rx_duty_cycle( sx126x_dev *dev, uint32_t rxTime, uint32_t sleepTime );

  /*!
   * \brief Sets the radio in CAD mode
   */
  void sx126x_set_cad( sx126x_dev *dev );

  /*!
   * \brief Sets the radio in continuous wave transmission mode
   */
  void sx126x_set_tx_continuous_wave( sx126x_dev *dev );

  /*!
   * \brief Sets the radio in continuous preamble transmission mode
   */
  void sx126x_set_tx_infinite_preamble( sx126x_dev *dev );

  /*!
   * \brief Decide which interrupt will stop the internal radio rx timer.
   *
   * \param [in]  enable          [0: Timer stop after header/syncword detection
   *                               1: Timer stop after preamble detection]
   */
  void sx126x_set_stop_rx_timer_on_preamble_detect( sx126x_dev *dev, bool enable );

  /*!
   * \brief Set the number of symbol the radio will wait to validate a reception
   *
   * \param [in]  SymbNum          number of LoRa symbols
   */
  void sx126x_set_lora_symb_num_timeout( sx126x_dev *dev, uint8_t SymbNum );

  /*!
   * \brief Sets the power regulators operating mode
   *
   * \param [in]  mode          [0: LDO, 1:DC_DC]
   */
  void sx126x_set_regulator_mode( sx126x_dev *dev, RadioRegulatorMode_t mode );

  /*!
   * \brief Calibrates the given radio block
   *
   * \param [in]  calibParam    The description of blocks to be calibrated
   */
  void sx126x_calibrate( sx126x_dev *dev, CalibrationParams_t calibParam );

  /*!
   * \brief Calibrates the Image rejection depending of the frequency
   *
   * \param [in]  freq    The operating frequency
   */
  void sx126x_calibrate_image( sx126x_dev *dev, uint32_t freq );

  /*!
   * \brief Activate the extention of the timeout when long preamble is used
   *
   * \param [in]  enable      The radio will extend the timeout to cope with long preamble
   */
  void sx126x_set_long_preamble( sx126x_dev *dev, uint8_t enable );

  /*!
   * \brief Sets the transmission parameters
   *
   * \param [in]  paDutyCycle     Duty Cycle for the PA
   * \param [in]  hpMax          0 for sx1261, 7 for sx1262
   * \param [in]  deviceSel       1 for sx1261, 0 for sx1262
   * \param [in]  paLut           0 for 14dBm LUT, 1 for 22dBm LUT
   */
  void sx126x_set_pa_config( sx126x_dev *dev, uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut );

  /*!
   * \brief Defines into which mode the chip goes after a TX / RX done
   *
   * \param [in]  fallbackMode    The mode in which the radio goes
   */
  void sx126x_set_rx_tx_fallback_mode( sx126x_dev *dev, uint8_t fallbackMode );

  /*!
   * \brief Write data to the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the radio
   * \param [in]  buffer        The data to be written in radio's memory
   * \param [in]  size          The number of bytes to write in radio's memory
   */
  void sx126x_write_registers( sx126x_dev *dev, uint16_t address, uint8_t *buffer, uint16_t size );

  /*!
   * \brief Read data from the radio memory
   *
   * \param [in]  address       The address of the first byte to read from the radio
   * \param [out] buffer        The buffer that holds data read from radio
   * \param [in]  size          The number of bytes to read from radio's memory
   */
  void sx126x_read_registers( sx126x_dev *dev, uint16_t address, uint8_t *buffer, uint16_t size );

  /*!
   * \brief Write data to the buffer holding the payload in the radio
   *
   * \param [in]  offset        The offset to start writing the payload
   * \param [in]  buffer        The data to be written (the payload)
   * \param [in]  size          The number of byte to be written
   */
  void sx126x_write_buffer( sx126x_dev *dev, uint8_t offset, uint8_t *buffer, uint8_t size );

  /*!
   * \brief Read data from the buffer holding the payload in the radio
   *
   * \param [in]  offset        The offset to start reading the payload
   * \param [out] buffer        A pointer to a buffer holding the data from the radio
   * \param [in]  size          The number of byte to be read
   */
  void sx126x_read_buffer( sx126x_dev *dev, uint8_t offset, uint8_t *buffer, uint8_t size );

  /*!
   * \brief   Sets the IRQ mask and DIO masks
   *
   * \param [in]  irqMask       General IRQ mask
   * \param [in]  dio1Mask      DIO1 mask
   * \param [in]  dio2Mask      DIO2 mask
   * \param [in]  dio3Mask      DIO3 mask
   */
  void sx126x_set_dio_irq_params( sx126x_dev *dev, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );

  /*!
   * \brief Returns the current IRQ status
   *
   * \retval      irqStatus     IRQ status
   */
  uint16_t sx126x_get_irq_status( sx126x_dev *dev );

  /*!
   * \brief Indicates if DIO2 is used to control an RF Switch
   *
   * \param [in] enable     true of false
   */
  void sx126x_set_dio2_as_rf_switch_ctrl( sx126x_dev *dev, uint8_t enable );

  /*!
   * \brief Indicates if the Radio main clock is supplied from a tcxo
   *
   * \param [in] tcxoVoltage     voltage used to control the TCXO
   * \param [in] timeout         time given to the TCXO to go to 32MHz
   */
  void sx126x_set_dio3_as_tcxo_ctrl( sx126x_dev *dev, RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout );

  /*!
   * \brief Sets the RF frequency
   *
   * \param [in]  frequency     RF frequency [Hz]
   */
  void sx126x_set_rf_frequency( sx126x_dev *dev, uint32_t frequency );

  /*!
   * \brief Sets the radio for the given protocol
   *
   * \param [in]  packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA]
   *
   * \remark This method has to be called before SetRfFrequency,
   *         SetModulationParams and SetPacketParams
   */
  void sx126x_set_packet_type( sx126x_dev *dev, RadioPacketTypes_t packetType );

  RadioPacketTypes_t sx126x_get_packet_type (sx126x_dev *dev);

  /*!
   * \brief Sets the transmission parameters
   *
   * \param [in]  power         RF output power [-18..13] dBm
   * \param [in]  rampTime      Transmission ramp up time
   */
  void sx126x_set_tx_params( sx126x_dev *dev, int8_t power, RadioRampTimes_t rampTime );

  /*!
   * \brief Set the modulation parameters
   *
   * \param [in]  modParams     A structure describing the modulation parameters
   */
  void sx126x_set_modulation_params( sx126x_dev *dev, ModulationParams_t *modParams );

  /*!
   * \brief Sets the packet parameters
   *
   * \param [in]  packetParams  A structure describing the packet parameters
   */
  void sx126x_set_packet_params( sx126x_dev *dev, PacketParams_t *packetParams );

  /*!
   * \brief Sets the Channel Activity Detection (CAD) parameters
   *
   * \param [in]  cadSymbolNum   The number of symbol to use for CAD operations
   *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
   *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
   *                              LORA_CAD_16_SYMBOL]
   * \param [in]  cadDetPeak     Limit for detection of SNR peak used in the CAD
   * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
   * \param [in]  cadExitMode    Operation to be done at the end of CAD action
   *                             [LORA_CAD_ONLY, LORA_CAD_RX, LORA_CAD_LBT]
   * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity
   */
  void sx126x_set_cad_params( sx126x_dev *dev, RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout );

  /*!
   * \brief Sets the data buffer base address for transmission and reception
   *
   * \param [in]  txBaseAddress Transmission base address
   * \param [in]  rxBaseAddress Reception base address
   */
  void sx126x_set_buffer_base_address( sx126x_dev *dev, uint8_t txBaseAddress, uint8_t rxBaseAddress );

  /*!
   * \brief Gets the current radio status
   *
   * \retval      status        Radio status
   */
  RadioStatus_t sx126x_get_status( sx126x_dev *dev );

  /*!
   * \brief Returns the instantaneous RSSI value for the last packet received
   *
   * \retval      rssiInst      Instantaneous RSSI
   */
  int8_t sx126x_get_rssi_inst( sx126x_dev *dev );

  /*!
   * \brief Gets the last received packet buffer status
   *
   * \param [out] payloadLength Last received packet payload length
   * \param [out] rxStartBuffer Last received packet buffer address pointer
   */
  void sx126x_get_rx_buffer_status( sx126x_dev *dev, uint8_t *payloadLength, uint8_t *rxStartBuffer );

  /*!
   * \brief Gets the last received packet payload length
   *
   * \param [out] pktStatus     A structure of packet status
   */
  void sx126x_get_packet_status( sx126x_dev *dev, PacketStatus_t *pktStatus );

  /*!
   * \brief Returns the possible system errors
   *
   * \retval sysErrors Value representing the possible sys failures
   */
  RadioError_t sx126x_get_device_errors( sx126x_dev *dev );

  /*!
   * \brief Clear all the errors in the device
   */
  void sx126x_clear_device_errors( sx126x_dev *dev );

  /*!
   * \brief Clears the IRQs
   *
   * \param [in]  irq           IRQ(s) to be cleared
   */
  void sx126x_clear_irq_status( sx126x_dev *dev, uint16_t irq );

  void sx126x_dio1_process(sx126x_dev *dev);

  void sx126x_dio2_process(sx126x_dev *dev);

  void sx126x_dio3_process(sx126x_dev *dev);

  /*!
   * \brief HW Reset of the radio
   */
  void sx126x_reset( sx126x_dev *dev );

  /*!
   * \brief Wakes up the radio
   */
  void sx126x_wakeup( sx126x_dev *dev );

  /*!
   * \brief Send a command that write data to the radio
   *
   * \param [in]  opcode        Opcode of the command
   * \param [in]  buffer        Buffer to be send to the radio
   * \param [in]  size          Size of the buffer to send
   */
  void sx126x_write_command( sx126x_dev *dev, RadioCommands_t opcode, uint8_t *buffer, uint16_t size );

  /*!
   * \brief Send a command that read data from the radio
   *
   * \param [in]  opcode        Opcode of the command
   * \param [out] buffer        Buffer holding data from the radio
   * \param [in]  size          Size of the buffer
   *
   * \retval status Return command radio status
   */
  uint8_t sx126x_read_command( sx126x_dev *dev, RadioCommands_t opcode, uint8_t *buffer, uint16_t size );

  /*!
   * \brief Write a single byte of data to the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the radio
   * \param [in]  value         The data to be written in radio's memory
   */
  void sx126x_write_register( sx126x_dev *dev, uint16_t address, uint8_t value );

  /*!
   * \brief Read a single byte of data from the radio memory
   *
   * \param [in]  address       The address of the first byte to write in the radio
   *
   * \retval      value         The value of the byte at the given address in radio's memory
   */
  uint8_t sx126x_read_register( sx126x_dev *dev, uint16_t address );

  /*!
   * \brief Sets the radio output power.
   *
   * \param [IN] power Sets the RF output power
   */
  void sx126x_set_rf_tx_power( sx126x_dev *dev, int8_t power );

  /*!
   * \brief Gets the device ID
   *
   * \retval id Connected device ID
   */
  uint8_t sx126x_get_device_id( sx126x_dev *dev );

  /*!
   * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
   *
   * \retval time Board TCXO wakeup time in ms.
   */
  uint32_t sx126x_get_board_tcxo_wakeup_time( sx126x_dev *dev );

  void sx126x_tx (sx126x_dev *dev, uint8_t *data, uint16_t len,
                  uint32_t timeout_ms);

  void sx126x_tx_set_payload (sx126x_dev *dev, uint8_t *data, uint16_t len);

  void sx126x_tx_commit(sx126x_dev *dev, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // __SX126x_H__