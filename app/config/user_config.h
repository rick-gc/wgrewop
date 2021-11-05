/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-10-19 17:20:48
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\config\user_config.h
 */

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define LTE_RX_BUF_LEN                 (1024 * 3)   //串口接收数组长度
#define LTE_RESTART_NUM                5            //ec600重启次数
#define LTE_RESTART_TIM                20           //ec600重启间隔 单位:s
#define LTE_START_TIM                  30           //ec600开机时间 单位:s
#define LTE_BUTTON_WAIT                7            //ec600开关等待时间 单位:s
#define LTE_BUTTON_PUSH                1000         //ec600开关长按时间 单位:ms
#define LTE_WAKE_TIM                   5            //ec600唤醒时间 单位:min
#define SENNUM_CHECK_NUM               5            //查询传感器数据已发送数量的次数 单位:s
#define SENNUM_CHECK_TIM               5            //查询传感器数据已发送数量的间隔 单位:s
#define LTE_CHECK_NUM                  5            //发送命令的次数 单位:s
#define LTE_CHECK_TIM                  5            //发送命令的间隔 单位:s
#define SENSOR_CHECK_NUM               5            //上报传感器数据的次数 单位:s
#define SENSOR_CHECK_TIM               5            //上报传感器数据的间隔 单位:s
#define LTE_BUSY_NUM                   5            //查询busy状态位次数
#define LTE_BUSY_TIM                   5            //查询busy状态位间隔 单位:s
#define SAMPLES_IN_BUFFER              3            //adc采集次数
#define SENSOR_LISE_BUF                10           //黑白名单缓存数组数量
#define SENSOR_LISE                    20           //黑白名单数量
#define MPOWER_CONFIG_LIST             20           //最大微功率传感器配置存储数量 最大：256

/*ble_config*/
#define DEVICE_NAME                         "Nordic_BLE"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//微功率传感器默认配置
#define MPOWER_MESSAGE_CYCLE                300000  //业务周期长度      单位:ms
#define MPOWER_CONTROL_CYCLE                12      //控制周期长度      单位:业务周期
#define MPOWER_REQ_WAIT_CYCLE               110     //REQ等待回复周期   单位:ms
#define MPOWER_BURST_WAIT_CYCLE             30      //BURST等待回复周期 单位:ms
#define MPOWER_RANDOM_PERT                  5       //随机扰动时长      单位:ms
#define MPOWER_DELAY                        0       //延迟 

//sx1280频点配置
#define SX1280_Freq_con 2400500000
#define SX1280_Freq_mes 2424500000

//470M
#define MASTER_ID_H 0XFF
#define MASTER_ID_L 0X01

//设备信息
#define EID     0x010203040506  //节点ID
#define HVER    0x010000        //硬件版本
#define SVER    0x010000        //软件版本

#endif /* _USER_CONFIG_H_ */
