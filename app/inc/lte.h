/*
 * @Author       : yfwang
 * @Date         : 2021-09-28 15:56:05
 * @LastEditTime : 2021-11-05 15:23:59
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\lte.h
 */

#ifndef _lTE_H_
#define _lTE_H_

typedef enum
{
    EC600_ON,
    EC600_OFF
} ec600_power;

typedef struct
{
    un_8u8_u64 timestamp;
    uint32_t system_time;
} s_timestamp;

typedef struct
{
    un_8u8_u64 dev_white_list[50];
    un_8u8_u64 sen_white_list[100];
    un_8u8_u64 dev_black_list[50];
    un_8u8_u64 sen_black_list[100];
    un_8u8_u64 temp_list[100];
    uint8_t dev_white_list_num, sen_white_list_num;
    uint8_t dev_black_list_num, sen_black_list_num;
    uint8_t temp_list_num;
} s_bw_list; // 8个字节 [6]类型 [5-0]id

typedef struct
{
    un_8u8_u64 hj_white_list[50];   //传感器白名单中 父节点名单
    un_8u8_u64 white_list[2][200];  //传感器白名单 [0]父节点 [1]主节点
    un_8u8_u64 hj_black_list[50];   //传感器黑名单中 父节点名单
    un_8u8_u64 black_list[2][200];  //传感器黑名单 [0]父节点 [1]主节点
    un_8u8_u64 temp_list[2][200];
    uint8_t hj_white_list_num, hj_black_list_num;
    uint8_t white_list_num, black_list_num;
    uint8_t temp_list_num;
} s_hj_bw_list;

typedef struct
{
    un_2u8_u16 frequency[5];
    un_2u8_u16 power[5];
} s_path_config;

typedef struct
{
    un_2u8_u16 power;
    un_2u8_u16 msg_cycle;
    un_2u8_u16 control_cycle;
    un_2u8_u16 drx_cycle;
    un_2u8_u16 control_frequency;
    un_2u8_u16 msg_frequency;
} s_sensor_config;

typedef struct
{
    un_8u8_u64 list[2][200]; //[0]父节点id [1]设备id
    uint8_t num;
} s_topology;

typedef struct
{
    un_8u8_u64 list[100]; //[0]父节点id [1]设备id
    uint8_t num;
    s_path_config path_config[100];
} s_hj_config;

typedef struct
{
    un_8u8_u64 list[100]; //[0]父节点id [1]设备id
    uint8_t num;
    s_sensor_config sensor_config[100];
} s_sensor_config_list;

extern un_8u8_u64 mac_addr;
extern s_timestamp lte_time, sensor_time;
extern s_bw_list bw_list;
extern s_hj_bw_list hj_bw_list;
extern s_path_config path_config;
extern s_topology topology;
extern s_hj_config hj_config;
extern s_sensor_config_list sensor_config_list;

void mac_addr_init();
void ec600s_io_init();
void ec600s_power_button(bool status);
void get_mid(uint8_t *buf);
void lte_tx(uint8_t *buf, uint8_t len);
void lte_reply(uint16_t data, uint32_t id);
void lte_reply_state(uint16_t data, uint32_t id, uint8_t state);
void lte_close();
void lte_get_time_tx();
void lte_get_time_rx(uint8_t *buf);
void lte_version_tx();
void lte_voltage_tx();
void pack_sensor_msg();
uint8_t find_bw_list(uint64_t list);
void path_config_rx(uint8_t *buf);
void path_config_tx();
void bw_list_hj_rx(uint8_t *buf);
void bw_list_hj_tx();
void bw_list_sensor_rx(uint8_t *buf);
void bw_list_sensor_tx();
void hj_config_rx(uint8_t *buf);
void hj_config_tx();
void hj_bw_list_rx(uint8_t *buf);
void sensor_config_rx(uint8_t *buf);
void sensor_config_reply(uint32_t id, uint8_t *buf);

#endif /* _lTE_H_ */