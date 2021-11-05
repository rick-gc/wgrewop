/*
 * @Author       : yfwang
 * @Date         : 2021-09-23 18:38:54
 * @LastEditTime : 2021-11-05 20:38:19
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\lpower.h
 */

#ifndef _lPOWER_H_
#define _lPOWER_H_

typedef struct
{
    union
    {
        struct
        {
            uint8_t encrypt : 1;
            uint8_t mic : 1;
            uint8_t reply : 1;
            uint8_t layer : 1;
            uint8_t type : 4;
        };
        uint8_t head;
    };
    uint8_t len;
} lpower_message_s;

typedef struct
{
    uint8_t buf[100];
    uint8_t len; //这是负载长度 整包长度等于负载长度+2帧头+2CRC
    union
    {
        struct
        {
            uint8_t node_num : 5;
            uint8_t type : 3;
        };
        uint8_t type_s;
    };
    uint8_t access_flag;
    un_8u8_u64 access_id;
    uint8_t node_type;
    uint8_t slave_id;
    uint8_t slave_num;
} lpower_dcch_s;

#define DCCH_DEFAULT_DATA      \
    {                          \
        .buf[0] = 0X12,        \
        .buf[1] = 3,           \
        .buf[2] = MASTER_ID_H, \
        .buf[3] = MASTER_ID_L, \
        .len = 3,              \
        .slave_id = 0xA0       \
    }

typedef struct
{
    union
    {
        struct
        {
            uint8_t reserve : 1;
            uint8_t apply : 1;
            uint8_t pic : 1;
            uint8_t type_len : 5;
        };
        uint8_t type_s;
    };
    union
    {
        struct
        {
            uint8_t sdu_flag : 2;
            uint8_t sdu_num : 6;
        };
        uint8_t SDU;
    };
    union
    {
        struct
        {
            uint8_t pdu_pri : 1;
            uint8_t pdu_num : 7;
        };
        uint8_t PDU;
    };
    uint8_t size;
    union
    {
        struct
        {
            uint8_t msg_num_flag : 1;
            uint8_t port_flag : 1;
            uint8_t sensor_eid_flag : 2;
            uint8_t device_eid_flag : 1;
            uint8_t msg_flag : 1;
            uint8_t uod_flag : 1; // up or down 上下行指示
            uint8_t bdc_flag : 1; // broadcast 广播指示
        };
        uint8_t msg_type;
    };
    un_8u8_u64 sensor_id;
    un_8u8_u64 device_id;
} lpower_usch_s;

typedef struct
{
    uint8_t buf[256];
    uint8_t buf_len;
    uint8_t data_len; //通信数据长度
    union
    {
        struct
        {
            uint8_t reserve : 2;
            uint8_t data_flag : 1; //通信数据指示
            uint8_t ins_len : 5;   //指令长度 0表示没有指令
        };
        uint8_t data;
    };
    union
    {
        struct
        {
            uint8_t sdu_flag : 2;
            uint8_t sdu_num : 6;
        };
        uint8_t SDU;
    };
    union
    {
        struct
        {
            uint8_t pdu_pri : 1;
            uint8_t pdu_num : 7;
        };
        uint8_t PDU;
    };
    uint8_t size;
    union
    {
        struct
        {
            uint8_t msg_num_flag : 1;
            uint8_t port_flag : 1;
            uint8_t sensor_eid : 2;
            uint8_t dev_eid : 1;
            uint8_t msg_flag : 1;
            uint8_t uod : 1;
            uint8_t boradcast : 1;
        };
        uint8_t msg_type;
    };
} lpower_dsch_s;
#define DSCH_DEFAULT_DATA     \
    {                         \
        .buf = { 0x3a,        \
                 0,           \
                 MASTER_ID_H, \
                 MASTER_ID_L, \
                 0xff,        \
                 0xff,        \
                 0,           \
                 0x04 }       \
    }

typedef enum
{
    BCH,
    DCCH,
    DSCH = 3,
    URCH,
    USCH
} lpower_TYPE;

extern lpower_message_s lpower_message;
extern lpower_dcch_s lpower_dcch;
extern lpower_usch_s lpower_usch;
extern lpower_dsch_s lpower_dsch;

extern uint8_t bch[55];

void urch_process(uint8_t *buf, uint8_t len);
void dcch_process();
void usch_mac_process(uint8_t *buf, uint8_t len);
void usch_msg_process(uint8_t *buf, uint8_t len);

#endif /* _lPOWER_H_ */