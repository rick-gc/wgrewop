/*
 * @Author       : yfwang
 * @Date         : 2021-09-08 13:23:07
 * @LastEditTime : 2021-10-27 16:36:46
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\mpower.h
 */

#ifndef _MPOWER_H_
#define _MPOWER_H_

typedef enum
{
    MESSAGE,
    REQ,
    RSP,
    RSP_END,
    BURST,
    ACK,
    RFU
} mpower_TYPE;

typedef enum
{
    RSP_END_ACK = 1,
    BURST_ACK,
    RSP_ACK = 4
} mpower_ACK;

typedef union
{
    struct
    {
        uint8_t key : 3;
        uint8_t ind : 1;
        uint8_t type : 4;
    };
    char value;
} head_byte;

typedef struct
{
    un_4u8_u32 message_cycle;
    un_2u8_u16 control_cycle;
    un_4u8_u32 delay;
    uint8_t random_pert;
    uint8_t req_wait;
    uint8_t burst_wait;
} mpower_config;

#define MPOWER_DEFAULT_CONFIG                    \
    {                                            \
        .message_cycle.b = MPOWER_MESSAGE_CYCLE, \
        .control_cycle.b = MPOWER_CONTROL_CYCLE, \
        .delay.b = MPOWER_DELAY,                 \
        .random_pert = MPOWER_RANDOM_PERT,       \
        .req_wait = MPOWER_REQ_WAIT_CYCLE,       \
        .burst_wait = MPOWER_BURST_WAIT_CYCLE    \
    }

uint8_t checksum(char *buf, uint8_t len);
void req_process(uint8_t buf, un_8u8_u64 id);
void burst_process(uint8_t *buf, uint8_t len, uint8_t ind);
void message_process(uint8_t *buf, uint8_t len);
void ack_process(uint8_t buf, un_8u8_u64 id);

#endif /* _MPOWER_H_ */