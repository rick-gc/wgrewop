/*
 * @Author       : yfwang
 * @Date         : 2021-09-26 17:12:26
 * @LastEditTime : 2021-10-27 20:18:46
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\common.h
 */

#ifndef _COMMON_H_
#define _COMMON_H_

typedef struct
{
    uint8_t type;
    uint8_t *buf;
    uint16_t len;
    uint32_t tick;
} msg_t;

typedef union
{
    uint8_t a[8];
    uint64_t b;
    uint32_t c[2];
} un_8u8_u64;

typedef union
{
    uint8_t a[4];
    uint32_t b;
} un_4u8_u32;

typedef union
{
    uint8_t a[2];
    uint16_t b;
} un_2u8_u16;

uint16_t RTU_CRC(uint8_t *puchMsg, uint16_t usDataLen);
uint32_t usticks(uint32_t us);
uint32_t msticks(uint32_t ms);
uint32_t ssticks(uint32_t s);
bool find_same(uint64_t imp, un_8u8_u64 *buf, uint8_t len);
bool find_same_num(uint64_t imp, un_8u8_u64 *buf, uint8_t len, uint8_t *num);
void freepoint(void *buf);
void queue_send(xQueueHandle queue, uint8_t *p_data, uint16_t len);
void queue_send_ISR(xQueueHandle queue, uint8_t *p_data, uint16_t len);
void data_copy(uint8_t *buf, uint8_t *data, uint16_t len);

#endif /* _COMMON_H_ */