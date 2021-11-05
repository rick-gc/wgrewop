/*
 * @Author       : yfwang
 * @Date         : 2021-09-08 13:13:07
 * @LastEditTime : 2021-11-02 16:17:28
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\mpower.c
 */

#include "main.h"

uint32_t sent_id;

uint8_t checksum(char *buf, uint8_t len)
{
    uint8_t n, sum;
    for (n = 0; n < len; n++)
    {
        sum += *(buf + n);
    }
    return sum;
}

void req_process(uint8_t flag, un_8u8_u64 id)
{
    head_byte hbyte;
    msg_t msg;
    mpower_config config = MPOWER_DEFAULT_CONFIG;
    if (flag < 2)
    {
        msg.len = 25;
        msg.buf = pvPortMalloc(msg.len);
        hbyte.type = RSP_END;
        hbyte.ind = 0;
        hbyte.key = 0;
        *msg.buf = hbyte.value;
        *(msg.buf + 1) = 16;
        *(msg.buf + 2) = id.a[5];
        *(msg.buf + 3) = id.a[4];
        *(msg.buf + 4) = id.a[3];
        *(msg.buf + 5) = id.a[2];
        *(msg.buf + 6) = id.a[1];
        *(msg.buf + 7) = id.a[0];
        *(msg.buf + 8) = 6;
        *(msg.buf + 9) = config.req_wait;
        *(msg.buf + 10) = 7;
        *(msg.buf + 11) = config.burst_wait;
        *(msg.buf + 12) = 0xff;
        *(msg.buf + 13) = config.message_cycle.a[3];
        *(msg.buf + 14) = config.message_cycle.a[2];
        *(msg.buf + 15) = config.message_cycle.a[1];
        *(msg.buf + 16) = config.message_cycle.a[0];
        *(msg.buf + 17) = config.control_cycle.a[1];
        *(msg.buf + 18) = config.control_cycle.a[0];
        *(msg.buf + 19) = config.delay.a[3];
        *(msg.buf + 20) = config.delay.a[2];
        *(msg.buf + 21) = config.delay.a[1];
        *(msg.buf + 22) = config.delay.a[0];
        *(msg.buf + 23) = config.random_pert;
        *(msg.buf + 24) = checksum(msg.buf, msg.len - 1);
        sent_id = id.b;
        xQueueSend(global.queue_mpower_send_msg, &msg, portMAX_DELAY);
        NRF_LOG_INFO("reply rsp_end success");
    }
}

void burst_process(uint8_t *buf, uint8_t len, uint8_t ind)
{
    msg_t msg;
    un_8u8_u64 id;
    head_byte hbyte;
    if (ind == 0)
    {
        id.a[5] = *(buf);
        id.a[4] = *(buf + 1);
        id.a[3] = *(buf + 2);
        id.a[2] = *(buf + 3);
        id.a[1] = *(buf + 4);
        id.a[0] = *(buf + 5);
        msg.len = 10;
        msg.buf = pvPortMalloc(msg.len);
        hbyte.type = ACK;
        hbyte.ind = 0;
        hbyte.key = 0;
        *msg.buf = hbyte.value;
        *(msg.buf + 1) = 1;
        *(msg.buf + 2) = id.a[5];
        *(msg.buf + 3) = id.a[4];
        *(msg.buf + 4) = id.a[3];
        *(msg.buf + 5) = id.a[2];
        *(msg.buf + 6) = id.a[1];
        *(msg.buf + 7) = id.a[0];
        *(msg.buf + 8) = BURST_ACK;
        *(msg.buf + 9) = checksum(msg.buf, msg.len - 1);
        xQueueSend(global.queue_mpower_send_msg, &msg, portMAX_DELAY);
        NRF_LOG_INFO("reply ACK success");
    }
    else if (ind == 1)
    {
        id.a[5] = *(buf);
        id.a[4] = *(buf + 1);
        id.a[3] = *(buf + 2);
        id.a[2] = *(buf + 3);
        id.a[1] = *(buf + 4);
        id.a[0] = *(buf + 5);
        // msg.len = len;
        // msg.buf = pvPortMalloc(msg.len);
        // memcpy(msg.buf, buf, len);
        // xQueueSend(global.queue_sensor_msg, &msg, portMAX_DELAY);
        msg.len = 10;
        msg.buf = pvPortMalloc(msg.len);
        hbyte.type = ACK;
        hbyte.ind = 0;
        hbyte.key = 0;
        *msg.buf = hbyte.value;
        *(msg.buf + 1) = 1;
        *(msg.buf + 2) = id.a[5];
        *(msg.buf + 3) = id.a[4];
        *(msg.buf + 4) = id.a[3];
        *(msg.buf + 5) = id.a[2];
        *(msg.buf + 6) = id.a[1];
        *(msg.buf + 7) = id.a[0];
        *(msg.buf + 8) = BURST_ACK;
        *(msg.buf + 9) = checksum(msg.buf, msg.len - 1);
        xQueueSend(global.queue_mpower_send_msg, &msg, portMAX_DELAY);
        NRF_LOG_INFO("reply ACK success");
    }
}

void message_process(uint8_t *buf, uint8_t len)
{
    msg_t msg;
    un_8u8_u64 id;

    id.a[7] = 0;
    id.a[6] = 0;
    data_copy(id.a,buf,6);
    NRF_LOG_HEXDUMP_INFO(id.a,6);  
    if (find_bw_list(id.b)==1)
    {
        msg.len = len;
        msg.buf = pvPortMalloc(msg.len);
        memcpy(msg.buf, buf, len);
        xQueueSend(global.queue_sensor_msg, &msg, portMAX_DELAY);
        NRF_LOG_INFO("send to 4g queue");
    }
    else
    {
        NRF_LOG_INFO("not in the white list");  
    }
}

void ack_process(uint8_t buf, un_8u8_u64 id)
{
    if (id.b == sent_id && buf == RSP_END_ACK)
    {
        NRF_LOG_INFO("ACK success");
        //give信号量
    }
}
