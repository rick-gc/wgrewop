/*
 * @Author       : yfwang
 * @Date         : 2021-09-23 18:38:29
 * @LastEditTime : 2021-11-05 20:41:13
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\lpower.c
 */

#include "main.h"

lpower_message_s lpower_message;
lpower_dcch_s lpower_dcch = DCCH_DEFAULT_DATA;
lpower_usch_s lpower_usch;
lpower_dsch_s lpower_dsch;

uint8_t bch[55] = {0x02, 0x16, 0, 0, 0, 0, 0, 5, 0xff, 0xff, 0, 0, 0, 1, 100, 100, 10, 10, 10, 10, 55, 0x48, 100, 0};

void urch_process(uint8_t *buf, uint8_t len)
{
    if (*buf == MASTER_ID_H && *(buf + 1) == MASTER_ID_L)
    {
        switch (*(buf + 2))
        {
        case 0x00: //上行资源请求
            break;
        case 0x01: //随机接入请求

            lpower_dcch.access_id.a[7] = 0xff;
            lpower_dcch.access_id.a[6] = 0xff;
            lpower_dcch.access_id.a[5] = *(buf + 3);
            lpower_dcch.access_id.a[4] = *(buf + 4);
            lpower_dcch.access_id.a[3] = *(buf + 5);
            lpower_dcch.access_id.a[2] = *(buf + 6);
            lpower_dcch.access_id.a[1] = *(buf + 7);
            lpower_dcch.access_id.a[0] = *(buf + 8);
            lpower_dcch.node_type = *(buf + 9);
            //黑白名单判断!!!
            if (lpower_dcch.access_flag != 1) // 0 无调度 1 usch调度和注册应答 2 usch调度
            {
                lpower_dcch.access_flag = 1;
                NRF_LOG_HEXDUMP_INFO(lpower_dcch.access_id.a, 8);

                if (false == find_same(lpower_dcch.access_id.b, topology.list[1], topology.num))
                {
                    topology.list[0][topology.num].b = 0xffffffff;
                    topology.list[1][topology.num].b = lpower_dcch.access_id.b;
                    // NRF_LOG_HEXDUMP_INFO(topology.list[1][topology.num].a, 8);
                    topology.num++;
                    lpower_dcch.slave_num++;
                }
                dcch_process();
            }
            break;
        case 0x02: //突发短数据
            break;
        default:
            break;
        }
    }
}

void dcch_process()
{
    if (lpower_dcch.access_flag == 1)
    {
        lpower_dcch.node_num = lpower_dcch.slave_num;
        lpower_dcch.type = 0;
        lpower_dcch.buf[4] = lpower_dcch.type_s;
        for (uint8_t n = 0; n < lpower_dcch.slave_num; n++)
        {
            lpower_dcch.buf[4 * n + 5] = lpower_dcch.slave_id;
            lpower_dcch.buf[4 * n + 6] = n + 1;
            lpower_dcch.buf[4 * n + 7] = n * 20;
            lpower_dcch.buf[4 * n + 8] = (n + 1) * 20;
        }

        lpower_dcch.node_num = 1;
        lpower_dcch.type = 2;
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 5] = lpower_dcch.type_s;
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 6] = lpower_dcch.access_id.a[5];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 7] = lpower_dcch.access_id.a[4];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 8] = lpower_dcch.access_id.a[3];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 9] = lpower_dcch.access_id.a[2];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 10] = lpower_dcch.access_id.a[1];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 11] = lpower_dcch.access_id.a[0];
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 12] = lpower_dcch.slave_id;
        lpower_dcch.buf[lpower_dcch.slave_num * 4 + 13] = lpower_dcch.slave_num;
        lpower_dcch.len = 12 + lpower_dcch.slave_num * 4;
        lpower_dcch.buf[1] = lpower_dcch.len;
    }
}

void usch_mac_process(uint8_t *buf, uint8_t len)
{
    if (*buf == MASTER_ID_H && *(buf + 1) == MASTER_ID_L)
    {
        //存储从地址和对应id!!!
        //拓扑存储 汇聚到接入!!!
        NRF_LOG_INFO("usch_mac_process");
    }
}

void usch_msg_data_analyze(uint8_t *buf, uint8_t len)
{
    msg_t msg;
    uint8_t num; //负载开始的字节号

    lpower_usch.msg_type = *buf;
    if (lpower_usch.msg_num_flag)
    {
        num++;
    }
    if (lpower_usch.port_flag)
    {
        num++;
    }
    if (lpower_usch.device_eid_flag)
    {
        data_copy(lpower_usch.device_id.a, buf + 1 + num, 6);
        num += 6;
    }
    if (lpower_usch.sensor_eid_flag)
    {
        data_copy(lpower_usch.sensor_id.a, buf + 1 + num, 6);
        num += 6;
    }
    if (!lpower_usch.msg_flag && lpower_usch.uod_flag && lpower_usch.device_eid_flag)
    {
        //拓扑存储 传感器到汇聚!!!
        msg.len = *(buf + 1 + num);
        if (msg.len > 8)
        {
            msg.len -= 8;
            msg.buf = pvPortMalloc(msg.len);
            memcpy(msg.buf, buf + 16, msg.len);
            xQueueSend(global.queue_sensor_msg, &msg, portMAX_DELAY);
        }
    }
}

void usch_msg_process(uint8_t *buf, uint8_t len)
{
    uint8_t num, data_len[20];

    if (*buf == MASTER_ID_H && *(buf + 1) == MASTER_ID_L)
    {
        NRF_LOG_INFO("usch_msg_process");
        //判断从设备是否注册!!!
        lpower_usch.type_s = *(buf + 4);
        if (lpower_usch.type_len == 0 && lpower_usch.pic == 1)
        {
            for (;;)
            {
                lpower_usch.size = *(buf + data_len[num] + 4 + 3 * (num + 1));
                data_len[num + 1] = lpower_usch.size;
                data_len[0] += data_len[num + 1];
                num++;
                if ((data_len[0] + 5 + 3 * num) == len)
                {
                    break;
                }
                else if ((data_len[0] + 5 + 3 * num) > len)
                {
                    num = 0;
                    break;
                }
            }
            data_len[0] = 0;
            for (uint8_t n; n < num; n++)
            {
                usch_msg_data_analyze(buf + (num + 1) * 4 + 4 + data_len[num], data_len[num + 1]);
            }
        }
    }
}
