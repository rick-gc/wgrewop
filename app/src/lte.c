/*
 * @Author       : yfwang
 * @Date         : 2021-09-28 15:55:49
 * @LastEditTime : 2021-11-05 15:24:23
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\lte.c
 */

#include "main.h"

un_8u8_u64 mac_addr;
s_timestamp lte_time, sensor_time;
s_bw_list bw_list;
s_hj_bw_list hj_bw_list;
s_path_config path_config;
s_topology topology;
s_hj_config hj_config;
s_sensor_config_list sensor_config_list;

void mac_addr_init()
{
    mac_addr.c[0] = NRF_FICR->DEVICEADDR[0];
    mac_addr.c[1] = NRF_FICR->DEVICEADDR[1];
    mac_addr.a[7] = 0xff;
    mac_addr.a[6] = 0xff;
    NRF_LOG_INFO("addr %4x%x", mac_addr.c[1], mac_addr.c[0]);
}

void ec600s_io_init()
{
    nrf_gpio_cfg_output(LTE_POWER_EN_PIN);
    nrf_gpio_cfg_output(LTE_PWRKEY_PIN);
    nrf_gpio_cfg_input(LTE_STATUS_PIN, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_pin_set(LTE_POWER_EN_PIN);
    vTaskDelay(msticks(50));
    ec600s_power_button(EC600_OFF);
}

// status:EC600_ON EC600_OFF 开机串口初始化 关机串口反初始化
void ec600s_power_button(bool status)
{
    if (status == EC600_OFF)
    {
        uarte_uninit();
    }
    else
    {
        nrf_gpio_pin_set(LTE_POWER_EN_PIN);
        vTaskDelay(usticks(50 * 1000));
    }
    for (;;)
    {
        if (nrf_gpio_pin_read(LTE_STATUS_PIN) != status)
        {
            nrf_gpio_pin_set(LTE_PWRKEY_PIN);
            vTaskDelay(usticks(LTE_BUTTON_PUSH * 1000));
            nrf_gpio_pin_clear(LTE_PWRKEY_PIN);
            vTaskDelay(usticks(LTE_BUTTON_WAIT * 1000 * 1000));
            if (nrf_gpio_pin_read(LTE_STATUS_PIN) == status)
            {
                if (status == EC600_OFF)
                {
                    nrf_gpio_pin_clear(LTE_POWER_EN_PIN);
                    NRF_LOG_INFO("ec600s close");
                }
                else
                {
                    NRF_LOG_INFO("ec600s open");
                }
                break;
            }
            else
            {
                if (status == EC600_OFF)
                {
                    NRF_LOG_INFO("close ec600 fail");
                }
                else
                {
                    NRF_LOG_INFO("open ec600 fail");
                }
            }
        }
        else
        {
            break;
        }
    }
    if (status == EC600_ON)
    {
        // vTaskDelay(usticks(LTE_START_TIM * 1000 * 1000)); //等待ec600开机
        uarte_lte_init(NRF_UARTE_BAUDRATE_115200);
    }
}

void get_mid(uint8_t *buf)
{
    sd_rand_application_vector_get(buf, 4);
}

void lte_tx(uint8_t *buf, uint8_t len)
{
    msg_t msg, hdlc;
    un_2u8_u16 crc;

    if (len > 0 && buf != NULL)
    {
        msg.len = len + 2;
        msg.buf = pvPortMalloc(msg.len);
        if (msg.buf != NULL)
        {
            memcpy(msg.buf, buf, len);
            crc.b = RTU_CRC(msg.buf, msg.len - 2);
            *(msg.buf + msg.len - 2) = crc.a[1];
            *(msg.buf + msg.len - 1) = crc.a[0];
            hdlc.buf = pvPortMalloc(msg.len * 2 + 2);
            if (hdlc.buf != NULL)
            {
                hdlc.len = hdlc_frame_encode(hdlc.buf, msg.buf, msg.len);
                NRF_LOG_HEXDUMP_INFO(hdlc.buf, hdlc.len);
                if (pdFALSE == xQueueSend(global.queue_uarte_lte_tx_msg, &hdlc, ssticks(10)))
                {
                    freepoint(hdlc.buf);
                }
            }
            freepoint(msg.buf);
        }
    }
}

void lte_reply(uint16_t data, uint32_t id)
{
    uint8_t buf[6];
    un_2u8_u16 type;
    un_4u8_u32 mid;
    type.b = data;
    mid.b = id;
    buf[0] = type.a[1];
    buf[1] = type.a[0];
    buf[2] = mid.a[3];
    buf[3] = mid.a[2];
    buf[4] = mid.a[1];
    buf[5] = mid.a[0];
    lte_tx(buf, 6);
}

void lte_reply_state(uint16_t data, uint32_t id, uint8_t state)
{
    uint8_t buf[6];
    un_2u8_u16 type;
    un_4u8_u32 mid;
    type.b = data;
    mid.b = id;
    buf[0] = type.a[1];
    buf[1] = type.a[0];
    buf[2] = mid.a[3];
    buf[3] = mid.a[2];
    buf[4] = mid.a[1];
    buf[5] = mid.a[0];
    buf[6] = state;
    lte_tx(buf, 7);
}

void lte_close()
{
    uint8_t buf[2];
    un_2u8_u16 crc;

    buf[0] = 0xaa;
    buf[1] = 0xaa;
    lte_tx(buf, 2);
}

void lte_get_time_tx()
{
    uint8_t buf[6];
    un_2u8_u16 crc;

    buf[0] = 0x82;
    buf[1] = 0x04;
    get_mid(buf + 2);
    lte_tx(buf, 6);
}

void lte_get_time_rx(uint8_t *buf)
{
    data_copy(lte_time.timestamp.a, buf + 2, 8);
}

void lte_version_tx()
{
    uint8_t buf[24];
    un_8u8_u64 id;
    un_4u8_u32 hwver, sfver;
    un_2u8_u16 crc;

    id.b = EID;
    hwver.b = HVER;
    sfver.b = SVER;

    buf[0] = 0x81;
    buf[1] = 0x01;
    get_mid(buf + 2);
    data_copy(buf + 6, id.a, 6);
    data_copy(buf + 12, hwver.a, 3);
    data_copy(buf + 15, sfver.a, 3);
    data_copy(buf + 18, id.a, 6);
    lte_tx(buf, 24);
}

void lte_voltage_tx()
{
    uint8_t buf[10];
    un_2u8_u16 crc;
    un_4u8_u32 mcu_temperature;

    buf[0] = 0x81;
    buf[1] = 0x02;
    get_mid(buf + 2);
    nrf_gpio_pin_set(ADC_PIN);
    vTaskDelay(msticks(1));
    saadc_sample();
    xSemaphoreTake(global.semphore_saadc_sample_done, ssticks(1));
    nrf_gpio_pin_clear(ADC_PIN);
    buf[6] = voltage.a[1];
    buf[7] = voltage.a[0];
    sd_temp_get(&mcu_temperature.b);
    buf[8] = mcu_temperature.a[1];
    buf[9] = mcu_temperature.a[0];
    NRF_LOG_INFO("temperature" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT((float)mcu_temperature.b / 4));
    lte_tx(buf, 10);
}

void pack_sensor_msg()
{
    msg_t msg;
    un_2u8_u16 crc;
    uint8_t sensor_num = 0, topology_num = 1;
    uint8_t sensor_data_buf[2048] = {0x80, 0x01};
    uint8_t topology_buf[1024] = {0x81, 0x03, 0, 0, 0, 0, 0x01, 0x01, 0x01,
                                  0x01, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0, 0, 1, 0xea};
    uint16_t sensor_data_len = 9, topology_len = 28;

    get_mid(sensor_data_buf + 2);
    get_mid(topology_buf + 2);
    for (;;)
    {
        if (pdTRUE == xQueueReceive(global.queue_sensor_msg, &msg, usticks(1000 * 1000)))
        {
            sensor_num++;
            sensor_data_buf[8] = sensor_num;
            sensor_data_buf[sensor_data_len] = msg.len + 8;
            memcpy(sensor_data_buf + sensor_data_len + 1, msg.buf, msg.len);
            sensor_data_len += msg.len + 1 + 8;
            /******插入时间戳****************/
            sensor_time.timestamp.b = (xTaskGetTickCount() / 8192) - lte_time.system_time + lte_time.timestamp.b;
            sensor_data_buf[sensor_data_len - 8] = sensor_time.timestamp.a[7];
            sensor_data_buf[sensor_data_len - 7] = sensor_time.timestamp.a[6];
            sensor_data_buf[sensor_data_len - 6] = sensor_time.timestamp.a[5];
            sensor_data_buf[sensor_data_len - 5] = sensor_time.timestamp.a[4];
            sensor_data_buf[sensor_data_len - 4] = sensor_time.timestamp.a[3];
            sensor_data_buf[sensor_data_len - 3] = sensor_time.timestamp.a[2];
            sensor_data_buf[sensor_data_len - 2] = sensor_time.timestamp.a[1];
            sensor_data_buf[sensor_data_len - 1] = sensor_time.timestamp.a[0];
            /*******************************/
            NRF_LOG_INFO("SX1268 Sensor Data len[[[[[%d]]]]]", sensor_data_len);

            topology_num++;
            topology_buf[8] = topology_num;
            topology_buf[topology_len] = 0x02;
            memcpy(topology_buf + topology_len + 1, msg.buf, 6);
            topology_buf[topology_len + 7] = 0x06;
            topology_buf[topology_len + 8] = 0x01;
            topology_buf[topology_len + 9] = 0x02;
            topology_buf[topology_len + 10] = 0x03;
            topology_buf[topology_len + 11] = 0x04;
            topology_buf[topology_len + 12] = 0x05;
            topology_buf[topology_len + 13] = 0x06;
            topology_buf[topology_len + 14] = sx1268.packet_status.Params.LoRa.RssiPkt;
            topology_buf[topology_len + 15] = sx1268.packet_status.Params.LoRa.SnrPkt;
            topology_buf[topology_len + 16] = 490 >> 8;
            topology_buf[topology_len + 17] = (uint8_t)490;
            topology_len += 18;
            freepoint(msg.buf);
        }
        else
        {
            if (sensor_num > 0)
            {
                sensor_data_len += 2;
                sensor_data_buf[6] = sensor_data_len >> 8;
                sensor_data_buf[7] = (uint8_t)sensor_data_len;
                // NRF_LOG_HEXDUMP_INFO(sensor_data_buf, sensor_data_len);
                lte_tx(sensor_data_buf, sensor_data_len - 2);
                NRF_LOG_INFO("SX1268 Sensor Data Queuesend ok %d", sensor_data_len - 2);
                sensor_num = 0;
                sensor_data_len = 9;

                // NRF_LOG_HEXDUMP_INFO(topology_buf, topology_len);
                lte_tx(topology_buf, topology_len);
                NRF_LOG_INFO("SX1268 topology Queuesend ok %d ", topology_len);
                topology_num = 1;
                topology_len = 28;
            }
            break;
        }
    }
}

// 0:不在名单中 1:在白名单中 2:在黑名单中 3:缓存名单已满
uint8_t find_bw_list(uint64_t id)
{
    if (find_same(id, bw_list.dev_white_list, bw_list.dev_white_list_num) ||
        find_same(id, bw_list.sen_white_list, bw_list.sen_white_list_num))
    {
        NRF_LOG_INFO("in the white list");
        return 1;
    }
    else
    {
        if (find_same(id, bw_list.dev_black_list, bw_list.dev_black_list_num) ||
            find_same(id, bw_list.sen_black_list, bw_list.sen_black_list_num))
        {
            NRF_LOG_INFO("in the black list");
            return 2;
        }
        else
        {
            if (find_same(id, bw_list.temp_list, bw_list.temp_list_num))
            {
                NRF_LOG_INFO("in the temp list");
                return 0;
            }
            else
            {
                if (bw_list.temp_list_num < 200)
                {
                    bw_list.temp_list[bw_list.temp_list_num].b = id;
                    bw_list.temp_list_num++;
                    NRF_LOG_INFO("save in bw_list.temp_list");
                    return 0;
                }
                else
                {
                    NRF_LOG_INFO("The temp_list of black and white list is full");
                    return 3;
                }
            }
        }
    }
}

void path_config_rx(uint8_t *buf)
{
    uint8_t num, path;
    path = *buf;
    num = *(buf + 1);
    if (path == 1 || path == 3 || path == 4)
    {
        for (uint8_t n = 0; n < num; n++)
        {
            switch (*(buf + 2 + n * 3))
            {
            case 1:
                path_config.frequency[path].a[1] = *(buf + 3 + n * 3);
                path_config.frequency[path].a[0] = *(buf + 4 + n * 3);
                NRF_LOG_INFO("frequency %x", path_config.frequency[path].b);
                break;
            case 2:
                path_config.power[path].a[1] = *(buf + 3 + n * 3);
                path_config.power[path].a[0] = *(buf + 4 + n * 3);
                NRF_LOG_INFO("power %x", path_config.power[path].b);
                break;
            default:
                break;
            }
        }
    }
}

void path_config_tx()
{
    uint8_t buf[16] = {0x03, 0x02, 0, 0, 0, 0, 0x00, 1, 0, 2, 1, 0, 0, 2, 0, 0};
    get_mid(buf + 2);
    buf[9] = 1;
    buf[11] = path_config.frequency[1].a[1];
    buf[12] = path_config.frequency[1].a[0];
    buf[14] = path_config.power[1].a[1];
    buf[15] = path_config.power[1].a[0];
    lte_tx(buf, 16);

    get_mid(buf + 2);
    buf[9] = 3;
    buf[11] = path_config.frequency[3].a[1];
    buf[12] = path_config.frequency[3].a[0];
    buf[14] = path_config.power[3].a[1];
    buf[15] = path_config.power[3].a[0];
    lte_tx(buf, 16);

    get_mid(buf + 2);
    buf[9] = 4;
    buf[11] = path_config.frequency[4].a[1];
    buf[12] = path_config.frequency[4].a[0];
    buf[14] = path_config.power[41].a[1];
    buf[15] = path_config.power[4].a[0];
    lte_tx(buf, 16);
}

void bw_list_hj_rx(uint8_t *buf)
{
    un_8u8_u64 id;
    uint8_t flag, num, data, len;
    flag = *buf;
    num = *(buf + 1);
    data = 2;
    if (flag)
    {
        for (uint8_t n = 0; n < num; n++)
        {
            len = *(buf + data);
            if (len < 7)
            {
                id.a[6] = 1;
                data_copy(id.a, buf + data + 1, len);
                if (!find_same(id.b, bw_list.dev_white_list, bw_list.dev_white_list_num))
                {
                    bw_list.dev_white_list[bw_list.dev_white_list_num].b = id.b;
                    NRF_LOG_INFO("%x", bw_list.dev_white_list[bw_list.dev_white_list_num].b);
                    bw_list.dev_white_list_num++;
                }
            }
            data += len + 1;
        }
    }
    else
    {
        for (uint8_t n = 0; n < num; n++)
        {
            len = *(buf + data);

            if (len < 7)
            {
                id.a[6] = 1;
                data_copy(id.a, buf + data + 1, len);
                if (!find_same(id.b, bw_list.dev_black_list, bw_list.dev_black_list_num))
                {
                    bw_list.dev_black_list[bw_list.dev_black_list_num].b = id.b;
                    NRF_LOG_INFO("%x", bw_list.dev_black_list[bw_list.dev_black_list_num].b);
                    bw_list.dev_black_list_num++;
                }
            }
            data += len + 1;
        }
    }
}

void bw_list_hj_tx()
{
    uint8_t *buf;

    buf = pvPortMalloc(8 + 7 * bw_list.dev_white_list_num);
    *buf = 0x03;
    *(buf + 1) = 0x52;
    get_mid(buf + 2);
    *(buf + 6) = 1;
    *(buf + 7) = bw_list.dev_white_list_num;
    for (uint8_t n = 0; n < bw_list.dev_white_list_num; n++)
    {
        *(buf + 8 + 7 * n) = 6;
        data_copy(buf + 9 + 7 * n, bw_list.dev_white_list[n].a, 6);
    }
    lte_tx(buf, 8 + 7 * bw_list.dev_white_list_num);
    freepoint(buf);

    buf = pvPortMalloc(8 + 7 * bw_list.dev_black_list_num);
    *buf = 0x03;
    *(buf + 1) = 0x52;
    get_mid(buf + 2);
    *(buf + 6) = 0;
    *(buf + 7) = bw_list.dev_black_list_num;
    for (uint8_t n = 0; n < bw_list.dev_black_list_num; n++)
    {
        *(buf + 8 + 7 * n) = 6;
        data_copy(buf + 9 + 7 * n, bw_list.dev_black_list[n].a, 6);
    }
    lte_tx(buf, 8 + 7 * bw_list.dev_black_list_num);
    freepoint(buf);
}

void bw_list_sensor_rx(uint8_t *buf)
{
    un_8u8_u64 id;
    uint8_t flag, type, num;
    flag = *buf;
    type = *(buf + 1);
    if (type == 1)
    {
        type = 2;
    }
    id.a[6] = type;
    num = *(buf + 2);
    if (flag)
    {
        for (uint8_t n = 0; n < num; n++)
        {
            data_copy(id.a, buf + 3 + n * 8, 6);
            if (!find_same(id.b, bw_list.sen_white_list, bw_list.sen_white_list_num))
            {
                bw_list.sen_white_list[bw_list.sen_white_list_num].b = id.b;
                NRF_LOG_INFO("white");
                NRF_LOG_HEXDUMP_INFO(bw_list.sen_white_list[bw_list.sen_white_list_num].a, 6);
                bw_list.sen_white_list_num++;
            }
        }
    }
    else
    {
        for (uint8_t n = 0; n < num; n++)
        {
            data_copy(id.a, buf + 3 + n * 8, 6);
            if (!find_same(id.b, bw_list.sen_black_list, bw_list.sen_black_list_num))
            {
                bw_list.sen_black_list[bw_list.sen_black_list_num].b = id.b;
                NRF_LOG_INFO("black");
                NRF_LOG_HEXDUMP_INFO(bw_list.sen_black_list[bw_list.sen_black_list_num].a, 6);
                bw_list.sen_black_list_num++;
            }
        }
    }
}

void bw_list_sensor_tx()
{
    uint8_t *buf;
    uint8_t num;

    num = 0;
    NRF_LOG_INFO("%d", bw_list.sen_white_list_num);
    buf = pvPortMalloc(11 + 6 * bw_list.sen_white_list_num);
    *buf = 0x03;
    *(buf + 1) = 0x62;
    get_mid(buf + 2);
    *(buf + 6) = 1;
    *(buf + 7) = 1;
    *(buf + 8) = 1;
    *(buf + 9) = 0;
    for (uint8_t n = 0; n < bw_list.sen_white_list_num; n++)
    {
        if (bw_list.sen_white_list[n].a[6] == 0)
        {
            data_copy(buf + 11 + 6 * num, bw_list.sen_white_list[n].a, 6);
            num++;
        }
    }
    *(buf + 10) = num;
    lte_tx(buf, 11 + 6 * num);
    num = 0;

    *buf = 0x03;
    *(buf + 1) = 0x62;
    get_mid(buf + 2);
    *(buf + 6) = 1;
    *(buf + 7) = 1;
    *(buf + 8) = 1;
    *(buf + 9) = 1;
    for (uint8_t n = 0; n < bw_list.sen_white_list_num; n++)
    {
        if (bw_list.sen_white_list[n].a[6] == 2)
        {
            data_copy(buf + 11 + 6 * num, bw_list.sen_white_list[n].a, 6);
            num++;
        }
    }
    *(buf + 10) = num;
    lte_tx(buf, 11 + 6 * num);
    num = 0;
    freepoint(buf);

    NRF_LOG_INFO("%d", bw_list.sen_black_list_num);
    buf = pvPortMalloc(11 + 6 * bw_list.sen_black_list_num);
    *buf = 0x03;
    *(buf + 1) = 0x62;
    get_mid(buf + 2);
    *(buf + 6) = 1;
    *(buf + 7) = 1;
    *(buf + 8) = 1;
    *(buf + 9) = 0;
    for (uint8_t n = 0; n < bw_list.sen_black_list_num; n++)
    {
        if (bw_list.sen_black_list[n].a[6] == 0)
        {
            data_copy(buf + 11 + 6 * num, bw_list.sen_black_list[n].a, 6);
            num++;
        }
    }
    *(buf + 10) = num;
    lte_tx(buf, 11 + 6 * num);
    num = 0;

    *buf = 0x03;
    *(buf + 1) = 0x62;
    get_mid(buf + 2);
    *(buf + 6) = 1;
    *(buf + 7) = 1;
    *(buf + 8) = 1;
    *(buf + 9) = 1;
    for (uint8_t n = 0; n < bw_list.sen_black_list_num; n++)
    {
        if (bw_list.sen_black_list[n].a[6] == 2)
        {
            data_copy(buf + 11 + 6 * num, bw_list.sen_black_list[n].a, 6);
            num++;
        }
    }
    *(buf + 10) = num;
    lte_tx(buf, 11 + 6 * num);
    num = 0;
    freepoint(buf);
}

void hj_config_rx(uint8_t *buf)
{
    un_8u8_u64 id;
    uint8_t len, path, type, num;
    len = *buf;
    path = *(buf + len + 1);
    num = *(buf + len + 2);
    data_copy(id.a, buf + 1, len);
    if (find_same(id.b, hj_config.list, hj_config.num))
    {
        for (uint8_t m; m < hj_config.num; m++)
        {
            if (id.b == hj_config.list[m].b)
            {
                for (uint8_t n = 0; n < num; n++)
                {
                    switch (*(buf + len + 3 + n * 3))
                    {
                    case 1:
                        hj_config.path_config[m].frequency[path].a[1] = *(buf + len + 4 + n * 3);
                        hj_config.path_config[m].frequency[path].a[0] = *(buf + len + 5 + n * 3);
                        break;
                    case 2:
                        hj_config.path_config[m].power[path].a[1] = *(buf + len + 4 + n * 3);
                        hj_config.path_config[m].power[path].a[0] = *(buf + len + 5 + n * 3);
                        break;
                    default:
                        break;
                    }
                }
                break;
            }
        }
    }
    else
    {
        hj_config.list[hj_config.num].b = id.b;
        for (uint8_t n = 0; n < num; n++)
        {
            switch (*(buf + len + 3 + n * 3))
            {
            case 1:
                hj_config.path_config[hj_config.num].frequency[path].a[1] = *(buf + len + 4 + n * 3);
                hj_config.path_config[hj_config.num].frequency[path].a[0] = *(buf + len + 5 + n * 3);
                break;
            case 2:
                hj_config.path_config[hj_config.num].power[path].a[1] = *(buf + len + 4 + n * 3);
                hj_config.path_config[hj_config.num].power[path].a[0] = *(buf + len + 5 + n * 3);
                break;
            default:
                break;
            }
            hj_config.num++;
        }
    }
}

void hj_config_tx()
{
    uint8_t buf[23] = {0x02, 0x12, 0, 0, 0, 0, 0x01, 6, 0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 0, 0, 2, 0, 0};

    for (uint8_t n = 0; n < hj_config.num; n++)
    {
        for (uint8_t m = 0; m < 5; m++)
        {
            if (m != 2)
            {
                get_mid(buf + 2);
                data_copy(buf + 8, hj_config.list[n].a, 6);
                *(buf + 15) = m;
                *(buf + 18) = hj_config.path_config[n].frequency[m].a[1];
                *(buf + 19) = hj_config.path_config[n].frequency[m].a[0];
                *(buf + 21) = hj_config.path_config[n].power[m].a[1];
                *(buf + 22) = hj_config.path_config[n].power[m].a[1];
                lte_tx(buf, 23);
            }
        }
    }
}

void hj_bw_list_rx(uint8_t *buf)
{
    un_8u8_u64 mid, eid;
    uint8_t flag, len, type, num;

    len = *buf;
    if (len > 6)
    {
        return;
    }
    mid.a[6] = 1;
    data_copy(mid.a, buf + 1, len);
    flag = *(buf + len + 1);
    type = *(buf + len + 2);
    num = *(buf + len + 3);

    if (flag)
    {
        if (find_same(mid.b, hj_bw_list.hj_white_list, hj_bw_list.hj_white_list_num) == 0)
        {
            hj_bw_list.hj_white_list[hj_bw_list.hj_white_list_num].b = mid.b;
            hj_bw_list.hj_white_list_num++;
        }
        for (uint8_t m = 0; m < num; m++)
        {
            eid.a[6] = type;
            data_copy(eid.a, buf + len + 4 + m * 6, 6);

            flag = 0;
            for (uint8_t n = 0; n < hj_bw_list.white_list_num; n++)
            {
                if (mid.b == hj_bw_list.white_list[0][n].b &&
                    eid.b == hj_bw_list.white_list[1][n].b)
                {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0)
            {
                hj_bw_list.white_list[0][hj_bw_list.white_list_num].b = mid.b;
                hj_bw_list.white_list[1][hj_bw_list.white_list_num].b = eid.b;
                hj_bw_list.white_list_num++;
            }
        }
    }
    else
    {
        if (find_same(mid.b, hj_bw_list.hj_black_list, hj_bw_list.hj_black_list_num) == 0)
        {
            hj_bw_list.hj_black_list[hj_bw_list.hj_black_list_num].b = mid.b;
            hj_bw_list.hj_black_list_num++;
        }
        for (uint8_t m = 0; m < num; m++)
        {
            eid.a[6] = type;
            data_copy(eid.a, buf + len + 4 + m * 6, 6);

            flag = 0; //黑白名单标志位 复用一下 当做去重的标志位
            for (uint8_t n = 0; n < hj_bw_list.black_list_num; n++)
            {
                if (mid.b == hj_bw_list.black_list[0][n].b &&
                    eid.b == hj_bw_list.black_list[1][n].b)
                {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0)
            {
                hj_bw_list.black_list[0][hj_bw_list.black_list_num].b = mid.b;
                hj_bw_list.black_list[1][hj_bw_list.black_list_num].b = eid.b;
                hj_bw_list.black_list_num++;
            }
        }
    }
}

void hj_bw_list_tx()
{
    uint8_t *buf;
    uint8_t num;

    num = 0;
    buf = pvPortMalloc(1024);
    *buf = 0x02;
    *(buf + 1) = 0x02;
    *(buf + 6) = 1;
    *(buf + 7) = 1;
    *(buf + 8) = 6;

    *(buf + 15) = 1;
    for (uint8_t m = 0; m < hj_bw_list.hj_white_list_num; m++)
    {
        get_mid(buf + 2);
        data_copy(buf + 9, hj_bw_list.hj_white_list[m].a, 6);
        *(buf + 16) = 0;
        for (uint8_t n = 0; n < hj_bw_list.white_list_num; n++)
        {
            if (hj_bw_list.hj_white_list[m].b == hj_bw_list.white_list[0][n].b &&
                hj_bw_list.white_list[1][n].a[6] == 0)
            {
                data_copy(buf + 18 + 6 * num, hj_bw_list.white_list[1][n].a, 6);
                num++;
            }
        }
        *(buf + 17) = num;
        lte_tx(buf, 18 + num * 6);
        num = 0;

        get_mid(buf + 2);
        *(buf + 16) = 1;
        for (uint8_t n = 0; n < hj_bw_list.white_list_num; n++)
        {
            if (hj_bw_list.hj_white_list[m].b == hj_bw_list.white_list[0][n].b &&
                hj_bw_list.white_list[1][n].a[6] == 1)
            {
                data_copy(buf + 18 + 6 * num, hj_bw_list.white_list[1][n].a, 6);
                num++;
            }
        }
        *(buf + 17) = num;
        lte_tx(buf, 18 + num * 6);
        num = 0;
    }

    *(buf + 15) = 0;
    for (uint8_t m = 0; m < hj_bw_list.hj_black_list_num; m++)
    {
        data_copy(buf + 9, hj_bw_list.hj_black_list[m].a, 6);
        get_mid(buf + 2);
        *(buf + 16) = 0;
        for (uint8_t n = 0; n < hj_bw_list.black_list_num; n++)
        {
            if (hj_bw_list.hj_black_list[m].b == hj_bw_list.black_list[0][n].b &&
                hj_bw_list.black_list[1][n].a[6] == 0)
            {
                data_copy(buf + 18 + 6 * num, hj_bw_list.black_list[1][n].a, 6);
                num++;
            }
        }
        *(buf + 17) = num;
        lte_tx(buf, 18 + num * 6);
        num = 0;

        get_mid(buf + 2);
        *(buf + 16) = 1;
        for (uint8_t n = 0; n < hj_bw_list.black_list_num; n++)
        {
            if (hj_bw_list.hj_black_list[m].b == hj_bw_list.black_list[0][n].b &&
                hj_bw_list.black_list[1][n].a[6] == 1)
            {
                data_copy(buf + 18 + 6 * num, hj_bw_list.black_list[1][n].a, 6);
                num++;
            }
        }
        *(buf + 17) = num;
        lte_tx(buf, 18 + num * 6);
        num = 0;
    }
    freepoint(buf);
}

void sensor_config_rx(uint8_t *buf)
{
    un_8u8_u64 id;
    uint8_t type, num;
    id.a[6] = *buf;
    data_copy(id.a, buf + 1, 6);
    if (0 == find_same_num(id.b, sensor_config_list.list, sensor_config_list.num, &num))
    {
        num = sensor_config_list.num;
        sensor_config_list.list[num].b = id.b;
    }
    type = *(buf + 8);
    switch (type)
    {
    case 2:
        sensor_config_list.sensor_config[num].power.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].power.a[0] = *(buf + 10);
        break;
    case 3:
        sensor_config_list.sensor_config[num].msg_cycle.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].msg_cycle.a[0] = *(buf + 10);
        break;
    case 4:
        sensor_config_list.sensor_config[num].control_cycle.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].control_cycle.a[0] = *(buf + 10);
        break;
    case 5:
        sensor_config_list.sensor_config[num].drx_cycle.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].drx_cycle.a[0] = *(buf + 10);
        break;
    case 6:
        sensor_config_list.sensor_config[num].control_frequency.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].control_frequency.a[0] = *(buf + 10);
        break;
    case 7:
        sensor_config_list.sensor_config[num].msg_frequency.a[1] = *(buf + 9);
        sensor_config_list.sensor_config[num].msg_frequency.a[0] = *(buf + 10);
        break;
    default:
        break;
    }
    sensor_config_list.num++;
}

void sensor_config_reply(uint32_t id, uint8_t *buf)
{
    un_4u8_u32 mid;
    un_8u8_u64 eid;
    uint8_t *data;
    uint8_t type, type_num, num, group;

    mid.b = id;
    eid.a[6] = *buf;
    data_copy(eid.a, buf + 1, 6);
    group = *(buf + 7);
    type_num = *(buf + 8);

    if (find_same_num(eid.b, sensor_config_list.list, sensor_config_list.num, &num))
    {
        data = pvPortMalloc(15 + type_num * 3);
        data[0] = 0x01;
        data[1] = 0x10;
        for (uint8_t n = 0; n < type_num; n++)
        {
            type = *(buf + 9 + n);
            data[15 + 3 * n] = type;
            switch (type)
            {
            case 2:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].power.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].power.a[0];
                break;
            case 3:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].msg_cycle.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].msg_cycle.a[0];
                break;
            case 4:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].control_cycle.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].control_cycle.a[0];
                break;
            case 5:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].drx_cycle.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].drx_cycle.a[0];
                break;
            case 6:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].control_frequency.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].control_frequency.a[0];
                break;
            case 7:
                *(data + 16 + 3 * n) = sensor_config_list.sensor_config[num].msg_frequency.a[1];
                *(data + 17 + 3 * n) = sensor_config_list.sensor_config[num].msg_frequency.a[0];
                break;
            default:
                *(data + 16 + 3 * n) = 0xff;
                *(data + 17 + 3 * n) = 0xff;
                break;
            }
        }
        data_copy(data + 2, mid.a, 4);
        data[6] = eid.a[6];
        data_copy(data + 7, eid.a, 6);
        data[13] = group;
        data[14] = type_num;
        lte_tx(data, 15 + type_num * 3);
        freepoint(data);
    }
}