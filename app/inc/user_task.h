/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-11-03 14:30:23
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\user_task.h
 */

#ifndef _USER_TASK_H_
#define _USER_TASK_H_

void task_lte();
void task_uarte_lte_rx();
void task_uarte_lte_tx();
void task_sx1268();
void sx1268_tx_handler(TimerHandle_t xTimer);
void task_low_power();
void task_tdma();
void task_sx1280();
void task_sx1280_1_dio_process();
void task_sx1280_2_dio_process();
void task_micro_power();
#endif /* _USER_TASK_H_ */