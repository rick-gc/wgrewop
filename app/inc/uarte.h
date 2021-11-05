/*
 * @Author       : yfwang
 * @Date         : 2021-10-14 15:08:03
 * @LastEditTime : 2021-10-14 19:29:28
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\uarte.h
 */

#ifndef _UARTE_H_
#define _UARTE_H_

void uarte_lte_init(nrf_uarte_baudrate_t baudRate);
void uarte_uninit();

#endif /* _UARTE_H_ */