/*
 * @Author       : yfwang
 * @Date         : 2021-09-16 14:40:38
 * @LastEditTime : 2021-09-28 10:23:47
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\sx1280_platform.h
 */

#ifndef _SX1280_PLATFORM_H_
#define _SX1280_PLATFORM_H_


#define SX1280_COM_Freq 2400500000
#define SX1280_MEG_Freq 2424500000

extern SX128X_TickTime_t sx1280TickTime;

void sx1280_user_init(void);

#endif /* _SX1280_PLATFORM_H_ */