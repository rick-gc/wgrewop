/*
 * @Author       : yfwang
 * @Date         : 2021-06-28 16:17:40
 * @LastEditTime : 2021-09-10 16:56:45
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\inc\saadc.h
 */

#ifndef _SAADC_H_
#define _SAADC_H_

extern un_2u8_u16 voltage;

void saadc_sample(void);
void saadc_callback(nrfx_saadc_evt_t const *p_event);
void saadc_init(void);

#endif /* _SAADC_H_ */