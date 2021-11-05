#ifndef HDLC_H__
#define HDLC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 *
 * @defgroup hdlc HDLC encoding and decoding
 * @{
 * @ingroup app_common
 *
 * @brief  This module encodes and decodes HDLC packages.
 *
 * @details The HDLC protocol is described in @linkHDLC.
 */

#define HDLC_FRAME_PATTERN				  0x7E
#define HDLC_CONVERT_PATTERN			  0x7D
#define HDLC_APPEND_DATA0				    0x5E
#define HDLC_APPEND_DATA1				    0x5D

uint16_t hdlc_frame_encode(uint8_t * p_dst, uint8_t * p_src, uint16_t src_len);
uint16_t hdlc_frame_decode(uint8_t * p_dst, uint8_t * p_src, uint16_t src_len);

#ifdef __cplusplus
}
#endif

#endif // HDLC_H__

/** @} */
