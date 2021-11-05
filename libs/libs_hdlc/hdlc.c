#include "hdlc.h"

/**
 * @brief         HDLC帧编码函数
 * @param p_src   原数据指针
 * @param p_dst   编码后数据位置指针
 * @param src_len 原数据长度
 * @return        返回值为编码后的长度
 * @note          调用时，p_dst空间要大于p_src空间的2倍+2
 */
uint16_t hdlc_frame_encode(uint8_t * p_dst, uint8_t * p_src, uint16_t src_len)
{
  uint16_t i;
  uint16_t j = 0;

  /* 增加FRAME起始标识 */
  p_dst[j] = HDLC_FRAME_PATTERN;
  j++;

  for (i = 0; i < src_len; i++)
  {
    if (p_src[i] == HDLC_FRAME_PATTERN)
    {
      /* 字节流中出现0x7E，则替换成0x7D和0x5E */
      p_dst[j] = HDLC_CONVERT_PATTERN;
      j++;

      p_dst[j] = HDLC_APPEND_DATA0;
      j++;
    }
    else if (p_src[i] == HDLC_CONVERT_PATTERN)
    {
      /* 字节流中出现0x7D，则替换成0x7D和0x5D */
      p_dst[j] = HDLC_CONVERT_PATTERN;
      j++;

      p_dst[j] = HDLC_APPEND_DATA1;
      j++;
    }
    else
    {
      p_dst[j] = p_src[i];
      j++;
    }
  }

  /* 增加尾标识 */
  p_dst[j] = HDLC_FRAME_PATTERN;
  j++;

  /* 返回编码后的字节流长度值 */
  return j;
}

/**
 * @brief         HDLC帧解码函数
 * @param p_src   原数据指针
 * @param p_dst   解码后数据位置指针
 * @param src_len 原数据长度
 * @return        返回值为解码后的长度，返回值为0，表示解码失败
 * @note          调用时，p_dst空间至少等于p_src空间
 */
uint16_t hdlc_frame_decode(uint8_t * p_dst, uint8_t * p_src, uint16_t src_len)
{
  uint16_t i;
  uint16_t j = 0;

  uint8_t frame_pattern_cnt = 0;

  /* 如果第一个字节和最后一个字节都不是0x7E，则直接返回 */
  if ((p_src[0] != HDLC_FRAME_PATTERN) || (p_src[src_len - 1] != HDLC_FRAME_PATTERN))
  {
    return 0;
  }

  for (i = 0; i < src_len; i++)
  {
    if (p_src[i] == HDLC_FRAME_PATTERN)
    {
      frame_pattern_cnt++;
      /* 检测到第二个0x7E，则认为已经到包尾，则跳出循环 */
      if (frame_pattern_cnt == 2)
      {
        break;
      }
    }
    else if (p_src[i] == HDLC_CONVERT_PATTERN)
    {
      /* 合并数据 */
      p_dst[j] = (p_src[i] & 0xF0) | (p_src[i+1] & 0x0F);

      /* 已处理第i+1字节，跳过 */
      i++;

      j++;
    }
    else
    {
      p_dst[j] = p_src[i];
      j++;
    }
  }
  return j;
}

