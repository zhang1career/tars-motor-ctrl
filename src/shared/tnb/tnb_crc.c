#include "tnb_crc.h"

uint8_t TnbCrc_Byte(uint8_t crc, uint8_t data)
{
  uint8_t i;

  crc ^= data;
  for (i = 0U; i < 8U; i++)
  {
    if ((crc & 0x80U) != 0U)
    {
      crc = (uint8_t)((crc << 1) ^ 0x07U);
    }
    else
    {
      crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t TnbCrc_Buf(uint8_t crc, const uint8_t *buf, uint32_t len)
{
  uint32_t i;

  if (buf == 0)
  {
    return crc;
  }
  for (i = 0U; i < len; i++)
  {
    crc = TnbCrc_Byte(crc, buf[i]);
  }
  return crc;
}
