#include "tvp5150.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c4;       /* created by CubeMX */

static inline void tvp_wr(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c4, 0x5D<<1, reg,
                    I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

int32_t TVP5150_Init(void)
{
  tvp_wr(0x03, 0x80);          /* reset */
  HAL_Delay(5);
  tvp_wr(0x0F, 0x8C);          /* 8-bit BT.656 + embedded sync */
  tvp_wr(0x00, 0x00);          /* NTSC auto, composite in */
  tvp_wr(0x28, 0x20);          /* disable blue-screen */
  return 0;
}
