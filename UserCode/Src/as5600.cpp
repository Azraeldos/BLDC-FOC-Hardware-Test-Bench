#include "as5600.h"
#include "foc_math.h"
#include "user_params.h"
#include "i2c.h"

static constexpr uint16_t AS5600_ADDR      = (0x36 << 1);
static constexpr uint8_t  AS5600_REG_ANGLE = 0x0E;

void as5600_init(void)
{
  g_i2c_nfound = 0;
  for (uint8_t addr7 = 1; addr7 < 127; ++addr7) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, 10) == HAL_OK) {
      if (g_i2c_nfound < 8) {
        g_i2c_addrs[g_i2c_nfound++] = addr7;
      }
    }
  }

  g_enc_found = 0;
  for (uint8_t i = 0; i < g_i2c_nfound; ++i) {
    if (g_i2c_addrs[i] == 0x36) {
      g_enc_found = 1;
      break;
    }
  }
}

uint16_t as5600_read_raw(void)
{
  uint8_t buf[2] = {0, 0};
  g_enc_status = HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_ANGLE,
                                  I2C_MEMADD_SIZE_8BIT, buf, 2, 2);
  if (g_enc_status != HAL_OK) {
    return g_enc_raw;
  }
  return (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
}

float as5600_raw_to_mech(uint16_t raw)
{
  return (static_cast<float>(raw) / 4096.0f) * FOC_TWO_PI;
}

float as5600_mech_to_elec(float mech, float zero_offset, uint8_t pole_pairs)
{
  return foc_wrap_0_2pi((mech - zero_offset) * static_cast<float>(pole_pairs));
}
