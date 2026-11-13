#include "user_main.h"
#include "main.h"
#include "i2c.h"

// AS5600 default 7-bit address. HAL wants it left-shifted.
static constexpr uint16_t AS5600_ADDR = (0x36 << 1);
static constexpr uint8_t  AS5600_REG_ANGLE = 0x0E;  // raw angle, 12-bit

// Watch these in the debugger Live Expressions / Watch window
volatile uint8_t  g_enc_found = 0;
volatile uint16_t g_enc_raw   = 0;
volatile HAL_StatusTypeDef g_enc_status = HAL_ERROR;
volatile uint8_t  g_i2c_addrs[8] = {0};
volatile uint8_t  g_i2c_nfound = 0;

static void i2c_bus_scan(void)
{
  g_i2c_nfound = 0;
  for (uint8_t addr7 = 1; addr7 < 127; ++addr7) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, 10) == HAL_OK) {
      if (g_i2c_nfound < 8) {
        g_i2c_addrs[g_i2c_nfound++] = addr7;
      }
    }
  }
}

static uint16_t as5600_read_raw(void)
{
  uint8_t buf[2] = {0, 0};
  g_enc_status = HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_ANGLE,
                                  I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
  if (g_enc_status != HAL_OK) {
    return 0;
  }
  return (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);  // 0..4095
}

extern "C" void user_setup(void)
{
  HAL_Delay(50);  // let encoder power up
  i2c_bus_scan();

  g_enc_found = 0;
  for (uint8_t i = 0; i < g_i2c_nfound; ++i) {
    if (g_i2c_addrs[i] == 0x36) {
      g_enc_found = 1;
      break;
    }
  }
}

extern "C" void user_loop(void)
{
  if (g_enc_found) {
    g_enc_raw = as5600_read_raw();
  }
  HAL_Delay(20);  // ~50 Hz — fine for a wiring check
}
