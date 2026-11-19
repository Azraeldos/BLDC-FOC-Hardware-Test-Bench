#include "user_main.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"

#include <cmath>

// AS5600 default 7-bit address. HAL wants it left-shifted.
static constexpr uint16_t AS5600_ADDR = (0x36 << 1);
static constexpr uint8_t  AS5600_REG_ANGLE = 0x0E;  // raw angle, 12-bit

// Watch these in the debugger Live Expressions / Watch window
volatile uint8_t  g_enc_found = 0;
volatile uint16_t g_enc_raw   = 0;
volatile HAL_StatusTypeDef g_enc_status = HAL_ERROR;
volatile uint8_t  g_i2c_addrs[8] = {0};
volatile uint8_t  g_i2c_nfound = 0;

// Open-loop motor — edit live in Watch (start with g_motor_en = 0)
volatile uint8_t g_motor_en = 1;     // 0 = off; 1 = spin
volatile float   g_amp      = 0.08f; // 0.05..0.20 first
volatile float   g_omega    = 30.0f; // rad/s electrical

static float g_theta = 0.0f;

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

static float clamp01(float x)
{
  if (x < 0.f) return 0.f;
  if (x > 1.f) return 1.f;
  return x;
}

static void motor_set_duties(float u, float v, float w)
{
  const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  // 3599
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(clamp01(u) * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(clamp01(v) * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(clamp01(w) * (float)arr));
}

static void motor_stop(void)
{
  motor_set_duties(0.f, 0.f, 0.f);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
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

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);

  motor_stop();  // EN low until g_motor_en = 1 in Watch
}

extern "C" void user_loop(void)
{
  static constexpr float dt     = 0.002f;  // 2 ms
  static constexpr float TWO_PI = 6.2831853f;
  static constexpr float DEG120 = 2.0943951f;

  if (g_enc_found) {
    g_enc_raw = as5600_read_raw();
  }

  if (!g_motor_en) {
    motor_stop();
    HAL_Delay(2);
    return;
  }

  g_theta += g_omega * dt;
  if (g_theta > TWO_PI) g_theta -= TWO_PI;
  if (g_theta < 0.f)    g_theta += TWO_PI;

  const float amp = g_amp;
  const float u = 0.5f + amp * cosf(g_theta);
  const float v = 0.5f + amp * cosf(g_theta - DEG120);
  const float w = 0.5f + amp * cosf(g_theta + DEG120);

  motor_set_duties(u, v, w);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

  HAL_Delay(2);
}
