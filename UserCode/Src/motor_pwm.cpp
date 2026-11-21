#include "motor_pwm.h"
#include "foc_math.h"
#include "main.h"
#include "tim.h"

#include <cmath>

void motor_pwm_start(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

void motor_pwm_set_duties(float u, float v, float w)
{
  const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(foc_clamp01(u) * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(foc_clamp01(v) * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(foc_clamp01(w) * (float)arr));
}

void motor_pwm_stop(void)
{
  motor_pwm_set_duties(0.f, 0.f, 0.f);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
}

void motor_pwm_enable(void)
{
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
}

void motor_pwm_set_dq(float vd, float vq, float theta_e)
{
  float va = 0.f, vb = 0.f, vc = 0.f;
  foc_dq_to_abc(vd, vq, theta_e, &va, &vb, &vc);
  motor_pwm_set_duties(0.5f + 0.5f * va, 0.5f + 0.5f * vb, 0.5f + 0.5f * vc);
  motor_pwm_enable();
}

void motor_pwm_open_loop(float theta, float amp)
{
  const float u = 0.5f + amp * cosf(theta);
  const float v = 0.5f + amp * cosf(theta - FOC_DEG120);
  const float w = 0.5f + amp * cosf(theta + FOC_DEG120);
  motor_pwm_set_duties(u, v, w);
  motor_pwm_enable();
}
