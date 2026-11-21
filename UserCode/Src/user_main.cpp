#include "user_main.h"
#include "as5600.h"
#include "control.h"
#include "motor_pwm.h"

#include "main.h"

extern "C" void user_setup(void)
{
  control_init();

  HAL_Delay(50);
  as5600_init();

  motor_pwm_start();
  motor_pwm_stop();
  control_start_irq();
}

extern "C" void user_loop(void)
{
  control_poll_save();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  control_on_tim_update(htim);
}
