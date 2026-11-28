#include "user_main.h"
#include "as5600.h"
#include "control.h"
#include "motor_pwm.h"
#include "user_comm.h"

#include "main.h"

extern "C" void user_setup(void)
{
  control_init();
  user_comm_init();

  HAL_Delay(50);
  as5600_init();

  motor_pwm_start();
  motor_pwm_stop();
  control_start_irq();
}

extern "C" void user_loop(void)
{
  // Serial RX/TX outside the TIM1 ISR — never blocks the ~2 kHz control tick.
  user_comm_poll();
  control_poll_save();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  control_on_tim_update(htim);
}
