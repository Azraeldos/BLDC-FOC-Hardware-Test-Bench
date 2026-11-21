#pragma once

#include "main.h"

/** Init nFAULT GPIO and load flash calib. */
void control_init(void);

/** Enable TIM1 update IRQ path (call after PWM start). */
void control_start_irq(void);

/** Fixed-rate FOC / mode tick (~2 kHz). */
void control_tick(void);

/** Call from HAL_TIM_PeriodElapsedCallback for TIM1 (handles divider). */
void control_on_tim_update(TIM_HandleTypeDef *htim);

/** Slow-path flash save (call from user_loop). */
void control_poll_save(void);
