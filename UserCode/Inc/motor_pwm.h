#pragma once

/** Start TIM1 PWM channels and main output enable. */
void motor_pwm_start(void);

void motor_pwm_set_duties(float u, float v, float w);
void motor_pwm_stop(void);
void motor_pwm_enable(void);

/** Voltage FOC: dq → PWM (enables driver). */
void motor_pwm_set_dq(float vd, float vq, float theta_e);

/** Open-loop 120° cosine set (enables driver). */
void motor_pwm_open_loop(float theta, float amp);
