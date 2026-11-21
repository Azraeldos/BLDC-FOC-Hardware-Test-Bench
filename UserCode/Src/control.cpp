#include "control.h"
#include "as5600.h"
#include "foc_math.h"
#include "motor_pwm.h"
#include "user_flash.h"
#include "user_params.h"
#include "tim.h"

// TIM1 center-aligned @ 25 kHz → update IRQ ~50 kHz; divide to ~2 kHz
static constexpr uint32_t CTRL_DIV = 25;
static constexpr float    CTRL_DT  = 1.0f / (50000.0f / static_cast<float>(CTRL_DIV));

static constexpr uint32_t ALIGN_HOLD_TICKS   = 2000;
static constexpr uint32_t ALIGN_SAMPLE_TICKS = 200;

enum AlignPhase : uint8_t { ALIGN_IDLE, ALIGN_HOLD, ALIGN_SAMPLE, ALIGN_FINISH };

static float g_theta_ol = 0.0f;
static float g_speed_i  = 0.0f;
static float g_pos_i    = 0.0f;
static float g_mech_prev = 0.0f;
static bool  g_mech_prev_valid = false;

static uint32_t g_ctrl_div_cnt = 0;
static volatile uint8_t g_save_pending = 0;

static AlignPhase g_align_phase = ALIGN_IDLE;
static uint32_t   g_align_ticks = 0;
static float      g_align_sum   = 0.0f;
static uint32_t   g_align_n     = 0;

static void nft_gpio_init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef cfg = {};
  cfg.Pin   = NFT_Pin;
  cfg.Mode  = GPIO_MODE_INPUT;
  cfg.Pull  = GPIO_PULLUP;
  cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NFT_GPIO_Port, &cfg);
}

static uint8_t nft_is_asserted(void)
{
  return (HAL_GPIO_ReadPin(NFT_GPIO_Port, NFT_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static void update_angle_speed(float mech)
{
  if (!g_mech_prev_valid) {
    g_mech_prev = mech;
    g_mech_unwrapped = mech;
    g_mech_prev_valid = true;
    g_speed_e = 0.f;
    return;
  }

  const float dmech = foc_wrap_pm_pi(mech - g_mech_prev);
  g_mech_prev = mech;
  g_mech_unwrapped += dmech;

  const float omega_m = dmech / CTRL_DT;
  const float omega_e = omega_m * static_cast<float>(g_pole_pairs);
  g_speed_e = 0.90f * g_speed_e + 0.10f * omega_e;
}

static float speed_pi(float omega_ref, float amp)
{
  const float err = omega_ref - g_speed_e;
  g_speed_i += err * CTRL_DT;
  float vq = g_kp_speed * err + g_ki_speed * g_speed_i;
  if (vq > amp) {
    vq = amp;
    g_speed_i -= err * CTRL_DT;
  } else if (vq < -amp) {
    vq = -amp;
    g_speed_i -= err * CTRL_DT;
  }
  return vq;
}

static float position_pi(float amp_omega)
{
  const float err = g_pos_ref - g_mech_unwrapped;
  g_pos_i += err * CTRL_DT;
  float w = g_kp_pos * err + g_ki_pos * g_pos_i;
  if (w > amp_omega) {
    w = amp_omega;
    g_pos_i -= err * CTRL_DT;
  } else if (w < -amp_omega) {
    w = -amp_omega;
    g_pos_i -= err * CTRL_DT;
  }
  return w * static_cast<float>(g_pole_pairs);
}

static void align_start(void)
{
  g_align_busy  = 1;
  g_align_done  = 0;
  g_align_phase = ALIGN_HOLD;
  g_align_ticks = 0;
  g_align_sum   = 0.f;
  g_align_n     = 0;
  g_speed_i     = 0.f;
  g_pos_i       = 0.f;
  g_mech_prev_valid = false;
}

static void align_tick(float mech)
{
  switch (g_align_phase) {
    case ALIGN_HOLD:
      motor_pwm_set_dq(g_align_amp, 0.f, 0.f);
      if (++g_align_ticks >= ALIGN_HOLD_TICKS) {
        g_align_ticks = 0;
        g_align_sum   = 0.f;
        g_align_n     = 0;
        g_align_phase = ALIGN_SAMPLE;
      }
      break;

    case ALIGN_SAMPLE:
      motor_pwm_set_dq(g_align_amp, 0.f, 0.f);
      g_align_sum += mech;
      g_align_n++;
      if (++g_align_ticks >= ALIGN_SAMPLE_TICKS) {
        g_align_phase = ALIGN_FINISH;
      }
      break;

    case ALIGN_FINISH:
      if (g_align_n > 0) {
        g_zero_offset = g_align_sum / static_cast<float>(g_align_n);
      }
      motor_pwm_stop();
      g_align_done  = 1;
      g_align_busy  = 0;
      g_align_req   = 0;
      g_align_phase = ALIGN_IDLE;
      g_motor_en    = 0;
      break;

    default:
      g_align_phase = ALIGN_IDLE;
      g_align_busy  = 0;
      break;
  }
}

static void fault_service(void)
{
  if (g_fault_clr) {
    g_fault_clr = 0;
    if (!g_fault_en || !nft_is_asserted()) {
      g_fault = 0;
    }
  }

  if (g_fault_en && nft_is_asserted()) {
    g_fault = 1;
  }
}

static void reset_integrators(void)
{
  g_vq_out = 0.f;
  g_omega_cmd = 0.f;
  g_speed_i = 0.f;
  g_pos_i = 0.f;
}

static void load_calib_from_flash(void)
{
  UserCalib cal = {};
  if (user_flash_load(&cal)) {
    g_pole_pairs   = static_cast<uint8_t>(cal.pole_pairs);
    g_zero_offset  = cal.zero_offset;
    g_calib_loaded = 1;
    g_align_done   = 1;
  } else {
    g_calib_loaded = 0;
  }
}

void control_tick(void)
{
  fault_service();

  if (g_enc_found) {
    g_enc_raw = as5600_read_raw();
  }
  const float mech = as5600_raw_to_mech(g_enc_raw);
  g_mech_angle = mech;
  g_theta_e    = as5600_mech_to_elec(mech, g_zero_offset, g_pole_pairs);
  update_angle_speed(mech);

  if (g_align_req && g_align_phase == ALIGN_IDLE) {
    align_start();
  }
  if (g_align_busy) {
    align_tick(mech);
    return;
  }

  if (g_fault) {
    motor_pwm_stop();
    g_motor_en = 0;
    reset_integrators();
    return;
  }

  if (!g_motor_en) {
    motor_pwm_stop();
    reset_integrators();
    return;
  }

  const float amp = foc_clamp(g_amp, 0.f, 0.95f);

  switch (g_mode) {
    case MODE_OPEN_LOOP: {
      g_theta_ol = foc_wrap_0_2pi(g_theta_ol + g_omega * CTRL_DT);
      motor_pwm_open_loop(g_theta_ol, amp);
      g_vq_out = 0.f;
      g_omega_cmd = g_omega;
      break;
    }

    case MODE_TORQUE: {
      const float vd = foc_clamp(g_vd, -amp, amp);
      const float vq = foc_clamp(g_vq, -amp, amp);
      g_vq_out = vq;
      g_omega_cmd = 0.f;
      motor_pwm_set_dq(vd, vq, g_theta_e);
      break;
    }

    case MODE_SPEED: {
      g_omega_cmd = g_omega;
      const float vq = speed_pi(g_omega_cmd, amp);
      const float vd = foc_clamp(g_vd, -amp, amp);
      g_vq_out = vq;
      motor_pwm_set_dq(vd, vq, g_theta_e);
      break;
    }

    case MODE_POSITION: {
      const float wmax = foc_clamp(g_omega_max, 0.f, 500.f);
      g_omega_cmd = position_pi(wmax);
      const float vq = speed_pi(g_omega_cmd, amp);
      const float vd = foc_clamp(g_vd, -amp, amp);
      g_vq_out = vq;
      motor_pwm_set_dq(vd, vq, g_theta_e);
      break;
    }

    default:
      motor_pwm_stop();
      break;
  }
}

void control_on_tim_update(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) {
    return;
  }
  if (++g_ctrl_div_cnt < CTRL_DIV) {
    return;
  }
  g_ctrl_div_cnt = 0;
  control_tick();
}

void control_init(void)
{
  nft_gpio_init();
  load_calib_from_flash();
}

void control_start_irq(void)
{
  g_ctrl_div_cnt = 0;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void control_poll_save(void)
{
  if (g_save_req) {
    g_save_req = 0;
    g_save_pending = 1;
  }

  if (!g_save_pending) {
    return;
  }
  g_save_pending = 0;

  const uint8_t was_en = g_motor_en;
  g_motor_en = 0;
  motor_pwm_stop();
  HAL_Delay(2);

  UserCalib cal = {};
  cal.pole_pairs  = g_pole_pairs;
  cal.zero_offset = g_zero_offset;

  __disable_irq();
  g_save_ok = user_flash_save(&cal);
  __enable_irq();

  if (g_save_ok) {
    g_calib_loaded = 1;
  }

  g_motor_en = was_en;
}
