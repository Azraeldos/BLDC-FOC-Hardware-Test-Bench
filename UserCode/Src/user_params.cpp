#include "user_params.h"

volatile uint8_t  g_enc_found = 0;
volatile uint16_t g_enc_raw   = 0;
volatile HAL_StatusTypeDef g_enc_status = HAL_ERROR;
volatile uint8_t  g_i2c_addrs[8] = {0};
volatile uint8_t  g_i2c_nfound = 0;

volatile uint8_t g_motor_en = 1;
volatile uint8_t g_mode  = 4;

volatile float g_amp   = 0.08f;
volatile float g_omega = 30.0f;
volatile float g_vd    = 0.0f;
volatile float g_vq    = 0.05f;

volatile uint8_t g_pole_pairs  = 7;
volatile float   g_zero_offset = 0.0f;
volatile uint8_t g_align_req   = 0;
volatile uint8_t g_align_busy  = 0;
volatile uint8_t g_align_done  = 0;
volatile float   g_align_amp   = 0.10f;

volatile float g_kp_speed = 0.02f;
volatile float g_ki_speed = 0.50f;

volatile float g_pos_ref   = 0.0f;
volatile float g_omega_max = 80.0f;
volatile float g_kp_pos    = 20.0f;
volatile float g_ki_pos    = 0.0f;

volatile uint8_t g_save_req     = 0;
volatile uint8_t g_save_ok      = 0;
volatile uint8_t g_calib_loaded = 0;

volatile uint8_t g_fault_en  = 1;
volatile uint8_t g_fault     = 0;
volatile uint8_t g_fault_clr = 0;

volatile uint8_t g_haptic_type       = HAPTIC_SPRING;
volatile uint8_t g_haptic_detents    = 12;
volatile float   g_haptic_kp         = 0.02f;
volatile float   g_haptic_kd         = 0.0002f;
volatile float   g_haptic_min_deg    = -45.0f;
volatile float   g_haptic_max_deg    = 45.0f;
volatile float   g_haptic_center_deg = 0.0f;
volatile float   g_haptic_capture_deg = 0.0f;  // auto

volatile float g_theta_e        = 0.0f;
volatile float g_mech_angle     = 0.0f;
volatile float g_mech_unwrapped = 0.0f;
volatile float g_speed_e        = 0.0f;
volatile float g_vq_out         = 0.0f;
volatile float g_omega_cmd      = 0.0f;

volatile float   g_angle_deg    = 0.0f;
volatile int16_t g_detent_index = 0;
volatile float   g_vel_deg_s    = 0.0f;
