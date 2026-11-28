#pragma once

#include "main.h"

#include <cstdint>

enum Mode : uint8_t {
  MODE_OPEN_LOOP = 0,
  MODE_TORQUE    = 1,
  MODE_SPEED     = 2,
  MODE_POSITION  = 3,
  MODE_HAPTIC    = 4,
};

/** Haptic profile ids (serial P line / g_haptic_type). */
enum HapticType : uint8_t {
  HAPTIC_DETENTS = 1,
  HAPTIC_RANGE   = 2,
  HAPTIC_SPRING  = 3,
  HAPTIC_DAMPER  = 4,
};

// Watch / Live Expressions — defined in user_params.cpp
extern volatile uint8_t  g_enc_found;
extern volatile uint16_t g_enc_raw;
extern volatile HAL_StatusTypeDef g_enc_status;
extern volatile uint8_t  g_i2c_addrs[8];
extern volatile uint8_t  g_i2c_nfound;

extern volatile uint8_t g_motor_en;
extern volatile uint8_t g_mode;

extern volatile float g_amp;
extern volatile float g_omega;
extern volatile float g_vd;
extern volatile float g_vq;

extern volatile uint8_t g_pole_pairs;
extern volatile float   g_zero_offset;
extern volatile uint8_t g_align_req;
extern volatile uint8_t g_align_busy;
extern volatile uint8_t g_align_done;
extern volatile float   g_align_amp;

extern volatile float g_kp_speed;
extern volatile float g_ki_speed;

extern volatile float g_pos_ref;
extern volatile float g_omega_max;
extern volatile float g_kp_pos;
extern volatile float g_ki_pos;

extern volatile uint8_t g_save_req;
extern volatile uint8_t g_save_ok;
extern volatile uint8_t g_calib_loaded;

extern volatile uint8_t g_fault_en;
extern volatile uint8_t g_fault;
extern volatile uint8_t g_fault_clr;

/** Haptic profile parameters (mode 4). Capture width is local-only (not on P line). */
extern volatile uint8_t g_haptic_type;
extern volatile uint8_t g_haptic_detents;
extern volatile float   g_haptic_kp;
extern volatile float   g_haptic_kd;
extern volatile float   g_haptic_min_deg;
extern volatile float   g_haptic_max_deg;
extern volatile float   g_haptic_center_deg;
extern volatile float   g_haptic_capture_deg;  // <=0 → auto (half sector)

extern volatile float g_theta_e;
extern volatile float g_mech_angle;
extern volatile float g_mech_unwrapped;
extern volatile float g_speed_e;
extern volatile float g_vq_out;
extern volatile float g_omega_cmd;

/** Haptic / serial telemetry (updated in control tick). */
extern volatile float   g_angle_deg;
extern volatile int16_t g_detent_index;
extern volatile float   g_vel_deg_s;
