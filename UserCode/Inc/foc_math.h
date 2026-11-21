#pragma once

constexpr float FOC_PI      = 3.14159265359f;
constexpr float FOC_TWO_PI  = 6.28318530718f;
constexpr float FOC_DEG120  = 2.09439510239f;
constexpr float FOC_SQRT3_2 = 0.86602540378f;

inline float foc_wrap_pm_pi(float x)
{
  while (x >  FOC_PI) x -= FOC_TWO_PI;
  while (x < -FOC_PI) x += FOC_TWO_PI;
  return x;
}

inline float foc_wrap_0_2pi(float x)
{
  while (x >= FOC_TWO_PI) x -= FOC_TWO_PI;
  while (x < 0.f)         x += FOC_TWO_PI;
  return x;
}

inline float foc_clamp(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

inline float foc_clamp01(float x)
{
  return foc_clamp(x, 0.f, 1.f);
}

/** Inverse Park + Clarke → phase voltages in ~[-1, 1]. */
void foc_dq_to_abc(float vd, float vq, float theta_e,
                   float *va, float *vb, float *vc);
