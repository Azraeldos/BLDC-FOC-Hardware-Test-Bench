#include "foc_math.h"

#include <cmath>

void foc_dq_to_abc(float vd, float vq, float theta_e,
                   float *va, float *vb, float *vc)
{
  const float c = cosf(theta_e);
  const float s = sinf(theta_e);

  const float valpha = vd * c - vq * s;
  const float vbeta  = vd * s + vq * c;

  *va = valpha;
  *vb = -0.5f * valpha + FOC_SQRT3_2 * vbeta;
  *vc = -0.5f * valpha - FOC_SQRT3_2 * vbeta;
}
