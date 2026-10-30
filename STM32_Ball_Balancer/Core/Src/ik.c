#include "ik.h"

#include <math.h>

static const float k_lp = 7.20f;
static const float k_l1 = 7.30f;
static const float k_l2 = 4.60f;
static const float k_lb = 4.00f;
static const float k_h = 9.50f;
static const float k_max_theta = 5.0f;
static const float k_sqrt3 = 1.73205080757f;
static const float k_deg = 0.01745329252f;
static const float k_to_deg = 57.2957795f;

static float held1;
static float held2;
static float held3;
static int held_valid;

static void keep_last(float *deg1, float *deg2, float *deg3)
{
  if (held_valid) {
    *deg1 = held1;
    *deg2 = held2;
    *deg3 = held3;
  }
}

int ik_solve(float theta_deg, float phi_deg, float *deg1, float *deg2, float *deg3)
{
  float theta;
  float phi;
  float a;
  float b;
  float c;
  float q1;
  float q3;
  float d1;
  float d2;
  float d3;
  float x1;
  float y1;
  float z1;
  float x2;
  float z2;
  float x3;
  float y3;
  float z3;
  float p1;
  float v1;
  float r1;
  float s1;
  float t1;
  float p2;
  float v2;
  float r2;
  float s2;
  float t2;
  float p3;
  float v3;
  float r3;
  float s3;
  float t3;
  float disc1;
  float disc2;
  float disc3;
  float c11;
  float c13;
  float c21;
  float c23;
  float c31;
  float c33;
  float under1;
  float under2;
  float under3;
  float out1;
  float out2;
  float out3;

  if (theta_deg < 0.0f) {
    theta_deg = 0.0f;
  }
  if (theta_deg > k_max_theta) {
    theta_deg = k_max_theta;
  }

  theta = theta_deg * k_deg;
  phi = phi_deg * k_deg;
  a = sinf(theta) * cosf(phi);
  b = sinf(theta) * sinf(phi);
  c = cosf(theta);

  q1 = a - k_sqrt3 * b;
  q3 = a + k_sqrt3 * b;
  d1 = sqrtf(4.0f * c * c + q1 * q1);
  d2 = sqrtf(c * c + a * a);
  d3 = sqrtf(4.0f * c * c + q3 * q3);
  if (d1 < 1.0e-4f || d2 < 1.0e-4f || d3 < 1.0e-4f) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  x1 = -(k_lp * c) / d1;
  y1 = (k_sqrt3 * k_lp * c) / d1;
  z1 = k_h + (k_lp * q1) / d1;
  x2 = (k_lp * c) / d2;
  z2 = k_h - (k_lp * a) / d2;
  x3 = -(k_lp * c) / d3;
  y3 = -(k_sqrt3 * k_lp * c) / d3;
  z3 = k_h + (k_lp * q3) / d3;

  if (fabsf(z1) < 1.0e-4f || fabsf(z2) < 1.0e-4f || fabsf(z3) < 1.0e-4f) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  p1 = (-x1 + k_sqrt3 * y1 - 2.0f * k_lb) / z1;
  v1 = (x1 * x1 + y1 * y1 + z1 * z1 + k_l2 * k_l2 - k_l1 * k_l1 - k_lb * k_lb) / (2.0f * z1);
  r1 = p1 * p1 + 4.0f;
  s1 = 2.0f * p1 * v1 + 4.0f * k_lb;
  t1 = v1 * v1 + k_lb * k_lb - k_l2 * k_l2;

  p2 = (k_lb - x2) / z2;
  v2 = (x2 * x2 + z2 * z2 - k_lb * k_lb + k_l2 * k_l2 - k_l1 * k_l1) / (2.0f * z2);
  r2 = p2 * p2 + 1.0f;
  s2 = 2.0f * (p2 * v2 - k_lb);
  t2 = v2 * v2 - k_l2 * k_l2 + k_lb * k_lb;

  p3 = (-x3 - k_sqrt3 * y3 - 2.0f * k_lb) / z3;
  v3 = (x3 * x3 + y3 * y3 + z3 * z3 + k_l2 * k_l2 - k_l1 * k_l1 - k_lb * k_lb) / (2.0f * z3);
  r3 = p3 * p3 + 4.0f;
  s3 = 2.0f * p3 * v3 + 4.0f * k_lb;
  t3 = v3 * v3 + k_lb * k_lb - k_l2 * k_l2;

  disc1 = s1 * s1 - 4.0f * r1 * t1;
  disc2 = s2 * s2 - 4.0f * r2 * t2;
  disc3 = s3 * s3 - 4.0f * r3 * t3;
  if (disc1 < 0.0f || disc2 < 0.0f || disc3 < 0.0f) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  c11 = (-s1 - sqrtf(disc1)) / (2.0f * r1);
  under1 = k_l2 * k_l2 - 4.0f * c11 * c11 - 4.0f * k_lb * c11 - k_lb * k_lb;
  c21 = (-s2 + sqrtf(disc2)) / (2.0f * r2);
  under2 = k_l2 * k_l2 - (c21 - k_lb) * (c21 - k_lb);
  c31 = (-s3 - sqrtf(disc3)) / (2.0f * r3);
  under3 = k_l2 * k_l2 - 4.0f * c31 * c31 - 4.0f * k_lb * c31 - k_lb * k_lb;
  if (under1 < 0.0f || under2 < 0.0f || under3 < 0.0f) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  c13 = sqrtf(under1);
  c23 = sqrtf(under2);
  c33 = sqrtf(under3);

  /* Same degree conversion as controller.Goto_N_time_spherical, before the offset. */
  out1 = atan2f(sqrtf(c11 * c11 + 3.0f * c11 * c11) - k_lb, c13) * k_to_deg;
  out2 = (1.57079632679f - atan2f(c23, c21 - k_lb)) * k_to_deg;
  out3 = atan2f(sqrtf(c31 * c31 + 3.0f * c31 * c31) - k_lb, c33) * k_to_deg;

  held1 = out1;
  held2 = out2;
  held3 = out3;
  held_valid = 1;
  *deg1 = out1;
  *deg2 = out2;
  *deg3 = out3;
  return 1;
}
