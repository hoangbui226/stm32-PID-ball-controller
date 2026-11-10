#include "ik.h"

#include <math.h>

static const float top_radius = 7.20f;
static const float upper_arm = 7.30f;
static const float lower_arm = 4.60f;
static const float bottom_radius = 4.00f;
static const float height = 9.50f;
static const float max_theta = 16.0f;
static const float sqrt3 = 1.73205080757f; // 
static const float deg_to_rad = 0.01745329252f;
static const float rad_to_deg = 57.2957795f;

static float prev_deg1;
static float prev_deg2;
static float prev_deg3;
static int held;

// keep the last valid solution
static void keep_last(float *deg1, float *deg2, float *deg3)
{
  if (held) {
    *deg1 = prev_deg1;
    *deg2 = prev_deg2;
    *deg3 = prev_deg3;
  }
}

// fix tip amount to (0, max_theta)
static float clamp_theta(float theta_deg)
{
  if (theta_deg < 0.0f) {
    return 0.0f;
  }
  if (theta_deg > max_theta) {
    return max_theta;
  }
  return theta_deg;
}

// plate normal vector (a, b, c) from theta and phi
static void plate_normal(float theta_deg, float phi_deg,
                         float *a, float *b, float *c)
{
  float theta = theta_deg * deg_to_rad;
  float phi = phi_deg * deg_to_rad;
  *a = sinf(theta) * cosf(phi);
  *b = sinf(theta) * sinf(phi);
  *c = cosf(theta);
}

// arm scales q1 and q3 and scale lengths d1, d2, d3
static int arm_scales(float a, float b, float c,
                      float *q1, float *q3,
                      float *d1, float *d2, float *d3)
{
  *q1 = a - sqrt3 * b;
  *q3 = a + sqrt3 * b;
  *d1 = sqrtf(4.0f * c * c + (*q1) * (*q1));
  *d2 = sqrtf(c * c + a * a);
  *d3 = sqrtf(4.0f * c * c + (*q3) * (*q3));
  if (*d1 < 1.0e-4f || *d2 < 1.0e-4f || *d3 < 1.0e-4f) {
    return 0;
  }
  return 1;
}

/* Place the three top (plate) joints. Returns 0 if any z is near zero. */
static int top_joints(float a, float c, float q1, float q3,
                      float d1, float d2, float d3,
                      float *x1, float *y1, float *z1,
                      float *x2, float *z2,
                      float *x3, float *y3, float *z3)
{
  *x1 = -(top_radius * c) / d1;
  *y1 = (sqrt3 * top_radius * c) / d1;
  *z1 = height + (top_radius * q1) / d1;
  *x2 = (top_radius * c) / d2;
  *z2 = height - (top_radius * a) / d2;
  *x3 = -(top_radius * c) / d3;
  *y3 = -(sqrt3 * top_radius * c) / d3;
  *z3 = height + (top_radius * q3) / d3;

  if (fabsf(*z1) < 1.0e-4f || fabsf(*z2) < 1.0e-4f || fabsf(*z3) < 1.0e-4f) {
    return 0;
  }
  return 1;
}

/* Build quadratic r u^2 + s u + t = 0 for one arm's 2-link solve. */
static void quad_arm1(float x1, float y1, float z1,
                      float *r, float *s, float *t)
{
  float p = (-x1 + sqrt3 * y1 - 2.0f * bottom_radius) / z1;
  float v = (x1 * x1 + y1 * y1 + z1 * z1 + lower_arm * lower_arm
             - upper_arm * upper_arm - bottom_radius * bottom_radius)
            / (2.0f * z1);
  *r = p * p + 4.0f;
  *s = 2.0f * p * v + 4.0f * bottom_radius;
  *t = v * v + bottom_radius * bottom_radius - lower_arm * lower_arm;
}

static void quad_arm2(float x2, float z2, float *r, float *s, float *t)
{
  float p = (bottom_radius - x2) / z2;
  float v = (x2 * x2 + z2 * z2 - bottom_radius * bottom_radius
             + lower_arm * lower_arm - upper_arm * upper_arm)
            / (2.0f * z2);
  *r = p * p + 1.0f;
  *s = 2.0f * (p * v - bottom_radius);
  *t = v * v - lower_arm * lower_arm + bottom_radius * bottom_radius;
}

static void quad_arm3(float x3, float y3, float z3,
                      float *r, float *s, float *t)
{
  float p = (-x3 - sqrt3 * y3 - 2.0f * bottom_radius) / z3;
  float v = (x3 * x3 + y3 * y3 + z3 * z3 + lower_arm * lower_arm
             - upper_arm * upper_arm - bottom_radius * bottom_radius)
            / (2.0f * z3);
  *r = p * p + 4.0f;
  *s = 2.0f * p * v + 4.0f * bottom_radius;
  *t = v * v + bottom_radius * bottom_radius - lower_arm * lower_arm;
}

/* Solve quads -> link coords, then horn angles in degrees.
   Returns 0 if unreachable (negative disc or under-sqrt). */
static int solve_servo_angles(float r1, float s1, float t1,
                              float r2, float s2, float t2,
                              float r3, float s3, float t3,
                              float *out1, float *out2, float *out3)
{
  float disc1 = s1 * s1 - 4.0f * r1 * t1;
  float disc2 = s2 * s2 - 4.0f * r2 * t2;
  float disc3 = s3 * s3 - 4.0f * r3 * t3;
  float c11;
  float c13;
  float c21;
  float c23;
  float c31;
  float c33;
  float under1;
  float under2;
  float under3;

  if (disc1 < 0.0f || disc2 < 0.0f || disc3 < 0.0f) {
    return 0;
  }

  c11 = (-s1 - sqrtf(disc1)) / (2.0f * r1);
  under1 = lower_arm * lower_arm - 4.0f * c11 * c11
           - 4.0f * bottom_radius * c11 - bottom_radius * bottom_radius;
  c21 = (-s2 + sqrtf(disc2)) / (2.0f * r2);
  under2 = lower_arm * lower_arm - (c21 - bottom_radius) * (c21 - bottom_radius);
  c31 = (-s3 - sqrtf(disc3)) / (2.0f * r3);
  under3 = lower_arm * lower_arm - 4.0f * c31 * c31
           - 4.0f * bottom_radius * c31 - bottom_radius * bottom_radius;
  if (under1 < 0.0f || under2 < 0.0f || under3 < 0.0f) {
    return 0;
  }

  c13 = sqrtf(under1);
  c23 = sqrtf(under2);
  c33 = sqrtf(under3);

  *out1 = atan2f(sqrtf(c11 * c11 + 3.0f * c11 * c11) - bottom_radius, c13)
          * rad_to_deg;
  *out2 = (1.57079632679f - atan2f(c23, c21 - bottom_radius)) * rad_to_deg;
  *out3 = atan2f(sqrtf(c31 * c31 + 3.0f * c31 * c31) - bottom_radius, c33)
          * rad_to_deg;
  return 1;
}

/* ---------- public entry ---------- */

int ik_solve(float theta_deg, float phi_deg, float *deg1, float *deg2, float *deg3)
{
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
  float r1;
  float s1;
  float t1;
  float r2;
  float s2;
  float t2;
  float r3;
  float s3;
  float t3;
  float out1;
  float out2;
  float out3;

  theta_deg = clamp_theta(theta_deg);

  plate_normal(theta_deg, phi_deg, &a, &b, &c);

  if (!arm_scales(a, b, c, &q1, &q3, &d1, &d2, &d3)) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  if (!top_joints(a, c, q1, q3, d1, d2, d3,
                  &x1, &y1, &z1, &x2, &z2, &x3, &y3, &z3)) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  quad_arm1(x1, y1, z1, &r1, &s1, &t1);
  quad_arm2(x2, z2, &r2, &s2, &t2);
  quad_arm3(x3, y3, z3, &r3, &s3, &t3);

  if (!solve_servo_angles(r1, s1, t1, r2, s2, t2, r3, s3, t3,
                          &out1, &out2, &out3)) {
    keep_last(deg1, deg2, deg3);
    return 0;
  }

  prev_deg1 = out1;
  prev_deg2 = out2;
  prev_deg3 = out3;
  held = 1;
  *deg1 = out1;
  *deg2 = out2;
  *deg3 = out3;
  return 1;
}
