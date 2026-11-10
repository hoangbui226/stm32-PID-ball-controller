#include "pid.h"

#include "stm32l1xx_hal.h"

#include <math.h>

static volatile float kp = 0.020f;
static volatile float ki = 0.00000f;
static volatile float kd = 0.000f;
static volatile float alpha = 0.8f;
static volatile float beta = 0.45f;

static float prev_out_x;
static float prev_out_y;
static float prev_err_x;
static float prev_err_y;
static float sum_err_x;
static float sum_err_y;
static uint32_t last_ms;
static int have_time;

void pid(float err_x, float err_y, float *theta_deg, float *phi_deg)
{
  uint32_t current_ms;
  float dt;
  float d_err_x;
  float d_err_y;
  float pid_x;
  float pid_y;
  float filtered_x;
  float filtered_y;
  float phi;
  float radius;

  // compute dt
  current_ms = HAL_GetTick();
  if (!have_time || current_ms == last_ms) {
    dt = 0.001f;
    have_time = 1;
  } else {
    dt = (float)(current_ms - last_ms) / 1000.0f;
  }
  last_ms = current_ms;

  //compute integral (I)
  sum_err_x += err_x * dt;
  sum_err_y += err_y * dt;

  //compute derivative (D)
  if (dt > 0.0f) {
    d_err_x = (err_x - prev_err_x) / dt;
    d_err_y = (err_y - prev_err_y) / dt;
  } else {
    d_err_x = 0.0f;
    d_err_y = 0.0f;
  }

  pid_x = kp * err_x + ki * sum_err_x + kd * d_err_x;
  pid_y = kp * err_y + ki * sum_err_y + kd * d_err_y;

  // low pass filter (LPF)
  filtered_x = alpha * pid_x + (1.0f - alpha) * prev_out_x;
  filtered_y = alpha * pid_y + (1.0f - alpha) * prev_out_y;

  phi = atan2f(filtered_y, filtered_x) * 57.2957795f; // which way to tilt
  if (phi < 0.0f) {
    phi += 360.0f;
  }
  radius = sqrtf(filtered_x * filtered_x + filtered_y * filtered_y); 
  *theta_deg = 17.0f * tanhf(beta * radius); // how much to tilt
  *phi_deg = phi;

  prev_err_x = err_x;
  prev_err_y = err_y;
  prev_out_x = filtered_x;
  prev_out_y = filtered_y;
}
