#include "pid.h"

float pid_step(PID* pid, float setpoint, float measurement) {
  float error = setpoint - measurement;

  // Proportional term
  float proportional = pid->kp * error;

  // Integrator term
  pid->integrator = pid->integrator + (0.5f * pid->ki * pid->T * (error + pid->prev_error));

  // Integrator anti-windup to prevent over-saturation
  float lim_min_int, lim_max_int;

  if (pid->lim_max > proportional) {
    lim_max_int = pid->lim_max - proportional;
  } else {
    lim_max_int = 0.0f;
  }

  if (pid->lim_min < proportional) {
    lim_min_int = pid->lim_min - proportional;
  } else {
    lim_min_int = 0.0f;
  }

  // Clamp integrator
  if (pid->integrator > lim_max_int) {
    pid->integrator = lim_max_int;
  } else if (pid->integrator < lim_min_int) {
    pid->integrator = lim_min_int;
  }

  pid->differentiator = - (2.0f * pid->kd * (measurement - pid->prev_measurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->lim_max) {
    pid->out = pid->lim_max;
  } else if (pid->out < pid->lim_min) {
    pid->out = pid->lim_min;
  }

  pid->prev_error = error;
  pid->prev_measurement = measurement;

  return pid->out;
}
