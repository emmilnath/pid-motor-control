/**
 * @file pid.c
 * @brief PID controller implementation with anti-windup and derivative-on-measurement.
 */
#include "pid.h"
#include <math.h>

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float dt, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = (out_max - out_min) * 0.8f;  /* 80% of range */
    PID_Reset(pid);
    pid->enabled = true;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->first_run = true;
}

static float clamp(float val, float lo, float hi)
{
    if (val > hi) return hi;
    if (val < lo) return lo;
    return val;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement)
{
    if (!pid->enabled) {
        pid->output = 0.0f;
        return 0.0f;
    }

    float error = setpoint - measurement;

    /* ── Proportional ──────────────────────────────── */
    float p_term = pid->Kp * error;

    /* ── Integral with clamping anti-windup ─────────── */
    pid->integral += pid->Ki * error * pid->dt;
    pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    float i_term = pid->integral;

    /* ── Derivative on measurement (avoids setpoint kick) ── */
    float d_term = 0.0f;
    if (!pid->first_run) {
        float d_measurement = (measurement - pid->prev_measurement) / pid->dt;
        d_term = -pid->Kd * d_measurement;
    }
    pid->first_run = false;

    /* ── Sum and clamp output ──────────────────────── */
    float output = p_term + i_term + d_term;
    output = clamp(output, pid->out_min, pid->out_max);

    /* ── Back-calculation anti-windup ─────────────── */
    if (output == pid->out_max || output == pid->out_min) {
        /* Only accumulate integral if it would reduce saturation */
        if ((error > 0 && output == pid->out_max) ||
            (error < 0 && output == pid->out_min)) {
            pid->integral -= pid->Ki * error * pid->dt;
        }
    }

    /* ── Store state ──────────────────────────────── */
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    pid->output = output;

    return output;
}

void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_Enable(PID_Controller *pid, bool enable)
{
    if (enable && !pid->enabled) {
        PID_Reset(pid);
    }
    pid->enabled = enable;
}
