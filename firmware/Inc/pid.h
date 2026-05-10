/**
 * @file pid.h
 * @brief PID controller with anti-windup and derivative-on-measurement.
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Output limits */
    float out_min;
    float out_max;

    /* Integral windup limit */
    float integral_limit;

    /* Sample time in seconds */
    float dt;

    /* Internal state */
    float integral;
    float prev_measurement;
    float prev_error;
    float output;

    /* Flags */
    bool  first_run;
    bool  enabled;
} PID_Controller;

/**
 * Initialise PID controller with gains and limits.
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float dt, float out_min, float out_max);

/**
 * Reset integral accumulator and internal state.
 */
void PID_Reset(PID_Controller *pid);

/**
 * Compute one PID update.
 * @param pid        Controller instance
 * @param setpoint   Desired value
 * @param measurement Current measured value
 * @return Controller output (clamped to [out_min, out_max])
 */
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

/**
 * Update gains at runtime (e.g. from serial tuning GUI).
 */
void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd);

/**
 * Enable or disable the controller. When disabled, output is 0.
 */
void PID_Enable(PID_Controller *pid, bool enable);

#endif /* PID_H */
