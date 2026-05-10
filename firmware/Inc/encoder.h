/**
 * @file encoder.h
 * @brief Quadrature encoder interface using STM32 timer in encoder mode.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    void    *htim;              /* TIM_HandleTypeDef* */
    int32_t  count_offset;      /* Accumulated overflow offset */
    uint32_t counts_per_rev;    /* Encoder PPR × 4 (quadrature) */
    float    rpm;               /* Calculated RPM */
    int32_t  prev_count;
    float    dt;                /* Sample period (s) */
} Encoder_Handle;

/**
 * Initialise encoder timer in encoder mode.
 */
void Encoder_Init(Encoder_Handle *enc, void *htim,
                  uint32_t counts_per_rev, float sample_dt);

/**
 * Read raw counter value (with overflow tracking).
 */
int32_t Encoder_GetCount(Encoder_Handle *enc);

/**
 * Calculate RPM from count difference over sample period.
 * Call this once per control loop iteration.
 */
float Encoder_GetRPM(Encoder_Handle *enc);

/**
 * Reset encoder count to zero.
 */
void Encoder_Reset(Encoder_Handle *enc);

#endif /* ENCODER_H */
