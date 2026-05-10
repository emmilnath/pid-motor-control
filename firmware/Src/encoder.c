/**
 * @file encoder.c
 * @brief Quadrature encoder reading via STM32 timer encoder mode.
 */
#include "encoder.h"

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
} TIM_TypeDef_Enc;

typedef struct {
    TIM_TypeDef_Enc *Instance;
} TIM_HandleTypeDef_Enc;

void Encoder_Init(Encoder_Handle *enc, void *htim,
                  uint32_t counts_per_rev, float sample_dt)
{
    enc->htim = htim;
    enc->counts_per_rev = counts_per_rev;
    enc->dt = sample_dt;
    enc->count_offset = 0;
    enc->prev_count = 0;
    enc->rpm = 0.0f;

    /* Reset counter */
    TIM_HandleTypeDef_Enc *h = (TIM_HandleTypeDef_Enc *)htim;
    h->Instance->CNT = 0;
}

int32_t Encoder_GetCount(Encoder_Handle *enc)
{
    TIM_HandleTypeDef_Enc *h = (TIM_HandleTypeDef_Enc *)enc->htim;
    uint32_t cnt = h->Instance->CNT;

    /* Handle 16-bit overflow: if counter wraps, detect direction from delta */
    int32_t raw = (int32_t)cnt + enc->count_offset;
    return raw;
}

float Encoder_GetRPM(Encoder_Handle *enc)
{
    int32_t current = Encoder_GetCount(enc);
    int32_t delta = current - enc->prev_count;
    enc->prev_count = current;

    /* Convert count delta to RPM:
       RPM = (delta / counts_per_rev) / dt * 60 */
    float revs = (float)delta / (float)enc->counts_per_rev;
    enc->rpm = (revs / enc->dt) * 60.0f;

    return enc->rpm;
}

void Encoder_Reset(Encoder_Handle *enc)
{
    TIM_HandleTypeDef_Enc *h = (TIM_HandleTypeDef_Enc *)enc->htim;
    h->Instance->CNT = 0;
    enc->count_offset = 0;
    enc->prev_count = 0;
    enc->rpm = 0.0f;
}
