/**
 * @file motor.c
 * @brief DC motor driver — PWM duty cycle and direction control via STM32 HAL.
 */
#include "motor.h"

/* STM32 HAL types — forward declarations to avoid HAL header dependency
   in the portable layer. Actual linking resolves these at build time. */
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
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef_Minimal;

typedef struct {
    TIM_TypeDef_Minimal *Instance;
} TIM_HandleTypeDef_Minimal;

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef_Minimal;

/* ── Helpers ─────────────────────────────────────────── */

static void set_ccr(TIM_HandleTypeDef_Minimal *htim, uint32_t channel, uint32_t val)
{
    switch (channel) {
        case 0: htim->Instance->CCR1 = val; break;
        case 1: htim->Instance->CCR2 = val; break;
        case 2: htim->Instance->CCR3 = val; break;
        case 3: htim->Instance->CCR4 = val; break;
    }
}

static void gpio_write(GPIO_TypeDef_Minimal *port, uint16_t pin, uint8_t state)
{
    if (state)
        port->BSRR = (uint32_t)pin;
    else
        port->BSRR = (uint32_t)pin << 16U;
}

/* ── Public API ──────────────────────────────────────── */

void Motor_Init(Motor_Handle *motor, void *htim, uint32_t channel,
                void *dir_port, uint16_t dir_pin, uint32_t timer_period)
{
    motor->htim = htim;
    motor->channel = channel;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->timer_period = timer_period;
    motor->duty_cycle = 0.0f;
    motor->direction = MOTOR_DIR_CW;

    /* Set initial state */
    set_ccr((TIM_HandleTypeDef_Minimal *)htim, channel, 0);
    gpio_write((GPIO_TypeDef_Minimal *)dir_port, dir_pin, 0);
}

void Motor_SetDuty(Motor_Handle *motor, float duty)
{
    /* Handle sign → direction */
    if (duty < 0.0f) {
        Motor_SetDirection(motor, MOTOR_DIR_CCW);
        duty = -duty;
    } else {
        Motor_SetDirection(motor, MOTOR_DIR_CW);
    }

    /* Clamp */
    if (duty > 1.0f) duty = 1.0f;
    motor->duty_cycle = duty;

    uint32_t ccr = (uint32_t)(duty * (float)motor->timer_period);
    set_ccr((TIM_HandleTypeDef_Minimal *)motor->htim, motor->channel, ccr);
}

void Motor_SetDirection(Motor_Handle *motor, Motor_Direction dir)
{
    motor->direction = dir;
    gpio_write((GPIO_TypeDef_Minimal *)motor->dir_port, motor->dir_pin,
               dir == MOTOR_DIR_CCW ? 1 : 0);
}

void Motor_Stop(Motor_Handle *motor)
{
    motor->duty_cycle = 0.0f;
    set_ccr((TIM_HandleTypeDef_Minimal *)motor->htim, motor->channel, 0);
}

float Motor_GetDuty(Motor_Handle *motor)
{
    return motor->duty_cycle;
}
