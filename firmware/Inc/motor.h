/**
 * @file motor.h
 * @brief DC motor driver interface — PWM + direction via STM32 HAL.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MOTOR_DIR_CW  = 0,
    MOTOR_DIR_CCW = 1
} Motor_Direction;

typedef struct {
    /* HAL timer handle & channel for PWM */
    void    *htim;          /* TIM_HandleTypeDef* */
    uint32_t channel;

    /* Direction GPIO */
    void    *dir_port;      /* GPIO_TypeDef* */
    uint16_t dir_pin;

    /* State */
    float    duty_cycle;    /* 0.0 – 1.0 */
    Motor_Direction direction;
    uint32_t timer_period;  /* ARR value */
} Motor_Handle;

/**
 * Initialise motor driver and start PWM output.
 */
void Motor_Init(Motor_Handle *motor, void *htim, uint32_t channel,
                void *dir_port, uint16_t dir_pin, uint32_t timer_period);

/**
 * Set duty cycle (0.0 to 1.0). Negative values reverse direction.
 */
void Motor_SetDuty(Motor_Handle *motor, float duty);

/**
 * Set direction explicitly.
 */
void Motor_SetDirection(Motor_Handle *motor, Motor_Direction dir);

/**
 * Emergency stop — set duty to 0 immediately.
 */
void Motor_Stop(Motor_Handle *motor);

/**
 * Read back current duty cycle setting.
 */
float Motor_GetDuty(Motor_Handle *motor);

#endif /* MOTOR_H */
