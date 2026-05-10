/**
 * @file main.c
 * @brief STM32F4 main loop — PID motor speed control with serial tuning.
 *
 * Hardware mapping (STM32F411 Nucleo):
 *   PWM output  → TIM2 CH1 (PA0)
 *   Direction   → PB0
 *   Encoder A   → TIM3 CH1 (PA6)
 *   Encoder B   → TIM3 CH2 (PA7)
 *   UART2 TX/RX → PA2/PA3 (ST-Link virtual COM)
 *
 * Control loop runs at 1 kHz via TIM6 interrupt.
 */

#include <string.h>
#include <stdio.h>
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "serial_protocol.h"

/* ── Platform stubs (replace with real HAL in STM32CubeIDE project) ── */
#ifdef SIMULATION_MODE
#include <stdlib.h>
#include <math.h>

static uint32_t sys_tick = 0;
uint32_t HAL_GetTick(void) { return sys_tick; }
void     HAL_UART_Transmit(void *huart, uint8_t *data, uint16_t len, uint32_t timeout) { (void)huart; (void)data; (void)len; (void)timeout; }
uint16_t HAL_UART_Receive_IT(void *huart, uint8_t *data, uint16_t len) { (void)huart; (void)data; (void)len; return 0; }

#else
/* Real HAL includes for STM32 target build */
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;   /* PWM timer */
extern TIM_HandleTypeDef htim3;   /* Encoder timer */
extern TIM_HandleTypeDef htim6;   /* Control loop timer (1 kHz) */
extern UART_HandleTypeDef huart2; /* Debug/tuning UART */
#endif

/* ── Global instances ────────────────────────────────── */
static PID_Controller  pid;
static Motor_Handle    motor;
static Encoder_Handle  encoder;
static Proto_Parser    rx_parser;

static float setpoint_rpm = 0.0f;
static volatile uint8_t rx_byte;
static volatile uint8_t control_flag = 0;

/* ── Control loop (called from TIM6 ISR at 1 kHz) ──── */
#define CONTROL_DT     0.001f   /* 1 ms */
#define REPORT_DIVIDER 10       /* Send telemetry every 10 ms */

static uint32_t report_counter = 0;

void control_loop(void)
{
    /* Read encoder speed */
    float rpm = Encoder_GetRPM(&encoder);

    /* PID update */
    float output = PID_Update(&pid, setpoint_rpm, rpm);

    /* Drive motor */
    Motor_SetDuty(&motor, output);

    /* Periodic telemetry */
    report_counter++;
    if (report_counter >= REPORT_DIVIDER) {
        report_counter = 0;

        /* Build STATE_REPORT frame */
        uint8_t payload[32];
        uint8_t idx = 0;
        uint32_t ms = HAL_GetTick();
        memcpy(&payload[idx], &ms,          4); idx += 4;
        memcpy(&payload[idx], &setpoint_rpm,4); idx += 4;
        memcpy(&payload[idx], &rpm,         4); idx += 4;
        float err = setpoint_rpm - rpm;
        memcpy(&payload[idx], &err,         4); idx += 4;
        memcpy(&payload[idx], &output,      4); idx += 4;
        memcpy(&payload[idx], &pid.Kp,      4); idx += 4;
        memcpy(&payload[idx], &pid.Ki,      4); idx += 4;
        memcpy(&payload[idx], &pid.Kd,      4); idx += 4;

        uint8_t frame[40];
        uint8_t flen = Proto_BuildFrame(frame, RESP_STATE_REPORT, payload, idx);

#ifndef SIMULATION_MODE
        HAL_UART_Transmit(&huart2, frame, flen, 5);
#else
        HAL_UART_Transmit(NULL, frame, flen, 5);
#endif
    }
}

/* ── Command handler ─────────────────────────────────── */
static void handle_command(Proto_Frame *frame)
{
    uint8_t ack_buf[8];
    uint8_t ack_len;

    switch (frame->cmd) {
    case CMD_SET_GAINS: {
        float kp, ki, kd;
        memcpy(&kp, &frame->payload[0], 4);
        memcpy(&ki, &frame->payload[4], 4);
        memcpy(&kd, &frame->payload[8], 4);
        PID_SetGains(&pid, kp, ki, kd);
        uint8_t ack_payload = CMD_SET_GAINS;
        ack_len = Proto_BuildFrame(ack_buf, RESP_ACK, &ack_payload, 1);
        break;
    }
    case CMD_SET_SETPOINT: {
        memcpy(&setpoint_rpm, &frame->payload[0], 4);
        uint8_t ack_payload = CMD_SET_SETPOINT;
        ack_len = Proto_BuildFrame(ack_buf, RESP_ACK, &ack_payload, 1);
        break;
    }
    case CMD_ENABLE: {
        PID_Enable(&pid, frame->payload[0] != 0);
        if (!frame->payload[0]) Motor_Stop(&motor);
        uint8_t ack_payload = CMD_ENABLE;
        ack_len = Proto_BuildFrame(ack_buf, RESP_ACK, &ack_payload, 1);
        break;
    }
    case CMD_RESET: {
        PID_Reset(&pid);
        Motor_Stop(&motor);
        setpoint_rpm = 0.0f;
        uint8_t ack_payload = CMD_RESET;
        ack_len = Proto_BuildFrame(ack_buf, RESP_ACK, &ack_payload, 1);
        break;
    }
    default: {
        uint8_t nack_payload[2] = { frame->cmd, 0x01 };
        ack_len = Proto_BuildFrame(ack_buf, RESP_NACK, nack_payload, 2);
        break;
    }
    }

#ifndef SIMULATION_MODE
    HAL_UART_Transmit(&huart2, ack_buf, ack_len, 5);
#else
    HAL_UART_Transmit(NULL, ack_buf, ack_len, 5);
#endif
}

/* ── UART receive callback ────────────────────────────── */
void UART_RxCallback(uint8_t byte)
{
    if (Proto_FeedByte(&rx_parser, byte)) {
        handle_command(&rx_parser.frame);
    }
}

/* ── Main ─────────────────────────────────────────────── */
int main(void)
{
#ifndef SIMULATION_MODE
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();   /* PWM */
    MX_TIM3_Init();   /* Encoder */
    MX_TIM6_Init();   /* Control loop timer */
    MX_USART2_UART_Init();
#endif

    /* Init subsystems */
    PID_Init(&pid, 0.5f, 0.1f, 0.01f, CONTROL_DT, -1.0f, 1.0f);

#ifndef SIMULATION_MODE
    Motor_Init(&motor, &htim2, 0 /* TIM_CHANNEL_1 */,
               GPIOB, 0x0001 /* GPIO_PIN_0 */,
               htim2.Instance->ARR);
    Encoder_Init(&encoder, &htim3, 2400 /* 600 PPR × 4 */, CONTROL_DT);
#else
    Motor_Init(&motor, NULL, 0, NULL, 0, 1000);
    Encoder_Init(&encoder, NULL, 2400, CONTROL_DT);
#endif

    Proto_InitParser(&rx_parser);

#ifndef SIMULATION_MODE
    /* Start encoder timer */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    /* Start PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    /* Start control loop timer interrupt */
    HAL_TIM_Base_Start_IT(&htim6);
    /* Start UART receive interrupt */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
#endif

    /* Superloop */
    while (1) {
#ifdef SIMULATION_MODE
        /* In simulation mode, manually tick the control loop */
        sys_tick++;
        control_loop();
#else
        /* Real target: control_loop() is called from TIM6 ISR.
           Main loop handles non-time-critical tasks. */
        __WFI();  /* Wait for interrupt — saves power */
#endif
    }

    return 0;
}

/* ── ISR hooks (for STM32 HAL) ───────────────────────── */
#ifndef SIMULATION_MODE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        control_loop();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        UART_RxCallback(rx_byte);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
    }
}
#endif
