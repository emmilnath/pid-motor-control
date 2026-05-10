# Hardware Setup Guide

## Components

| Component | Specification |
|-----------|--------------|
| MCU | STM32F411RE Nucleo-64 |
| Motor | 12V DC brushed motor with encoder |
| Motor driver | L298N dual H-bridge |
| Encoder | 600 PPR incremental quadrature |
| Power supply | 12V 3A for motor, USB for MCU |

## Wiring

### Motor Driver (L298N)

| L298N Pin | Connection |
|-----------|-----------|
| IN1 | PB0 (direction) |
| IN2 | GND (via inverter or tied low) |
| ENA | PA0 (TIM2 CH1 PWM) |
| OUT1/OUT2 | Motor terminals |
| +12V | External 12V supply |
| GND | Common ground with Nucleo |

### Encoder

| Encoder Wire | STM32 Pin | Timer |
|-------------|-----------|-------|
| Channel A | PA6 | TIM3 CH1 |
| Channel B | PA7 | TIM3 CH2 |
| VCC | 3.3V | — |
| GND | GND | — |

### UART (Tuning GUI)

The Nucleo board's ST-Link provides a virtual COM port on PA2 (TX) / PA3 (RX) via USART2. No additional wiring needed — just connect USB.

## STM32CubeMX Configuration

1. **TIM2** — PWM Generation CH1, ARR=999 (1 kHz PWM at 1 MHz timer clock)
2. **TIM3** — Encoder Mode, both edges, ARR=0xFFFF
3. **TIM6** — Basic timer, 1 kHz interrupt (control loop)
4. **USART2** — 115200 baud, 8N1, interrupt mode
5. **PB0** — GPIO Output (motor direction)

## Flashing

```bash
# Using ST-Link (OpenOCD)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/arm/pid_firmware.elf verify reset exit"

# Or via STM32CubeProgrammer GUI
```
