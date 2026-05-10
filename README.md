# 🎛️ PID Motor Control

Closed-loop DC motor speed controller on STM32F4 with real-time Python tuning GUI. Implements PID control with anti-windup and derivative-on-measurement, communicating over a binary serial protocol for live gain adjustment and telemetry.

## Features

- **PID Controller** — Anti-windup (clamping + back-calculation), derivative-on-measurement to avoid setpoint kick, runtime gain tuning
- **Motor Driver** — PWM duty cycle control with bidirectional support via H-bridge direction pin
- **Quadrature Encoder** — Hardware timer encoder mode for accurate speed measurement at 2400 counts/rev (600 PPR × 4x)
- **Binary Serial Protocol** — CRC-8 protected frames for reliable command/telemetry exchange at 115200 baud
- **Real-Time Tuning GUI** — PyQt5 + pyqtgraph dashboard with live RPM/error/output plots, slider-based setpoint, and one-click gain updates
- **Simulation Mode** — Test firmware logic on x86 (no hardware needed) and run GUI with built-in motor model

## Quick Start

### GUI (with simulated motor)
```bash
cd gui
pip install -r requirements.txt
python tuning_gui.py --simulate
```

### GUI (with real hardware)
```bash
python tuning_gui.py --port /dev/ttyACM0
```

### Firmware (simulation build)
```bash
cd firmware
make sim
./build/sim/pid_sim
```

### Firmware (ARM cross-compilation)
```bash
cd firmware
make arm    # Requires arm-none-eabi-gcc
```

## Project Structure

```
pid-motor-control/
├── firmware/
│   ├── Inc/
│   │   ├── pid.h               # PID controller interface
│   │   ├── motor.h             # Motor driver interface
│   │   ├── encoder.h           # Quadrature encoder interface
│   │   └── serial_protocol.h   # Binary protocol definitions
│   ├── Src/
│   │   ├── main.c              # Control loop, UART handling, init
│   │   ├── pid.c               # PID with anti-windup
│   │   ├── motor.c             # PWM + direction control
│   │   ├── encoder.c           # Timer encoder mode reading
│   │   └── serial_protocol.c   # Frame parser + builder + CRC-8
│   └── Makefile                # sim/arm build targets
├── gui/
│   ├── tuning_gui.py           # PyQt5 real-time dashboard
│   ├── serial_client.py        # Protocol client + simulated controller
│   └── requirements.txt
├── tests/
│   └── test_pid.py             # PID unit tests + closed-loop validation
├── docs/
│   └── hardware_setup.md       # Wiring, pinout, CubeMX config
└── README.md
```

## Performance

| Metric | Value |
|--------|-------|
| Control loop rate | 1 kHz |
| Step response (0→1500 RPM) | < 500 ms settling |
| Steady-state error | < 1% |
| Overshoot (tuned) | < 10% |
| Telemetry rate | 100 Hz |

## Protocol

Binary frame format: `[0xAA] [CMD] [LEN] [PAYLOAD...] [CRC8]`

| Command | ID | Payload |
|---------|----|---------|
| SET_GAINS | 0x01 | Kp(f32) Ki(f32) Kd(f32) |
| SET_SETPOINT | 0x02 | RPM(f32) |
| ENABLE | 0x04 | on/off(u8) |
| STATE_REPORT | 0x81 | time(u32) setpoint(f32) rpm(f32) error(f32) output(f32) Kp Ki Kd |

## License

MIT
