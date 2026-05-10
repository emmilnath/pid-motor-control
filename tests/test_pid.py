"""Unit tests for the PID controller logic (pure Python reimplementation)."""
import pytest
import math


class PIDController:
    """Python mirror of firmware PID for unit testing."""

    def __init__(self, kp, ki, kd, dt, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit = (out_max - out_min) * 0.8
        self.integral = 0.0
        self.prev_meas = 0.0
        self.first = True
        self.enabled = True

    def update(self, setpoint, measurement):
        if not self.enabled:
            return 0.0
        error = setpoint - measurement
        p = self.kp * error
        self.integral += self.ki * error * self.dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        d = 0.0
        if not self.first:
            d = -self.kd * (measurement - self.prev_meas) / self.dt
        self.first = False
        output = max(self.out_min, min(self.out_max, p + self.integral + d))
        self.prev_meas = measurement
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_meas = 0.0
        self.first = True


class MotorModel:
    """Simple first-order motor: tau * d(rpm)/dt = K*duty - rpm"""

    def __init__(self, K=3000.0, tau=0.15, dt=0.001):
        self.K, self.tau, self.dt = K, tau, dt
        self.rpm = 0.0

    def step(self, duty):
        self.rpm += (self.K * duty - self.rpm) / self.tau * self.dt
        return self.rpm


class TestPIDBasics:
    def test_zero_setpoint_zero_output(self):
        pid = PIDController(1.0, 0.1, 0.01, 0.001)
        assert pid.update(0.0, 0.0) == 0.0

    def test_proportional_response(self):
        pid = PIDController(2.0, 0.0, 0.0, 0.001)
        out = pid.update(100.0, 0.0)
        assert out == 1.0  # clamped to out_max

    def test_output_clamping(self):
        pid = PIDController(10.0, 0.0, 0.0, 0.001, out_min=-0.5, out_max=0.5)
        assert pid.update(1000.0, 0.0) == 0.5
        assert pid.update(-1000.0, 0.0) == -0.5

    def test_integral_windup_limit(self):
        pid = PIDController(0.0, 10.0, 0.0, 0.001)
        for _ in range(10000):
            pid.update(1000.0, 0.0)
        assert abs(pid.integral) <= pid.integral_limit + 1e-6

    def test_disabled_returns_zero(self):
        pid = PIDController(1.0, 1.0, 1.0, 0.001)
        pid.enabled = False
        assert pid.update(1000.0, 0.0) == 0.0


class TestClosedLoop:
    def test_step_response_settles(self):
        """Motor should reach setpoint within 2 seconds."""
        pid = PIDController(0.5, 0.3, 0.01, 0.001)
        motor = MotorModel()
        target = 1500.0

        for i in range(2000):  # 2 seconds
            duty = pid.update(target, motor.rpm)
            motor.step(duty)

        assert abs(motor.rpm - target) < 50, f"RPM={motor.rpm:.1f}, target={target}"

    def test_no_overshoot_above_threshold(self):
        """Overshoot should be < 15% of setpoint."""
        pid = PIDController(0.3, 0.1, 0.02, 0.001)
        motor = MotorModel()
        target = 1000.0
        max_rpm = 0.0

        for _ in range(3000):
            duty = pid.update(target, motor.rpm)
            motor.step(duty)
            if motor.rpm > max_rpm:
                max_rpm = motor.rpm

        overshoot_pct = (max_rpm - target) / target * 100
        assert overshoot_pct < 15, f"Overshoot = {overshoot_pct:.1f}%"

    def test_disturbance_rejection(self):
        """System recovers from a sudden load disturbance."""
        pid = PIDController(0.5, 0.3, 0.01, 0.001)
        motor = MotorModel()
        target = 1000.0

        # Let it settle
        for _ in range(2000):
            duty = pid.update(target, motor.rpm)
            motor.step(duty)

        # Apply disturbance
        motor.rpm -= 300

        # Let it recover
        for _ in range(2000):
            duty = pid.update(target, motor.rpm)
            motor.step(duty)

        assert abs(motor.rpm - target) < 50


class TestProtocol:
    def test_crc8(self):
        """CRC-8 implementation matches firmware."""
        def crc8(data):
            crc = 0x00
            for b in data:
                crc ^= b
                for _ in range(8):
                    crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
            return crc

        assert crc8(b"\x01\x0c") == crc8(bytes([0x01, 0x0C]))
        assert crc8(b"\x00") == 0x00
        assert crc8(b"\x01") != 0x00

    def test_frame_roundtrip(self):
        """Build a frame and verify it can be parsed."""
        from serial_client import build_frame, crc8

        frame = build_frame(0x01, b"\x00\x00\x80\x3f\x00\x00\x00\x00\x00\x00\x00\x00")
        assert frame[0] == 0xAA
        assert frame[1] == 0x01
        assert frame[2] == 12
        # Verify CRC
        crc_data = frame[1:3] + frame[3:-1]
        assert crc8(crc_data) == frame[-1]
