"""Real-time PID tuning GUI using PyQt5 + pyqtgraph.

Usage:
    python tuning_gui.py [--port /dev/ttyACM0] [--baud 115200]
    python tuning_gui.py --simulate   # Run with simulated data
"""
from __future__ import annotations
import sys
import argparse
import math
import time
import threading
from collections import deque
from dataclasses import dataclass

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np

from serial_client import PIDSerialClient, ControllerState


# ── Simulated controller for testing without hardware ──────────
class SimulatedController:
    """First-order motor model + PID for GUI testing."""

    def __init__(self):
        self.kp, self.ki, self.kd = 0.5, 0.1, 0.01
        self.setpoint = 0.0
        self.rpm = 0.0
        self.integral = 0.0
        self.prev_err = 0.0
        self.enabled = True
        self._running = False
        self._callback = None
        self._t0 = time.time()

        # Motor model: tau * d(rpm)/dt = K*duty - rpm
        self.tau = 0.15   # time constant (s)
        self.K = 3000.0   # gain (rpm per unit duty)
        self.dt = 0.01    # 10 ms

    def connect(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._running = False

    def on_state(self, cb):
        self._callback = cb

    def set_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def set_setpoint(self, rpm):
        self.setpoint = rpm

    def enable(self, on):
        self.enabled = on
        if not on:
            self.rpm = 0
            self.integral = 0

    def reset(self):
        self.rpm = 0
        self.integral = 0
        self.prev_err = 0
        self.setpoint = 0

    def _loop(self):
        while self._running:
            if self.enabled:
                err = self.setpoint - self.rpm
                self.integral += self.ki * err * self.dt
                self.integral = max(-0.8, min(0.8, self.integral))
                d_term = -self.kd * (self.rpm - self.prev_err) / self.dt if self.prev_err != 0 else 0
                duty = max(-1, min(1, self.kp * err + self.integral + d_term))
                self.prev_err = self.rpm

                # Simulate motor dynamics
                d_rpm = (self.K * duty - self.rpm) / self.tau * self.dt
                noise = np.random.normal(0, 5)
                self.rpm += d_rpm + noise
            else:
                duty = 0
                err = 0

            ms = int((time.time() - self._t0) * 1000)
            state = ControllerState(ms, self.setpoint, self.rpm, err, duty,
                                    self.kp, self.ki, self.kd)
            if self._callback:
                self._callback(state)

            time.sleep(self.dt)


# ── Main GUI ────────────────────────────────────────────
class PIDTuningWindow(QtWidgets.QMainWindow):
    state_received = QtCore.pyqtSignal(object)

    def __init__(self, client):
        super().__init__()
        self.client = client
        self.setWindowTitle("PID Motor Tuning")
        self.setMinimumSize(1000, 650)

        # Data buffers (10 seconds at 100 Hz)
        self.max_points = 1000
        self.time_buf = deque(maxlen=self.max_points)
        self.setpoint_buf = deque(maxlen=self.max_points)
        self.rpm_buf = deque(maxlen=self.max_points)
        self.error_buf = deque(maxlen=self.max_points)
        self.output_buf = deque(maxlen=self.max_points)

        self._build_ui()
        self.state_received.connect(self._on_state)
        self.client.on_state(lambda s: self.state_received.emit(s))

        # Plot refresh timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(50)  # 20 fps

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ── Left panel: controls ──
        ctrl_panel = QtWidgets.QVBoxLayout()
        ctrl_panel.setSpacing(12)

        # Gains
        gains_group = QtWidgets.QGroupBox("PID Gains")
        gains_layout = QtWidgets.QFormLayout()
        self.kp_spin = self._make_spin(0.0, 50.0, 0.5, 0.01)
        self.ki_spin = self._make_spin(0.0, 50.0, 0.1, 0.01)
        self.kd_spin = self._make_spin(0.0, 10.0, 0.01, 0.001)
        gains_layout.addRow("Kp:", self.kp_spin)
        gains_layout.addRow("Ki:", self.ki_spin)
        gains_layout.addRow("Kd:", self.kd_spin)
        apply_btn = QtWidgets.QPushButton("Apply Gains")
        apply_btn.clicked.connect(self._apply_gains)
        gains_layout.addRow(apply_btn)
        gains_group.setLayout(gains_layout)
        ctrl_panel.addWidget(gains_group)

        # Setpoint
        sp_group = QtWidgets.QGroupBox("Setpoint")
        sp_layout = QtWidgets.QFormLayout()
        self.sp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sp_slider.setRange(-3000, 3000)
        self.sp_slider.setValue(0)
        self.sp_label = QtWidgets.QLabel("0 RPM")
        self.sp_slider.valueChanged.connect(self._setpoint_changed)
        sp_layout.addRow("RPM:", self.sp_slider)
        sp_layout.addRow("", self.sp_label)

        # Step response buttons
        step_row = QtWidgets.QHBoxLayout()
        for rpm in [500, 1000, 1500, 2000]:
            btn = QtWidgets.QPushButton(f"{rpm}")
            btn.clicked.connect(lambda _, r=rpm: self._set_sp(r))
            step_row.addWidget(btn)
        sp_layout.addRow("Quick:", step_row)
        sp_group.setLayout(sp_layout)
        ctrl_panel.addWidget(sp_group)

        # Controls
        ctrl_group = QtWidgets.QGroupBox("Control")
        ctrl_layout = QtWidgets.QVBoxLayout()
        self.enable_btn = QtWidgets.QPushButton("Enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.toggled.connect(self._toggle_enable)
        ctrl_layout.addWidget(self.enable_btn)
        reset_btn = QtWidgets.QPushButton("Reset")
        reset_btn.clicked.connect(self._reset)
        ctrl_layout.addWidget(reset_btn)
        ctrl_group.setLayout(ctrl_layout)
        ctrl_panel.addWidget(ctrl_group)

        # Live metrics
        metrics_group = QtWidgets.QGroupBox("Live Metrics")
        metrics_layout = QtWidgets.QFormLayout()
        self.rpm_label = QtWidgets.QLabel("—")
        self.err_label = QtWidgets.QLabel("—")
        self.out_label = QtWidgets.QLabel("—")
        font = QtGui.QFont("Menlo", 14, QtGui.QFont.Bold)
        for lbl in [self.rpm_label, self.err_label, self.out_label]:
            lbl.setFont(font)
        metrics_layout.addRow("RPM:", self.rpm_label)
        metrics_layout.addRow("Error:", self.err_label)
        metrics_layout.addRow("Output:", self.out_label)
        metrics_group.setLayout(metrics_layout)
        ctrl_panel.addWidget(metrics_group)

        ctrl_panel.addStretch()
        layout.addLayout(ctrl_panel, 1)

        # ── Right panel: plots ──
        plot_panel = QtWidgets.QVBoxLayout()
        pg.setConfigOptions(antialias=True)

        self.rpm_plot = pg.PlotWidget(title="Speed (RPM)")
        self.rpm_plot.addLegend()
        self.rpm_plot.setLabel("bottom", "Time", "s")
        self.rpm_curve = self.rpm_plot.plot(pen=pg.mkPen("c", width=2), name="RPM")
        self.sp_curve = self.rpm_plot.plot(pen=pg.mkPen("r", width=1, style=QtCore.Qt.DashLine), name="Setpoint")
        plot_panel.addWidget(self.rpm_plot)

        self.error_plot = pg.PlotWidget(title="Error (RPM)")
        self.error_plot.setLabel("bottom", "Time", "s")
        self.error_curve = self.error_plot.plot(pen=pg.mkPen("y", width=2))
        plot_panel.addWidget(self.error_plot)

        self.output_plot = pg.PlotWidget(title="Control Output")
        self.output_plot.setLabel("bottom", "Time", "s")
        self.output_plot.setYRange(-1.1, 1.1)
        self.output_curve = self.output_plot.plot(pen=pg.mkPen("g", width=2))
        plot_panel.addWidget(self.output_plot)

        layout.addLayout(plot_panel, 3)

    def _make_spin(self, lo, hi, val, step):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setValue(val)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        return spin

    def _on_state(self, state: ControllerState):
        t = state.timestamp_ms / 1000.0
        self.time_buf.append(t)
        self.setpoint_buf.append(state.setpoint)
        self.rpm_buf.append(state.rpm)
        self.error_buf.append(state.error)
        self.output_buf.append(state.output)

        self.rpm_label.setText(f"{state.rpm:.1f}")
        self.err_label.setText(f"{state.error:.1f}")
        self.out_label.setText(f"{state.output:.3f}")

    def _update_plots(self):
        if len(self.time_buf) < 2:
            return
        t = np.array(self.time_buf)
        self.rpm_curve.setData(t, np.array(self.rpm_buf))
        self.sp_curve.setData(t, np.array(self.setpoint_buf))
        self.error_curve.setData(t, np.array(self.error_buf))
        self.output_curve.setData(t, np.array(self.output_buf))

    def _apply_gains(self):
        self.client.set_gains(self.kp_spin.value(), self.ki_spin.value(), self.kd_spin.value())

    def _setpoint_changed(self, val):
        self.sp_label.setText(f"{val} RPM")
        self.client.set_setpoint(float(val))

    def _set_sp(self, rpm):
        self.sp_slider.setValue(rpm)

    def _toggle_enable(self, checked):
        self.enable_btn.setText("Disable" if checked else "Enable")
        self.client.enable(checked)

    def _reset(self):
        self.client.reset()
        self.sp_slider.setValue(0)
        self.time_buf.clear()
        self.rpm_buf.clear()
        self.setpoint_buf.clear()
        self.error_buf.clear()
        self.output_buf.clear()

    def closeEvent(self, event):
        self.client.disconnect()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="PID Motor Tuning GUI")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--simulate", action="store_true", help="Use simulated controller")
    args = parser.parse_args()

    if args.simulate:
        client = SimulatedController()
    else:
        client = PIDSerialClient(args.port, args.baud)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = PIDTuningWindow(client)
    window.show()

    client.connect()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
