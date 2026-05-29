"""
PID Calibration tab — signal generator, real-time plots, gain tuning.
Used as a mixin for MainWindow.
"""

import math
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QComboBox, QSpinBox, QLabel,
)

from helpers import make_label, make_double_spin
from steadywin_can import ControlMode, INPUT_MODE_NAMES


class PIDCalibrationMixin:
    """Mixin providing the PID Calibration tab and its slot methods."""

    def _init_pid_state(self):
        """Call from MainWindow.__init__ to set up PID state."""
        self._pid_running = False
        self._pid_start_time = 0.0
        self._pid_signal_timer = QTimer()
        self._pid_signal_timer.timeout.connect(self._pid_signal_tick)
        self._pid_history_time = deque(maxlen=2000)
        self._pid_history_target = deque(maxlen=2000)
        self._pid_history_actual = deque(maxlen=2000)
        self._pid_history_error = deque(maxlen=2000)
        self._pid_history_vel_target = deque(maxlen=2000)
        self._pid_history_vel_actual = deque(maxlen=2000)
        self.bridge.pid_data_signal.connect(self._on_pid_data)

    def _build_pid_calibration_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(4)

        # ── Top: Settings row ──
        settings_grp = QGroupBox("Signal Generator & PID Settings")
        settings_lay = QGridLayout(settings_grp)

        # Node ID for PID
        settings_lay.addWidget(make_label("Node ID:", bold=True), 0, 0)
        self.pid_spin_node = QSpinBox()
        self.pid_spin_node.setRange(0, 63)
        self.pid_spin_node.setValue(0)
        settings_lay.addWidget(self.pid_spin_node, 0, 1)

        # Control type
        settings_lay.addWidget(make_label("Control:"), 0, 2)
        self.pid_cmb_control = QComboBox()
        self.pid_cmb_control.addItem("Position", "position")
        self.pid_cmb_control.addItem("Velocity", "velocity")
        settings_lay.addWidget(self.pid_cmb_control, 0, 3)

        # Signal type
        settings_lay.addWidget(make_label("Signal:"), 0, 4)
        self.pid_cmb_signal = QComboBox()
        self.pid_cmb_signal.addItems(["Sine", "Square", "Triangle", "Step", "Sawtooth"])
        settings_lay.addWidget(self.pid_cmb_signal, 0, 5)

        # Amplitude
        settings_lay.addWidget(make_label("Amplitude:"), 0, 6)
        self.pid_spin_amplitude = make_double_spin(0.01, 100, 3, 1.0, 0.1)
        self.pid_spin_amplitude.setToolTip("rev for Position, rev/s for Velocity")
        settings_lay.addWidget(self.pid_spin_amplitude, 0, 7)

        # Frequency
        settings_lay.addWidget(make_label("Freq (Hz):"), 0, 8)
        self.pid_spin_freq = make_double_spin(0.01, 20.0, 3, 0.5, 0.1)
        settings_lay.addWidget(self.pid_spin_freq, 0, 9)

        # Offset
        settings_lay.addWidget(make_label("Offset:"), 0, 10)
        self.pid_spin_offset = make_double_spin(-100, 100, 3, 0.0, 0.1)
        settings_lay.addWidget(self.pid_spin_offset, 0, 11)

        # Duration
        settings_lay.addWidget(make_label("Duration (s):"), 1, 0)
        self.pid_spin_duration = make_double_spin(1, 300, 1, 10.0, 1.0)
        settings_lay.addWidget(self.pid_spin_duration, 1, 1)

        # Send rate
        settings_lay.addWidget(make_label("Rate (ms):"), 1, 2)
        self.pid_spin_rate = QSpinBox()
        self.pid_spin_rate.setRange(10, 500)
        self.pid_spin_rate.setValue(20)
        self.pid_spin_rate.setToolTip("Signal send interval in ms")
        settings_lay.addWidget(self.pid_spin_rate, 1, 3)

        # PID Gains — inline quick tuning
        settings_lay.addWidget(make_label("Pos Kp:"), 1, 4)
        self.pid_spin_pos_kp = make_double_spin(0, 500, 2, 20.0, 1.0)
        settings_lay.addWidget(self.pid_spin_pos_kp, 1, 5)

        settings_lay.addWidget(make_label("Vel Kp:"), 1, 6)
        self.pid_spin_vel_kp = make_double_spin(0, 100, 4, 0.16, 0.01)
        settings_lay.addWidget(self.pid_spin_vel_kp, 1, 7)

        settings_lay.addWidget(make_label("Vel Ki:"), 1, 8)
        self.pid_spin_vel_ki = make_double_spin(0, 100, 4, 0.32, 0.01)
        settings_lay.addWidget(self.pid_spin_vel_ki, 1, 9)

        btn_apply_gains = QPushButton("Apply Gains")
        btn_apply_gains.setStyleSheet("background: #1a5276; padding: 6px;")
        btn_apply_gains.clicked.connect(self._pid_apply_gains)
        settings_lay.addWidget(btn_apply_gains, 1, 10)

        # Input mode for PID test
        settings_lay.addWidget(make_label("Input Mode:"), 2, 0)
        self.pid_cmb_input_mode = QComboBox()
        for val, name in sorted(INPUT_MODE_NAMES.items()):
            self.pid_cmb_input_mode.addItem(f"{val}: {name}", val)
        self.pid_cmb_input_mode.setCurrentIndex(1)
        settings_lay.addWidget(self.pid_cmb_input_mode, 2, 1)

        # Buttons
        self.pid_btn_start = QPushButton("▶  Start Test")
        self.pid_btn_start.setStyleSheet(
            "background: #1e8449; font-weight: bold; padding: 8px; font-size: 13px;")
        self.pid_btn_start.clicked.connect(self._pid_start)
        settings_lay.addWidget(self.pid_btn_start, 2, 4, 1, 2)

        self.pid_btn_stop = QPushButton("⬛  Stop")
        self.pid_btn_stop.setStyleSheet(
            "background: #922b21; font-weight: bold; padding: 8px; font-size: 13px;")
        self.pid_btn_stop.setEnabled(False)
        self.pid_btn_stop.clicked.connect(self._pid_stop)
        settings_lay.addWidget(self.pid_btn_stop, 2, 6, 1, 2)

        btn_clear_plot = QPushButton("Clear Plots")
        btn_clear_plot.clicked.connect(self._pid_clear_plots)
        settings_lay.addWidget(btn_clear_plot, 2, 8)

        # Time window
        settings_lay.addWidget(make_label("View (s):"), 2, 9)
        self.pid_spin_view_window = make_double_spin(2, 300, 1, 10.0, 1.0)
        self.pid_spin_view_window.setToolTip("Time window to show in plots")
        settings_lay.addWidget(self.pid_spin_view_window, 2, 10)

        layout.addWidget(settings_grp)

        # ── Graphs ──
        self.pid_pos_plot = pg.PlotWidget(title="Position: Target vs Estimated")
        self.pid_pos_plot.setLabel('left', 'Position', 'rev')
        self.pid_pos_plot.setLabel('bottom', 'Time', 's')
        self.pid_pos_plot.addLegend(offset=(10, 10))
        self.pid_pos_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pid_pos_curve_target = self.pid_pos_plot.plot(
            pen=pg.mkPen('#ff6600', width=2), name='Target')
        self.pid_pos_curve_actual = self.pid_pos_plot.plot(
            pen=pg.mkPen('#00ccff', width=2), name='Estimated')

        self.pid_err_plot = pg.PlotWidget(title="Tracking Error")
        self.pid_err_plot.setLabel('left', 'Error', 'rev')
        self.pid_err_plot.setLabel('bottom', 'Time', 's')
        self.pid_err_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pid_err_curve = self.pid_err_plot.plot(
            pen=pg.mkPen('#ff3333', width=2), name='Error')

        self.pid_vel_plot = pg.PlotWidget(title="Velocity: Target vs Estimated")
        self.pid_vel_plot.setLabel('left', 'Velocity', 'rev/s')
        self.pid_vel_plot.setLabel('bottom', 'Time', 's')
        self.pid_vel_plot.addLegend(offset=(10, 10))
        self.pid_vel_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pid_vel_curve_target = self.pid_vel_plot.plot(
            pen=pg.mkPen('#ffcc00', width=2), name='Vel Target')
        self.pid_vel_curve_actual = self.pid_vel_plot.plot(
            pen=pg.mkPen('#66ff66', width=2), name='Vel Estimated')

        self.pid_stats_label = QLabel(
            "Stats: —  |  RMS Error: —  |  Max Error: —  |  Overshoot: —")
        self.pid_stats_label.setStyleSheet(
            "font-size: 12px; padding: 4px; background: #252525; border: 1px solid #444;")

        layout.addWidget(self.pid_pos_plot, 3)
        layout.addWidget(self.pid_err_plot, 2)
        layout.addWidget(self.pid_vel_plot, 2)
        layout.addWidget(self.pid_stats_label)

        return widget

    # ── Signal Generation ────────────────────────────────────────────────

    def _generate_signal(self, t: float) -> float:
        sig_type = self.pid_cmb_signal.currentText()
        amp = self.pid_spin_amplitude.value()
        freq = self.pid_spin_freq.value()
        offset = self.pid_spin_offset.value()
        period = 1.0 / freq if freq > 0 else 1.0
        phase = (t % period) / period

        if sig_type == "Sine":
            val = amp * math.sin(2 * math.pi * freq * t)
        elif sig_type == "Square":
            val = amp if phase < 0.5 else -amp
        elif sig_type == "Triangle":
            if phase < 0.25:
                val = amp * (phase / 0.25)
            elif phase < 0.75:
                val = amp * (1.0 - (phase - 0.25) / 0.25)
            else:
                val = amp * (-1.0 + (phase - 0.75) / 0.25)
        elif sig_type == "Step":
            val = amp if t > 0.5 else 0.0
        elif sig_type == "Sawtooth":
            val = amp * (2 * phase - 1)
        else:
            val = 0.0

        return val + offset

    def _pid_start(self):
        if not self._check_connected():
            return

        nid = self.pid_spin_node.value()
        self.bus.register_motor(nid)

        ctrl = self.pid_cmb_control.currentData()
        input_mode = self.pid_cmb_input_mode.currentData()
        if ctrl == "position":
            self.bus.set_controller_mode(nid, ControlMode.POSITION, input_mode)
        else:
            self.bus.set_controller_mode(nid, ControlMode.VELOCITY, input_mode)

        self._pid_clear_plots()

        self._pid_running = True
        self._pid_start_time = time.time()
        rate_ms = self.pid_spin_rate.value()
        self._pid_signal_timer.start(rate_ms)

        self.pid_btn_start.setEnabled(False)
        self.pid_btn_stop.setEnabled(True)
        self.statusBar().showMessage("PID Test running...")

    def _pid_stop(self):
        self._pid_running = False
        self._pid_signal_timer.stop()

        nid = self.pid_spin_node.value()
        ctrl = self.pid_cmb_control.currentData()
        if self.bus.is_connected:
            if ctrl == "position":
                self.bus.set_input_pos(nid, self.pid_spin_offset.value())
            else:
                self.bus.set_input_vel(nid, 0.0)

        self.pid_btn_start.setEnabled(True)
        self.pid_btn_stop.setEnabled(False)
        self.statusBar().showMessage("PID Test stopped")
        self._pid_update_stats()

    def _pid_signal_tick(self):
        if not self._pid_running or not self.bus.is_connected:
            self._pid_stop()
            return

        now = time.time()
        t = now - self._pid_start_time
        duration = self.pid_spin_duration.value()

        if t >= duration:
            self._pid_stop()
            return

        nid = self.pid_spin_node.value()
        target = self._generate_signal(t)
        ctrl = self.pid_cmb_control.currentData()

        if ctrl == "position":
            self.bus.set_input_pos(nid, target)
        else:
            self.bus.set_input_vel(nid, target)

        self.bus.request_encoder_estimates(nid)

        fb = self.bus.get_feedback(nid)
        if fb:
            if ctrl == "position":
                estimated = fb.pos_estimate
                vel_estimated = fb.vel_estimate
                dt = 0.001
                vel_target = (self._generate_signal(t + dt) - target) / dt
            else:
                estimated = fb.vel_estimate
                vel_estimated = fb.vel_estimate
                vel_target = target

            self.bridge.pid_data_signal.emit(t, target, estimated)

            self._pid_history_time.append(t)
            self._pid_history_target.append(target)
            self._pid_history_actual.append(estimated)
            self._pid_history_error.append(target - estimated)
            self._pid_history_vel_target.append(vel_target)
            self._pid_history_vel_actual.append(vel_estimated)

            self._pid_update_plots()

    def _on_pid_data(self, t: float, target: float, estimated: float):
        pass

    def _pid_update_plots(self):
        if len(self._pid_history_time) < 2:
            return

        t_arr = np.array(self._pid_history_time)
        target_arr = np.array(self._pid_history_target)
        actual_arr = np.array(self._pid_history_actual)
        error_arr = np.array(self._pid_history_error)
        vel_t_arr = np.array(self._pid_history_vel_target)
        vel_a_arr = np.array(self._pid_history_vel_actual)

        view_window = self.pid_spin_view_window.value()
        t_max = t_arr[-1]
        if t_max > view_window:
            mask = t_arr >= (t_max - view_window)
            t_arr = t_arr[mask]
            target_arr = target_arr[mask]
            actual_arr = actual_arr[mask]
            error_arr = error_arr[mask]
            vel_t_arr = vel_t_arr[mask]
            vel_a_arr = vel_a_arr[mask]

        self.pid_pos_curve_target.setData(t_arr, target_arr)
        self.pid_pos_curve_actual.setData(t_arr, actual_arr)
        self.pid_err_curve.setData(t_arr, error_arr)
        self.pid_vel_curve_target.setData(t_arr, vel_t_arr)
        self.pid_vel_curve_actual.setData(t_arr, vel_a_arr)

        self._pid_update_stats()

    def _pid_update_stats(self):
        if len(self._pid_history_error) < 2:
            return
        error_arr = np.array(self._pid_history_error)
        rms = np.sqrt(np.mean(error_arr ** 2))
        max_err = np.max(np.abs(error_arr))
        mean_err = np.mean(np.abs(error_arr))

        target_arr = np.array(self._pid_history_target)
        actual_arr = np.array(self._pid_history_actual)
        if np.max(np.abs(target_arr)) > 1e-6:
            overshoot_pct = (np.max(np.abs(actual_arr)) - np.max(np.abs(target_arr))) / \
                            np.max(np.abs(target_arr)) * 100
            overshoot_pct = max(0, overshoot_pct)
        else:
            overshoot_pct = 0.0

        n = len(self._pid_history_time)
        self.pid_stats_label.setText(
            f"Samples: {n}  |  "
            f"RMS Error: {rms:.4f}  |  "
            f"Mean |Error|: {mean_err:.4f}  |  "
            f"Max |Error|: {max_err:.4f}  |  "
            f"Overshoot: {overshoot_pct:.1f}%"
        )

    def _pid_clear_plots(self):
        self._pid_history_time.clear()
        self._pid_history_target.clear()
        self._pid_history_actual.clear()
        self._pid_history_error.clear()
        self._pid_history_vel_target.clear()
        self._pid_history_vel_actual.clear()
        self.pid_pos_curve_target.setData([], [])
        self.pid_pos_curve_actual.setData([], [])
        self.pid_err_curve.setData([], [])
        self.pid_vel_curve_target.setData([], [])
        self.pid_vel_curve_actual.setData([], [])
        self.pid_stats_label.setText(
            "Stats: —  |  RMS Error: —  |  Max Error: —  |  Overshoot: —")

    def _pid_apply_gains(self):
        if not self._check_connected():
            return
        nid = self.pid_spin_node.value()
        self.bus.set_pos_gain(nid, self.pid_spin_pos_kp.value())
        time.sleep(0.005)
        self.bus.set_vel_gains(
            nid, self.pid_spin_vel_kp.value(), self.pid_spin_vel_ki.value())
        self.statusBar().showMessage(f"Gains applied to node {nid}")
