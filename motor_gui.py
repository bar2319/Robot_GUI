#!/usr/bin/env python3
"""
SteadyWin GIM6010-8 Motor Control GUI
Robotic Dog - 8 actuators, 4 legs x 2 motors each
"""

import os
import sys
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QSpinBox, QLineEdit,
    QTextEdit, QTabWidget, QSplitter, QMessageBox, QFrame, QLabel,
)

# Ensure QApplication exists before importing modules that may instantiate
# QWidget/PlotWidget at import time.
_EARLY_APP = QApplication.instance()
if _EARLY_APP is None:
    _EARLY_APP = QApplication(sys.argv)

import pyqtgraph as pg

# Configure pyqtgraph for dark theme (must be before any PlotWidget creation)
pg.setConfigOptions(antialias=True, background='#1a1a1a', foreground='#d4d4d4')

from helpers import (
    SignalBridge, LEDIndicator, make_label, DEFAULT_NODE_IDS,
)
from steadywin_can import (
    SteadyWinBus, AxisState, AXIS_STATE_NAMES,
)

from tab_single_motor import SingleMotorMixin
from tab_pid import PIDCalibrationMixin
from tab_all_motors import AllMotorsMixin
from tab_locomotion import LocomotionMixin
from tab_one_leg import OneLegMixin


# ─── Main Window ──────────────────────────────────────────────────────────────

class MainWindow(QMainWindow,
                 SingleMotorMixin,
                 PIDCalibrationMixin,
                 AllMotorsMixin,
                 LocomotionMixin,
                 OneLegMixin):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SteadyWin GIM6010-8 — Robotic Dog Controller")
        self.setMinimumSize(1100, 750)

        self.bus = SteadyWinBus()
        self.bridge = SignalBridge()
        self.selected_node_id: int = 0
        self._can_monitoring_enabled = False
        self._leg_visualization_enabled = False

        # Register default motors
        for nid in DEFAULT_NODE_IDS:
            self.bus.register_motor(nid)

        # Connect signals
        self.bridge.log_signal.connect(self._append_log)
        if self._can_monitoring_enabled:
            self.bridge.heartbeat_signal.connect(self._on_heartbeat_ui)
            self.bridge.feedback_signal.connect(self._on_feedback_ui)
            self.bus.set_heartbeat_callback(
                lambda nid: self.bridge.heartbeat_signal.emit(nid))
            self.bus.set_feedback_callback(
                lambda nid, t: self.bridge.feedback_signal.emit(nid, t))
        else:
            self.bus.set_heartbeat_callback(None)
            self.bus.set_feedback_callback(None)

        # Robot geometry (from datasheet / Arduino reference)
        self._robot_L1 = 133.0  # upper link mm
        self._robot_L2 = 133.0  # lower link mm
        self._robot_GR = 8.0    # gear ratio
        self._motor_offsets = [0.0] * 8
        self._motor_directions = [+1, +1, -1, -1, +1, +1, -1, -1]

        # Mixin state initialization
        self._init_pid_state()
        self._init_locomotion_state()
        self._init_one_leg_state()
        self._ol_offset = [0.0, 0.0]

        self._build_ui()

        # Polling timer for requesting feedback
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._poll_feedback)
        self._poll_interval_ms = 200

        # Lightweight status refresh — always runs when connected, independent
        # of the heavy per-channel polling (_can_monitoring_enabled).
        self._status_poll_timer = QTimer()
        self._status_poll_timer.timeout.connect(self._refresh_all_motors_status)
        self._status_poll_count = 0  # used to sub-sample vbus requests (1 Hz)

        # Run log — filled by _append_log while _run_logging is True.
        self._run_log: list[str] = []
        self._run_logging: bool = False

        self._apply_dark_theme()

        # INA219 powerbank monitor — init after UI so labels exist.
        # Gracefully disabled if the sensor is absent or smbus unavailable.
        self._ina219 = None
        self._ina_timer = QTimer()
        self._ina_timer.timeout.connect(self._ina_tick)
        self._init_ina219()

    # ── UI Construction ──────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # Top: Connection bar
        main_layout.addWidget(self._build_connection_bar())

        # Power status row (powerbank via INA219 + motor LiPo via CAN vbus)
        main_layout.addWidget(self._build_power_bar())

        # Middle: splitter with tabs left, log right
        self.main_splitter = QSplitter(Qt.Horizontal)

        tabs = QTabWidget()
        tabs.addTab(self._build_single_motor_tab(), "Single Motor")
        tabs.addTab(self._build_pid_calibration_tab(), "PID Calibration")
        tabs.addTab(self._build_all_motors_tab(), "All Motors")
        tabs.addTab(self._build_locomotion_tab(), "Locomotion")
        tabs.addTab(self._build_one_leg_tab(), "One Leg")
        self.main_splitter.addWidget(tabs)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Courier", 9))
        self.log_box.setMaximumWidth(420)
        self.log_group = QGroupBox("CAN Log")
        log_layout = QVBoxLayout(self.log_group)
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self.log_box.clear)
        log_layout.addWidget(self.log_box)
        log_layout.addWidget(btn_clear)
        self.main_splitter.addWidget(self.log_group)

        self.main_splitter.setStretchFactor(0, 3)
        self.main_splitter.setStretchFactor(1, 1)
        # Keep CAN log hidden by default to reduce UI and signal overhead.
        self.log_group.setVisible(False)
        self.main_splitter.setSizes([1, 0])
        self._set_can_log_enabled(False)
        main_layout.addWidget(self.main_splitter, 1)

        # Status bar
        self.statusBar().showMessage("Disconnected")

    # ── Connection Bar ───────────────────────────────────────────────────

    def _build_connection_bar(self) -> QGroupBox:
        grp = QGroupBox("CAN Connection")
        lay = QHBoxLayout(grp)

        lay.addWidget(make_label("Interface:"))
        self.cmb_interface = QComboBox()
        self.cmb_interface.addItems(["slcan", "socketcan", "gs_usb", "virtual"])
        self.cmb_interface.setCurrentText("slcan")
        lay.addWidget(self.cmb_interface)

        lay.addWidget(make_label("Channel:"))
        self.txt_channel = QLineEdit("/dev/ttyACM0")
        self.txt_channel.setFixedWidth(140)
        lay.addWidget(self.txt_channel)

        lay.addWidget(make_label("Bitrate:"))
        self.cmb_bitrate = QComboBox()
        self.cmb_bitrate.addItems(["250000", "500000", "1000000", "125000"])
        lay.addWidget(self.cmb_bitrate)

        self.led_conn = LEDIndicator()
        lay.addWidget(self.led_conn)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(100)
        self.btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self.btn_connect)

        self.btn_toggle_log = QPushButton("Show CAN Log")
        self.btn_toggle_log.setCheckable(True)
        self.btn_toggle_log.toggled.connect(self._toggle_log_panel)
        lay.addWidget(self.btn_toggle_log)

        lay.addStretch()

        poll_label = make_label("Poll (ms):")
        poll_label.setEnabled(self._can_monitoring_enabled)
        lay.addWidget(poll_label)
        self.spin_poll = QSpinBox()
        self.spin_poll.setRange(50, 5000)
        self.spin_poll.setValue(200)
        self.spin_poll.valueChanged.connect(
            lambda v: setattr(self, '_poll_interval_ms', v))
        self.spin_poll.setEnabled(self._can_monitoring_enabled)
        if not self._can_monitoring_enabled:
            self.spin_poll.setToolTip(
                "Background CAN polling is disabled for performance.")
        lay.addWidget(self.spin_poll)

        return grp

    # ── Connection ───────────────────────────────────────────────────────

    def _toggle_connection(self):
        if self.bus.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _set_can_log_enabled(self, enabled: bool):
        if enabled:
            self.bus.set_message_log_callback(
                lambda s: self.bridge.log_signal.emit(s))
        else:
            self.bus.set_message_log_callback(None)

    def _toggle_log_panel(self, checked: bool):
        self.log_group.setVisible(checked)
        if checked:
            self.btn_toggle_log.setText("Hide CAN Log")
            self._set_can_log_enabled(True)
            self.main_splitter.setSizes([3, 1])
        else:
            self.btn_toggle_log.setText("Show CAN Log")
            self._set_can_log_enabled(False)
            self.main_splitter.setSizes([1, 0])

    def _connect(self):
        interface = self.cmb_interface.currentText()
        channel = self.txt_channel.text().strip()
        bitrate = int(self.cmb_bitrate.currentText())

        if not channel:
            QMessageBox.warning(self, "Error", "Please enter a CAN channel.")
            return

        try:
            self.bus.connect(interface=interface, channel=channel,
                             bitrate=bitrate)
            self.btn_connect.setText("Disconnect")
            self.led_conn.set_color("#00cc44")
            status = f"Connected: {interface} @ {channel}, {bitrate} bps"
            if not self._can_monitoring_enabled:
                status += " | background monitoring disabled"
            self.statusBar().showMessage(status)
            if self._can_monitoring_enabled:
                self._poll_timer.start(self._poll_interval_ms)
            self._status_poll_timer.start(200)  # 5 Hz motor overview refresh
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", str(e))

    def _disconnect(self):
        self._poll_timer.stop()
        self._status_poll_timer.stop()
        self.bus.disconnect()
        self.btn_connect.setText("Connect")
        self.led_conn.set_color("gray")
        self.statusBar().showMessage("Disconnected")

    # ── Polling ──────────────────────────────────────────────────────────

    def _poll_feedback(self):
        if not self._can_monitoring_enabled or not self.bus.is_connected:
            return
        try:
            nid = self.selected_node_id
            self.bus.request_encoder_estimates(nid)
            self.bus.request_bus_voltage_current(nid)
            self.bus.request_iq(nid)
            self.bus.request_torques(nid)
            self.bus.request_powers(nid)

            # Poll all motors for visualization if enabled
            if hasattr(self, 'chk_poll_all') and self.chk_poll_all.isChecked():
                for poll_nid in self._get_all_node_ids():
                    if poll_nid != nid:
                        self.bus.request_encoder_estimates(poll_nid)
        except Exception:
            pass

    # ── UI Update Slots ──────────────────────────────────────────────────

    def _on_heartbeat_ui(self, node_id: int):
        if not self._can_monitoring_enabled:
            return
        fb = self.bus.get_feedback(node_id)
        if not fb:
            return

        if node_id == self.selected_node_id:
            state_name = AXIS_STATE_NAMES.get(fb.axis_state, f"Unknown({fb.axis_state})")
            self.lbl_motor_state.setText(f"State: {state_name}")

            if fb.axis_state == AxisState.CLOSED_LOOP:
                self.led_heartbeat.set_color("#00cc44")
            elif fb.axis_state == AxisState.IDLE:
                self.led_heartbeat.set_color("#ffcc00")
            elif fb.axis_error != 0:
                self.led_heartbeat.set_color("#ff3333")
            else:
                self.led_heartbeat.set_color("#3399ff")

            self.fb_fields['fb_axis_error'].setText(f"0x{fb.axis_error:08X}")
            self.fb_fields['fb_life'].setText(str(fb.life))

        # Update all-motors tab
        self._update_all_motors_status(node_id, fb)

    def _on_feedback_ui(self, node_id: int, feedback_type: str):
        if not self._can_monitoring_enabled:
            return
        # Update robot visualization on any encoder feedback
        if feedback_type == 'encoder' and hasattr(self, 'robot_viz_plot'):
            self._update_robot_viz()

        if node_id != self.selected_node_id:
            return

        fb = self.bus.get_feedback(node_id)
        if not fb:
            return

        if feedback_type == 'encoder':
            self.fb_fields['fb_pos'].setText(f"{fb.pos_estimate:.4f}")
            self.fb_fields['fb_vel'].setText(f"{fb.vel_estimate:.3f}")
        elif feedback_type == 'iq':
            self.fb_fields['fb_iq_set'].setText(f"{fb.iq_setpoint:.3f}")
            self.fb_fields['fb_iq_meas'].setText(f"{fb.iq_measured:.3f}")
        elif feedback_type == 'bus':
            self.fb_fields['fb_vbus'].setText(f"{fb.bus_voltage:.2f}")
            self.fb_fields['fb_ibus'].setText(f"{fb.bus_current:.3f}")
        elif feedback_type == 'torques':
            self.fb_fields['fb_torque_set'].setText(f"{fb.torque_setpoint:.3f}")
            self.fb_fields['fb_torque_meas'].setText(f"{fb.torque_measured:.3f}")
        elif feedback_type == 'powers':
            self.fb_fields['fb_elec_power'].setText(f"{fb.electrical_power:.2f}")
            self.fb_fields['fb_mech_power'].setText(f"{fb.mechanical_power:.2f}")

    def _append_log(self, text: str):
        self.log_box.append(text)
        sb = self.log_box.verticalScrollBar()
        sb.setValue(sb.maximum())
        if self._run_logging:
            self._run_log.append(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]}  {text}")

    def _save_run_log(self):
        if not self._run_log:
            return
        logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(logs_dir, exist_ok=True)
        fname = f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        path = os.path.join(logs_dir, fname)
        with open(path, "w") as fh:
            fh.write("\n".join(self._run_log) + "\n")
        self._append_log(f"[LOG] Run log saved → {path}")

    # ── Helpers ──────────────────────────────────────────────────────────

    def _check_connected(self) -> bool:
        if not self.bus.is_connected:
            QMessageBox.warning(self, "Not Connected",
                                "Please connect to CAN bus first.")
            return False
        return True

    def _get_all_node_ids(self) -> list[int]:
        ids = set()
        for spin in self.node_id_spins.values():
            ids.add(spin.value())
        return sorted(ids)

    # ── Theme ────────────────────────────────────────────────────────────

    def _apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #d4d4d4;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 14px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #7fba00;
            }
            QPushButton {
                background-color: #333;
                color: #d4d4d4;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 4px 10px;
                min-height: 22px;
            }
            QPushButton:hover {
                background-color: #444;
                border-color: #777;
            }
            QPushButton:pressed {
                background-color: #555;
            }
            QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
                background-color: #2b2b2b;
                color: #d4d4d4;
                border: 1px solid #555;
                border-radius: 2px;
                padding: 2px 4px;
            }
            QTextEdit {
                background-color: #1a1a1a;
                color: #cccccc;
                border: 1px solid #444;
            }
            QTabWidget::pane {
                border: 1px solid #444;
            }
            QTabBar::tab {
                background: #2b2b2b;
                color: #d4d4d4;
                padding: 6px 16px;
                border: 1px solid #444;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background: #1e1e1e;
                color: #7fba00;
            }
            QStatusBar {
                background: #252525;
                color: #888;
            }
            QSplitter::handle {
                background: #444;
                width: 3px;
            }
        """)

    # ── Power Bar ────────────────────────────────────────────────────────

    def _build_power_bar(self) -> QGroupBox:
        grp = QGroupBox("Power Status")
        grp.setMaximumHeight(62)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(8)

        # ── Powerbank (INA219 on I²C) ─────────────────────────────────────
        lay.addWidget(make_label("Powerbank:", bold=True))

        self.lbl_pb_voltage = QLabel("—")
        self.lbl_pb_voltage.setFixedWidth(66)
        self.lbl_pb_voltage.setToolTip("Bus voltage measured by INA219 (load side)")
        lay.addWidget(self.lbl_pb_voltage)

        self.lbl_pb_current = QLabel("—")
        self.lbl_pb_current.setFixedWidth(72)
        self.lbl_pb_current.setToolTip("Current draw measured by INA219")
        lay.addWidget(self.lbl_pb_current)

        self.lbl_pb_power = QLabel("—")
        self.lbl_pb_power.setFixedWidth(60)
        self.lbl_pb_power.setToolTip("Power (V × I) measured by INA219")
        lay.addWidget(self.lbl_pb_power)

        self.lbl_pb_pct = QLabel("—")
        self.lbl_pb_pct.setFixedWidth(52)
        self.lbl_pb_pct.setToolTip(
            "Powerbank charge level.\n"
            "Assumes voltage range 9.0 V (0%) → 12.6 V (100%)\n"
            "(3-cell lithium pack or USB-PD 12 V output).")
        lay.addWidget(self.lbl_pb_pct)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setFrameShadow(QFrame.Sunken)
        lay.addWidget(sep)

        # ── Motor battery (6S LiPo, voltage from CAN feedback) ────────────
        lay.addWidget(make_label("Motor Battery (6S LiPo):", bold=True))

        self.lbl_mb_voltage = QLabel("—")
        self.lbl_mb_voltage.setFixedWidth(66)
        self.lbl_mb_voltage.setToolTip(
            "Average bus voltage reported by motors (idle = battery voltage).")
        lay.addWidget(self.lbl_mb_voltage)

        self.lbl_mb_pct = QLabel("—")
        self.lbl_mb_pct.setFixedWidth(52)
        self.lbl_mb_pct.setToolTip(
            "6S LiPo charge level.\n"
            "Safe range: 19.8 V (3.30 V/cell, 0%) → 25.2 V (4.20 V/cell, 100%).\n"
            "Minimum recommended operating voltage: 21.0 V (3.50 V/cell).")
        lay.addWidget(self.lbl_mb_pct)

        lay.addStretch()
        return grp

    @staticmethod
    def _pct_color(pct: float) -> str:
        if pct > 50:
            return "#00cc44"
        if pct > 20:
            return "#ffcc00"
        return "#ff4444"

    def _init_ina219(self):
        try:
            from INA219 import INA219  # noqa: PLC0415
            self._ina219 = INA219(addr=0x41)
            self._ina_timer.start(2000)  # read every 2 s
        except Exception:
            # Sensor absent, smbus not installed, or permission denied — show static N/A.
            self.lbl_pb_voltage.setText("N/A")
            self.lbl_pb_current.setText("")
            self.lbl_pb_power.setText("")
            self.lbl_pb_pct.setText("N/A")

    def _ina_tick(self):
        if self._ina219 is None:
            return
        try:
            v = self._ina219.getBusVoltage_V()
            i_ma = self._ina219.getCurrent_mA()
            p = self._ina219.getPower_W()
            pct = max(0.0, min(100.0, (v - 9.0) / 3.6 * 100.0))

            self.lbl_pb_voltage.setText(f"{v:.2f} V")
            self.lbl_pb_current.setText(f"{i_ma / 1000:.3f} A")
            self.lbl_pb_power.setText(f"{p:.2f} W")
            self.lbl_pb_pct.setText(f"{pct:.0f}%")
            color = self._pct_color(pct)
            self.lbl_pb_pct.setStyleSheet(
                f"font-weight: bold; font-size: 13px; color: {color};")
        except Exception:
            self.lbl_pb_voltage.setText("Err")

    # ── Motor status / power refresh ─────────────────────────────────────

    def _refresh_all_motors_status(self):
        """Poll encoder estimates for all motors and refresh the status table.

        Runs at 5 Hz whenever the bus is connected, regardless of whether the
        full CAN-monitoring pipeline (_can_monitoring_enabled) is active.
        Bus voltage is requested every 5th call (1 Hz) to keep CAN traffic low.
        """
        if not self.bus.is_connected:
            return
        from steadywin_can import AXIS_STATE_NAMES, AxisState

        self._status_poll_count += 1
        request_vbus = (self._status_poll_count % 5 == 0)

        vbus_samples = []

        for (leg_idx, joint_idx), labels in self.all_motors_labels.items():
            spin = self.node_id_spins[(leg_idx, joint_idx)]
            nid = spin.value()
            labels['node'].setText(str(nid))
            try:
                self.bus.request_encoder_estimates(nid)
                if request_vbus:
                    self.bus.request_bus_voltage_current(nid)
            except Exception:
                pass
            fb = self.bus.get_feedback(nid)
            if fb is None:
                continue
            labels['state'].setText(AXIS_STATE_NAMES.get(fb.axis_state, "?"))
            labels['error'].setText(
                f"0x{fb.axis_error:04X}" if fb.axis_error else "OK")
            labels['pos'].setText(f"{fb.pos_estimate:.3f}")
            labels['vel'].setText(f"{fb.vel_estimate:.2f}")
            labels['vbus'].setText(f"{fb.bus_voltage:.1f}")
            if fb.axis_state == AxisState.CLOSED_LOOP:
                labels['led'].set_color("#00cc44")
            elif fb.axis_error:
                labels['led'].set_color("#ff3333")
            elif fb.axis_state == AxisState.IDLE:
                labels['led'].set_color("#ffcc00")
            else:
                labels['led'].set_color("gray")

            if fb.bus_voltage > 1.0:
                vbus_samples.append(fb.bus_voltage)

        # Update motor battery display from average vbus across all motors.
        if vbus_samples:
            v = sum(vbus_samples) / len(vbus_samples)
            # 6S LiPo: 3.30 V/cell × 6 = 19.8 V (0%) → 4.20 V/cell × 6 = 25.2 V (100%)
            pct = max(0.0, min(100.0, (v - 19.8) / (25.2 - 19.8) * 100.0))
            self.lbl_mb_voltage.setText(f"{v:.1f} V")
            self.lbl_mb_pct.setText(f"{pct:.0f}%")
            color = self._pct_color(pct)
            self.lbl_mb_pct.setStyleSheet(
                f"font-weight: bold; font-size: 13px; color: {color};")

    def closeEvent(self, event):
        if self._pid_running:
            self._pid_stop()
        self._poll_timer.stop()
        self._status_poll_timer.stop()
        self._ina_timer.stop()
        if self.bus.is_connected:
            self.bus.disconnect()
        event.accept()


# ─── Entry Point ──────────────────────────────────────────────────────────────

def main():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    app.setApplicationName("SteadyWin Motor Controller")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
