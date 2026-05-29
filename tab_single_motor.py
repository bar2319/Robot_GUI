"""
Single Motor tab — state control, mode, targets, gains, feedback, limits, actions.
Used as a mixin for MainWindow.
"""

import time
from functools import partial

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QComboBox, QSpinBox, QMessageBox,
)

from helpers import (
    LEDIndicator, make_label, make_readonly_line, make_double_spin,
)
from steadywin_can import (
    AxisState, ControlMode, InputMode,
    CONTROL_MODE_NAMES, INPUT_MODE_NAMES,
)


class SingleMotorMixin:
    """Mixin providing the Single Motor tab and its slot methods."""

    def _build_single_motor_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(6)

        # Motor selector
        sel_box = QHBoxLayout()
        sel_box.addWidget(make_label("Node ID:", bold=True))
        self.spin_node_id = QSpinBox()
        self.spin_node_id.setRange(0, 63)
        self.spin_node_id.setValue(0)
        self.spin_node_id.valueChanged.connect(self._on_node_id_changed)
        sel_box.addWidget(self.spin_node_id)
        sel_box.addSpacing(20)

        self.lbl_motor_state = make_label("State: —")
        self.lbl_motor_state.setStyleSheet("font-weight: bold; font-size: 13px;")
        sel_box.addWidget(self.lbl_motor_state)
        self.led_heartbeat = LEDIndicator()
        sel_box.addWidget(self.led_heartbeat)
        sel_box.addStretch()
        layout.addLayout(sel_box)

        # Two columns
        cols = QHBoxLayout()

        left = QVBoxLayout()
        left.addWidget(self._build_state_control_group())
        left.addWidget(self._build_mode_control_group())
        left.addWidget(self._build_target_group())
        left.addWidget(self._build_gains_group())
        left.addStretch()
        cols.addLayout(left, 1)

        right = QVBoxLayout()
        right.addWidget(self._build_feedback_group())
        right.addWidget(self._build_limits_group())
        right.addWidget(self._build_actions_group())
        right.addStretch()
        cols.addLayout(right, 1)

        layout.addLayout(cols, 1)
        return widget

    # ── Sub-groups ───────────────────────────────────────────────────────

    def _build_state_control_group(self) -> QGroupBox:
        grp = QGroupBox("State Control")
        lay = QGridLayout(grp)

        states = [
            ("IDLE", AxisState.IDLE, "#888"),
            ("Motor Calib", AxisState.MOTOR_CALIBRATION, "#e6a800"),
            ("Encoder Calib", AxisState.ENCODER_CALIBRATION, "#e6a800"),
            ("Full Calib", AxisState.FULL_CALIBRATION, "#e68a00"),
            ("Closed Loop", AxisState.CLOSED_LOOP, "#00b33c"),
            ("Homing", AxisState.HOMING, "#3399ff"),
        ]

        for i, (label, state, color) in enumerate(states):
            btn = QPushButton(label)
            btn.setStyleSheet(f"QPushButton {{ border: 2px solid {color}; padding: 6px; }}"
                              f"QPushButton:hover {{ background: {color}; color: white; }}")
            btn.clicked.connect(partial(self._set_state, state))
            lay.addWidget(btn, i // 3, i % 3)

        return grp

    def _build_mode_control_group(self) -> QGroupBox:
        grp = QGroupBox("Control & Input Mode")
        lay = QGridLayout(grp)

        lay.addWidget(make_label("Control Mode:"), 0, 0)
        self.cmb_control_mode = QComboBox()
        for val, name in sorted(CONTROL_MODE_NAMES.items()):
            self.cmb_control_mode.addItem(f"{val}: {name}", val)
        self.cmb_control_mode.setCurrentIndex(2)
        lay.addWidget(self.cmb_control_mode, 0, 1)

        lay.addWidget(make_label("Input Mode:"), 1, 0)
        self.cmb_input_mode = QComboBox()
        for val, name in sorted(INPUT_MODE_NAMES.items()):
            self.cmb_input_mode.addItem(f"{val}: {name}", val)
        self.cmb_input_mode.setCurrentIndex(1)
        lay.addWidget(self.cmb_input_mode, 1, 1)

        btn_set_mode = QPushButton("Set Mode")
        btn_set_mode.clicked.connect(self._set_controller_mode)
        lay.addWidget(btn_set_mode, 0, 2, 2, 1)

        return grp

    def _build_target_group(self) -> QGroupBox:
        grp = QGroupBox("Target Control")
        lay = QGridLayout(grp)

        # Position
        lay.addWidget(make_label("Position (rev):"), 0, 0)
        self.spin_pos = make_double_spin(-1000, 1000, 4, 0.0, 0.1)
        lay.addWidget(self.spin_pos, 0, 1)
        lay.addWidget(make_label("Vel FF (rev/s):"), 0, 2)
        self.spin_pos_vel_ff = make_double_spin(-100, 100, 3, 0.0, 0.1)
        lay.addWidget(self.spin_pos_vel_ff, 0, 3)
        lay.addWidget(make_label("Torque FF (Nm):"), 0, 4)
        self.spin_pos_torque_ff = make_double_spin(-50, 50, 3, 0.0, 0.1)
        lay.addWidget(self.spin_pos_torque_ff, 0, 5)
        btn_pos = QPushButton("Send Pos")
        btn_pos.setStyleSheet("background: #1a5276; padding: 6px;")
        btn_pos.clicked.connect(self._send_pos)
        lay.addWidget(btn_pos, 0, 6)

        # Velocity
        lay.addWidget(make_label("Velocity (rev/s):"), 1, 0)
        self.spin_vel = make_double_spin(-100, 100, 3, 0.0, 0.5)
        lay.addWidget(self.spin_vel, 1, 1)
        lay.addWidget(make_label("Torque FF (Nm):"), 1, 2)
        self.spin_vel_torque_ff = make_double_spin(-50, 50, 3, 0.0, 0.1)
        lay.addWidget(self.spin_vel_torque_ff, 1, 3)
        btn_vel = QPushButton("Send Vel")
        btn_vel.setStyleSheet("background: #1a5276; padding: 6px;")
        btn_vel.clicked.connect(self._send_vel)
        lay.addWidget(btn_vel, 1, 6)

        # Torque
        lay.addWidget(make_label("Torque (Nm):"), 2, 0)
        self.spin_torque = make_double_spin(-50, 50, 3, 0.0, 0.1)
        lay.addWidget(self.spin_torque, 2, 1)
        btn_torque = QPushButton("Send Torque")
        btn_torque.setStyleSheet("background: #1a5276; padding: 6px;")
        btn_torque.clicked.connect(self._send_torque)
        lay.addWidget(btn_torque, 2, 6)

        # MIT Control
        lay.addWidget(make_label("— MIT Control —", bold=True), 3, 0, 1, 7)
        lay.addWidget(make_label("Pos (rad):"), 4, 0)
        self.spin_mit_pos = make_double_spin(-12.5, 12.5, 3, 0.0, 0.1)
        lay.addWidget(self.spin_mit_pos, 4, 1)
        lay.addWidget(make_label("Vel (rad/s):"), 4, 2)
        self.spin_mit_vel = make_double_spin(-65, 65, 3, 0.0, 1.0)
        lay.addWidget(self.spin_mit_vel, 4, 3)
        lay.addWidget(make_label("KP:"), 4, 4)
        self.spin_mit_kp = make_double_spin(0, 500, 2, 0.0, 1.0)
        lay.addWidget(self.spin_mit_kp, 4, 5)
        lay.addWidget(make_label("KD:"), 5, 0)
        self.spin_mit_kd = make_double_spin(0, 5, 3, 0.0, 0.1)
        lay.addWidget(self.spin_mit_kd, 5, 1)
        lay.addWidget(make_label("Torque (Nm):"), 5, 2)
        self.spin_mit_torque = make_double_spin(-50, 50, 3, 0.0, 0.1)
        lay.addWidget(self.spin_mit_torque, 5, 3)
        btn_mit = QPushButton("Send MIT")
        btn_mit.setStyleSheet("background: #7b241c; padding: 6px;")
        btn_mit.clicked.connect(self._send_mit)
        lay.addWidget(btn_mit, 5, 6)

        return grp

    def _build_gains_group(self) -> QGroupBox:
        grp = QGroupBox("PID Gains")
        lay = QGridLayout(grp)

        lay.addWidget(make_label("Pos Gain (Kp):"), 0, 0)
        self.spin_pos_gain = make_double_spin(0, 500, 2, 20.0, 1.0)
        lay.addWidget(self.spin_pos_gain, 0, 1)
        btn_pg = QPushButton("Set")
        btn_pg.clicked.connect(self._set_pos_gain)
        lay.addWidget(btn_pg, 0, 2)

        lay.addWidget(make_label("Vel Gain (Kp):"), 1, 0)
        self.spin_vel_gain = make_double_spin(0, 100, 4, 0.16, 0.01)
        lay.addWidget(self.spin_vel_gain, 1, 1)
        lay.addWidget(make_label("Vel Int (Ki):"), 1, 2)
        self.spin_vel_int_gain = make_double_spin(0, 100, 4, 0.32, 0.01)
        lay.addWidget(self.spin_vel_int_gain, 1, 3)
        btn_vg = QPushButton("Set")
        btn_vg.clicked.connect(self._set_vel_gains)
        lay.addWidget(btn_vg, 1, 4)

        return grp

    def _build_feedback_group(self) -> QGroupBox:
        grp = QGroupBox("Live Feedback")
        lay = QGridLayout(grp)

        fields = [
            ("Position (rev):", "fb_pos"),
            ("Velocity (rev/s):", "fb_vel"),
            ("Iq Setpoint (A):", "fb_iq_set"),
            ("Iq Measured (A):", "fb_iq_meas"),
            ("Bus Voltage (V):", "fb_vbus"),
            ("Bus Current (A):", "fb_ibus"),
            ("Torque Target (Nm):", "fb_torque_set"),
            ("Torque Measured (Nm):", "fb_torque_meas"),
            ("Elec Power (W):", "fb_elec_power"),
            ("Mech Power (W):", "fb_mech_power"),
            ("Axis Error:", "fb_axis_error"),
            ("Life Counter:", "fb_life"),
        ]

        self.fb_fields = {}
        for i, (label, key) in enumerate(fields):
            lay.addWidget(make_label(label), i, 0)
            le = make_readonly_line()
            lay.addWidget(le, i, 1)
            self.fb_fields[key] = le

        return grp

    def _build_limits_group(self) -> QGroupBox:
        grp = QGroupBox("Limits")
        lay = QGridLayout(grp)

        lay.addWidget(make_label("Vel Limit (rev/s):"), 0, 0)
        self.spin_vel_limit = make_double_spin(0, 200, 2, 30.0, 1.0)
        lay.addWidget(self.spin_vel_limit, 0, 1)

        lay.addWidget(make_label("Current Limit (A):"), 0, 2)
        self.spin_current_limit = make_double_spin(0, 100, 1, 10.0, 1.0)
        lay.addWidget(self.spin_current_limit, 0, 3)

        btn = QPushButton("Set Limits")
        btn.clicked.connect(self._set_limits)
        lay.addWidget(btn, 0, 4)

        lay.addWidget(make_label("Traj Vel (rev/s):"), 1, 0)
        self.spin_traj_vel = make_double_spin(0, 200, 2, 10.0, 1.0)
        lay.addWidget(self.spin_traj_vel, 1, 1)

        lay.addWidget(make_label("Accel (rev/s²):"), 1, 2)
        self.spin_traj_accel = make_double_spin(0, 500, 1, 50.0, 5.0)
        lay.addWidget(self.spin_traj_accel, 1, 3)

        lay.addWidget(make_label("Decel (rev/s²):"), 2, 0)
        self.spin_traj_decel = make_double_spin(0, 500, 1, 50.0, 5.0)
        lay.addWidget(self.spin_traj_decel, 2, 1)

        btn_traj = QPushButton("Set Traj Limits")
        btn_traj.clicked.connect(self._set_traj_limits)
        lay.addWidget(btn_traj, 2, 4)

        return grp

    def _build_actions_group(self) -> QGroupBox:
        grp = QGroupBox("Actions")
        lay = QGridLayout(grp)

        actions = [
            ("E-STOP", self._estop, "background: #922b21; font-weight: bold; padding: 8px;"),
            ("Clear Errors", self._clear_errors, ""),
            ("Save Config", self._save_config, ""),
            ("Reboot", self._reboot, "background: #7d6608;"),
            ("Anticogging", self._start_anticogging, ""),
            ("Set Zero Here", self._set_zero_here, ""),
        ]

        for i, (label, func, style) in enumerate(actions):
            btn = QPushButton(label)
            if style:
                btn.setStyleSheet(style)
            btn.clicked.connect(func)
            lay.addWidget(btn, i // 3, i % 3)

        return grp

    # ── Slot methods ─────────────────────────────────────────────────────

    def _on_node_id_changed(self, nid: int):
        self.selected_node_id = nid
        self.bus.register_motor(nid)

    def _set_state(self, state: int):
        if not self._check_connected():
            return
        self.bus.set_axis_state(self.selected_node_id, state)

    def _set_all_state(self, state: int):
        if not self._check_connected():
            return
        for nid in self._get_all_node_ids():
            self.bus.set_axis_state(nid, state)
            time.sleep(0.01)

    def _set_controller_mode(self):
        if not self._check_connected():
            return
        cm = self.cmb_control_mode.currentData()
        im = self.cmb_input_mode.currentData()
        self.bus.set_controller_mode(self.selected_node_id, cm, im)

    def _send_pos(self):
        if not self._check_connected():
            return
        self.bus.set_input_pos(
            self.selected_node_id,
            self.spin_pos.value(),
            self.spin_pos_vel_ff.value(),
            self.spin_pos_torque_ff.value(),
        )

    def _send_vel(self):
        if not self._check_connected():
            return
        self.bus.set_input_vel(
            self.selected_node_id,
            self.spin_vel.value(),
            self.spin_vel_torque_ff.value(),
        )

    def _send_torque(self):
        if not self._check_connected():
            return
        self.bus.set_input_torque(
            self.selected_node_id,
            self.spin_torque.value(),
        )

    def _send_mit(self):
        if not self._check_connected():
            return
        self.bus.mit_control(
            self.selected_node_id,
            self.spin_mit_pos.value(),
            self.spin_mit_vel.value(),
            self.spin_mit_kp.value(),
            self.spin_mit_kd.value(),
            self.spin_mit_torque.value(),
        )

    def _set_pos_gain(self):
        if not self._check_connected():
            return
        self.bus.set_pos_gain(self.selected_node_id, self.spin_pos_gain.value())

    def _set_vel_gains(self):
        if not self._check_connected():
            return
        self.bus.set_vel_gains(
            self.selected_node_id,
            self.spin_vel_gain.value(),
            self.spin_vel_int_gain.value(),
        )

    def _set_limits(self):
        if not self._check_connected():
            return
        self.bus.set_limits(
            self.selected_node_id,
            self.spin_vel_limit.value(),
            self.spin_current_limit.value(),
        )

    def _set_traj_limits(self):
        if not self._check_connected():
            return
        nid = self.selected_node_id
        self.bus.set_traj_vel_limit(nid, self.spin_traj_vel.value())
        time.sleep(0.005)
        self.bus.set_traj_accel_limits(
            nid, self.spin_traj_accel.value(), self.spin_traj_decel.value())

    def _estop(self):
        if not self._check_connected():
            return
        self.bus.estop(self.selected_node_id)

    def _estop_all(self):
        if not self._check_connected():
            return
        for nid in self._get_all_node_ids():
            self.bus.estop(nid)

    def _clear_errors(self):
        if not self._check_connected():
            return
        self.bus.clear_errors(self.selected_node_id)

    def _clear_all_errors(self):
        if not self._check_connected():
            return
        for nid in self._get_all_node_ids():
            self.bus.clear_errors(nid)
            time.sleep(0.005)

    def _save_config(self):
        if not self._check_connected():
            return
        self.bus.save_configuration(self.selected_node_id)

    def _save_all_config(self):
        if not self._check_connected():
            return
        for nid in self._get_all_node_ids():
            self.bus.save_configuration(nid)
            time.sleep(0.05)

    def _reboot(self):
        if not self._check_connected():
            return
        self.bus.reboot(self.selected_node_id)

    def _start_anticogging(self):
        if not self._check_connected():
            return
        self.bus.start_anticogging(self.selected_node_id)

    def _set_zero_here(self):
        if not self._check_connected():
            return
        self.bus.set_linear_count(self.selected_node_id, 0)
