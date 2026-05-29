"""
One Leg tab — control two motors (hip + knee) with IK, calibration,
gait, stand-up, and offset saving.
Used as a mixin for MainWindow.
"""

import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QSpinBox, QLabel, QScrollArea, QComboBox,
)

from helpers import (
    LEDIndicator, make_label, make_readonly_line, make_double_spin,
)
from steadywin_can import AxisState, ControlMode, InputMode


class OneLegMixin:
    """Mixin providing the One Leg tab for single-leg control."""

    # ── State init (called from MainWindow.__init__) ─────────────────────

    def _init_one_leg_state(self):
        self._ol_running = False
        self._ol_mode = None  # 'stand', 'sit', 'gait'
        self._ol_timer = QTimer()
        self._ol_timer.timeout.connect(self._ol_tick)
        self._ol_start_time = 0.0
        self._ol_start_pose = [0.0, 0.0]  # hip, knee
        self._ol_sitting_pose = [0.0, 0.0]
        self._ol_step = 0

        # Viz update timer
        self._ol_viz_timer = QTimer()
        self._ol_viz_timer.timeout.connect(self._ol_update_viz)

    # ── Build tab ────────────────────────────────────────────────────────

    def _build_one_leg_tab(self) -> QWidget:
        widget = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        layout = QVBoxLayout(inner)

        # ── Axis Selection ──
        sel_grp = QGroupBox("Axis Selection")
        sel_lay = QHBoxLayout(sel_grp)

        sel_lay.addWidget(make_label("Hip Axis ID:", bold=True))
        self.ol_spin_hip = QSpinBox()
        self.ol_spin_hip.setRange(0, 63)
        self.ol_spin_hip.setValue(0)
        sel_lay.addWidget(self.ol_spin_hip)

        sel_lay.addSpacing(20)

        sel_lay.addWidget(make_label("Knee Axis ID:", bold=True))
        self.ol_spin_knee = QSpinBox()
        self.ol_spin_knee.setRange(0, 63)
        self.ol_spin_knee.setValue(1)
        sel_lay.addWidget(self.ol_spin_knee)

        sel_lay.addSpacing(20)

        sel_lay.addWidget(make_label("Direction:"))
        self.ol_cmb_direction = QComboBox()
        self.ol_cmb_direction.addItems(["Left (+1, +1)", "Right (-1, -1)"])
        sel_lay.addWidget(self.ol_cmb_direction)

        sel_lay.addStretch()

        # Status
        self.ol_status_led = LEDIndicator(14)
        sel_lay.addWidget(self.ol_status_led)
        self.ol_lbl_status = QLabel("Idle")
        self.ol_lbl_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        sel_lay.addWidget(self.ol_lbl_status)

        layout.addWidget(sel_grp)

        # ── Quick Actions ──
        act_grp = QGroupBox("Motor Actions")
        act_lay = QGridLayout(act_grp)

        btn_calib = QPushButton("Run Calibration (Both)")
        btn_calib.setStyleSheet("background: #b7950b; font-weight: bold; padding: 8px;")
        btn_calib.clicked.connect(self._ol_calibrate)
        act_lay.addWidget(btn_calib, 0, 0)

        btn_closed = QPushButton("Enter Closed Loop (Both)")
        btn_closed.setStyleSheet("background: #1e8449; font-weight: bold; padding: 8px;")
        btn_closed.clicked.connect(self._ol_enter_closed_loop)
        act_lay.addWidget(btn_closed, 0, 1)

        btn_idle = QPushButton("Set IDLE (Both)")
        btn_idle.setStyleSheet("padding: 8px;")
        btn_idle.clicked.connect(self._ol_set_idle)
        act_lay.addWidget(btn_idle, 0, 2)

        btn_clear_err = QPushButton("Clear Errors (Both)")
        btn_clear_err.clicked.connect(self._ol_clear_errors)
        act_lay.addWidget(btn_clear_err, 0, 3)

        btn_reboot = QPushButton("Reboot (Both)")
        btn_reboot.setStyleSheet("background: #922b21; padding: 8px;")
        btn_reboot.clicked.connect(self._ol_reboot)
        act_lay.addWidget(btn_reboot, 0, 4)

        layout.addWidget(act_grp)

        # ── Feedback ──
        fb_grp = QGroupBox("Live Feedback")
        fb_lay = QGridLayout(fb_grp)

        fb_lay.addWidget(make_label("", bold=True), 0, 0)
        fb_lay.addWidget(make_label("Position (rev)", bold=True), 0, 1)
        fb_lay.addWidget(make_label("Velocity (rev/s)", bold=True), 0, 2)
        fb_lay.addWidget(make_label("Current Iq (A)", bold=True), 0, 3)
        fb_lay.addWidget(make_label("State", bold=True), 0, 4)

        fb_lay.addWidget(make_label("Hip:"), 1, 0)
        self.ol_fb_hip_pos = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_hip_pos, 1, 1)
        self.ol_fb_hip_vel = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_hip_vel, 1, 2)
        self.ol_fb_hip_iq = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_hip_iq, 1, 3)
        self.ol_fb_hip_state = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_hip_state, 1, 4)

        fb_lay.addWidget(make_label("Knee:"), 2, 0)
        self.ol_fb_knee_pos = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_knee_pos, 2, 1)
        self.ol_fb_knee_vel = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_knee_vel, 2, 2)
        self.ol_fb_knee_iq = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_knee_iq, 2, 3)
        self.ol_fb_knee_state = make_readonly_line(100)
        fb_lay.addWidget(self.ol_fb_knee_state, 2, 4)

        layout.addWidget(fb_grp)

        # ── Offset & Pose ──
        offset_grp = QGroupBox("Offsets & Sitting Pose")
        offset_lay = QGridLayout(offset_grp)

        offset_lay.addWidget(make_label("Hip Offset:", bold=True), 0, 0)
        self.ol_fb_hip_offset = make_readonly_line(100)
        self.ol_fb_hip_offset.setText("0.000")
        offset_lay.addWidget(self.ol_fb_hip_offset, 0, 1)

        offset_lay.addWidget(make_label("Knee Offset:", bold=True), 0, 2)
        self.ol_fb_knee_offset = make_readonly_line(100)
        self.ol_fb_knee_offset.setText("0.000")
        offset_lay.addWidget(self.ol_fb_knee_offset, 0, 3)

        btn_save_offset = QPushButton("Save Current as Offset (Fully Extended)")
        btn_save_offset.setStyleSheet("background: #2874a6; font-weight: bold; padding: 6px;")
        btn_save_offset.setToolTip(
            "With leg fully extended, save current positions as zero offsets")
        btn_save_offset.clicked.connect(self._ol_save_offset)
        offset_lay.addWidget(btn_save_offset, 0, 4)

        offset_lay.addWidget(make_label("Hip Sit Pose:", bold=True), 1, 0)
        self.ol_fb_hip_sit = make_readonly_line(100)
        self.ol_fb_hip_sit.setText("0.000")
        offset_lay.addWidget(self.ol_fb_hip_sit, 1, 1)

        offset_lay.addWidget(make_label("Knee Sit Pose:", bold=True), 1, 2)
        self.ol_fb_knee_sit = make_readonly_line(100)
        self.ol_fb_knee_sit.setText("0.000")
        offset_lay.addWidget(self.ol_fb_knee_sit, 1, 3)

        btn_save_sit = QPushButton("Save Current as Sitting Pose")
        btn_save_sit.clicked.connect(self._ol_save_sitting_pose)
        offset_lay.addWidget(btn_save_sit, 1, 4)

        layout.addWidget(offset_grp)

        # ── XY Movement ──
        xy_grp = QGroupBox("XY Leg Control (IK)")
        xy_lay = QGridLayout(xy_grp)

        xy_lay.addWidget(make_label("X (mm):"), 0, 0)
        self.ol_spin_x = make_double_spin(0, 300, 1, 200.0, 5.0)
        xy_lay.addWidget(self.ol_spin_x, 0, 1)

        xy_lay.addWidget(make_label("Y (mm):"), 0, 2)
        self.ol_spin_y = make_double_spin(-200, 200, 1, 0.0, 5.0)
        xy_lay.addWidget(self.ol_spin_y, 0, 3)

        btn_move = QPushButton("Move Leg")
        btn_move.setStyleSheet("background: #1e8449; padding: 6px;")
        btn_move.clicked.connect(self._ol_move_xy)
        xy_lay.addWidget(btn_move, 0, 4)

        # Preset positions
        xy_lay.addWidget(make_label("Presets:"), 1, 0)

        btn_extended = QPushButton("Fully Extended (266, 0)")
        btn_extended.clicked.connect(lambda: self._ol_move_to_preset(266.0, 0.0))
        xy_lay.addWidget(btn_extended, 1, 1)

        btn_stand = QPushButton("Standing (220, 0)")
        btn_stand.clicked.connect(lambda: self._ol_move_to_preset(220.0, 0.0))
        xy_lay.addWidget(btn_stand, 1, 2)

        btn_mid = QPushButton("Mid (180, 0)")
        btn_mid.clicked.connect(lambda: self._ol_move_to_preset(180.0, 0.0))
        xy_lay.addWidget(btn_mid, 1, 3)

        btn_crouch = QPushButton("Crouch (120, 0)")
        btn_crouch.clicked.connect(lambda: self._ol_move_to_preset(120.0, 0.0))
        xy_lay.addWidget(btn_crouch, 1, 4)

        layout.addWidget(xy_grp)

        # ── Stand / Sit ──
        ss_grp = QGroupBox("Stand Up / Sit Down (Single Leg)")
        ss_lay = QGridLayout(ss_grp)

        ss_lay.addWidget(make_label("Stand X (mm):"), 0, 0)
        self.ol_spin_stand_x = make_double_spin(50, 300, 1, 220.0, 5.0)
        ss_lay.addWidget(self.ol_spin_stand_x, 0, 1)

        ss_lay.addWidget(make_label("Stand Y (mm):"), 0, 2)
        self.ol_spin_stand_y = make_double_spin(-150, 150, 1, 0.0, 5.0)
        ss_lay.addWidget(self.ol_spin_stand_y, 0, 3)

        ss_lay.addWidget(make_label("Duration (s):"), 0, 4)
        self.ol_spin_stand_dur = make_double_spin(0.5, 10.0, 1, 2.0, 0.5)
        ss_lay.addWidget(self.ol_spin_stand_dur, 0, 5)

        btn_stand_up = QPushButton("▲ Stand Up")
        btn_stand_up.setStyleSheet("background: #1e8449; font-weight: bold; padding: 8px;")
        btn_stand_up.clicked.connect(self._ol_start_stand)
        ss_lay.addWidget(btn_stand_up, 1, 0, 1, 3)

        btn_sit_down = QPushButton("▼ Sit Down")
        btn_sit_down.setStyleSheet("background: #b7950b; font-weight: bold; padding: 8px;")
        btn_sit_down.clicked.connect(self._ol_start_sit)
        ss_lay.addWidget(btn_sit_down, 1, 3, 1, 3)

        layout.addWidget(ss_grp)

        # ── Gait ──
        gait_grp = QGroupBox("Single Leg Gait Test")
        gait_lay = QGridLayout(gait_grp)

        gait_lay.addWidget(make_label("Step Length (mm):"), 0, 0)
        self.ol_spin_step_len = make_double_spin(10, 200, 1, 90.0, 5.0)
        gait_lay.addWidget(self.ol_spin_step_len, 0, 1)

        gait_lay.addWidget(make_label("Stand Height (mm):"), 0, 2)
        self.ol_spin_gait_h = make_double_spin(50, 300, 1, 220.0, 5.0)
        gait_lay.addWidget(self.ol_spin_gait_h, 0, 3)

        gait_lay.addWidget(make_label("Lift Height (mm):"), 0, 4)
        self.ol_spin_gait_lift = make_double_spin(10, 200, 1, 100.0, 5.0)
        gait_lay.addWidget(self.ol_spin_gait_lift, 0, 5)

        gait_lay.addWidget(make_label("Cycle Time (ms):"), 1, 0)
        self.ol_spin_gait_cycle = QSpinBox()
        self.ol_spin_gait_cycle.setRange(200, 5000)
        self.ol_spin_gait_cycle.setValue(700)
        self.ol_spin_gait_cycle.setSingleStep(50)
        gait_lay.addWidget(self.ol_spin_gait_cycle, 1, 1)

        btn_gait_start = QPushButton("▶ Start Gait")
        btn_gait_start.setStyleSheet("background: #1e8449; font-weight: bold; padding: 8px;")
        btn_gait_start.clicked.connect(self._ol_start_gait)
        gait_lay.addWidget(btn_gait_start, 1, 2, 1, 2)

        btn_gait_stop = QPushButton("■ Stop")
        btn_gait_stop.setStyleSheet("background: #b03a2e; padding: 8px;")
        btn_gait_stop.clicked.connect(self._ol_stop)
        gait_lay.addWidget(btn_gait_stop, 1, 4, 1, 2)

        layout.addWidget(gait_grp)

        # ── Visualization Status ──
        viz_grp = QGroupBox("Leg Visualization")
        viz_lay = QVBoxLayout(viz_grp)

        viz_note = QLabel(
            "2-link leg visualization and live refresh are disabled to "
            "reduce GUI load.")
        viz_note.setWordWrap(True)
        viz_lay.addWidget(viz_note)
        layout.addWidget(viz_grp)

        layout.addStretch()
        scroll.setWidget(inner)

        outer = QVBoxLayout(widget)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)
        return widget

    # ── Helpers ──────────────────────────────────────────────────────────

    def _ol_hip_id(self) -> int:
        return self.ol_spin_hip.value()

    def _ol_knee_id(self) -> int:
        return self.ol_spin_knee.value()

    def _ol_directions(self):
        """Return (hip_dir, knee_dir) based on combo selection."""
        if self.ol_cmb_direction.currentIndex() == 0:
            return +1.0, +1.0
        else:
            return -1.0, -1.0

    def _ol_get_position(self, node_id: int) -> float:
        fb = self.bus.get_feedback(node_id)
        return fb.pos_estimate if fb else 0.0

    def _ol_get_iq(self, node_id: int) -> float:
        fb = self.bus.get_feedback(node_id)
        return fb.iq_measured if fb else 0.0

    def _ol_ik_angle(self, joint: int, x: float, y: float) -> float:
        """IK for one leg. joint=0 → hip, joint=1 → knee.
        Uses the selected axis offsets and directions."""
        L1 = self._robot_L1
        L2 = self._robot_L2
        GR = self._robot_GR

        r = math.sqrt(x * x + y * y)
        if r > (L1 + L2) or r < L1 / 2:
            return float('nan')

        if joint == 0:  # hip
            lam = math.atan2(y, x)
            theta = math.acos(
                max(-1.0, min(1.0, (r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))))
            temp = -(lam + theta)
        else:  # knee
            alpha = math.acos(
                max(-1.0, min(1.0, (L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))))
            temp = -(math.pi - alpha)

        hip_dir, knee_dir = self._ol_directions()
        d = hip_dir if joint == 0 else knee_dir

        base_turns = -((temp * 180.0 / math.pi) / 360.0 * GR)
        # Use per-leg offsets stored in _ol_offset
        offset = self._ol_offset[joint] if hasattr(self, '_ol_offset') else 0.0
        return offset + d * base_turns

    def _ol_fk(self, hip_pos: float, knee_pos: float):
        """Forward kinematics: motor positions (rev) → (x, y) in mm.
        Returns shoulder, elbow, foot coordinates."""
        L1 = self._robot_L1
        L2 = self._robot_L2
        GR = self._robot_GR

        hip_dir, knee_dir = self._ol_directions()
        hip_offset = self._ol_offset[0] if hasattr(self, '_ol_offset') else 0.0
        knee_offset = self._ol_offset[1] if hasattr(self, '_ol_offset') else 0.0

        # Reverse the IK transform to get joint angles in radians
        hip_base_turns = (hip_pos - hip_offset) / hip_dir if hip_dir != 0 else 0
        knee_base_turns = (knee_pos - knee_offset) / knee_dir if knee_dir != 0 else 0

        # base_turns = -((temp * 180 / pi) / 360 * GR)
        # so temp = -(base_turns * 360 / GR) * pi / 180
        hip_angle = -(hip_base_turns * 360.0 / GR) * math.pi / 180.0
        knee_angle = -(knee_base_turns * 360.0 / GR) * math.pi / 180.0

        # hip_angle = -(lambda + theta)  →  shoulder angle
        # knee_angle = -(pi - alpha)     →  elbow relative angle
        shoulder_angle = -hip_angle
        elbow_rel = -knee_angle  # = pi - alpha → alpha = pi - elbow_rel

        # Elbow position
        elbow_x = L1 * math.cos(shoulder_angle)
        elbow_y = L1 * math.sin(shoulder_angle)

        # The knee angle gives elbow-relative bending
        # alpha = pi + knee_angle (since knee_angle = -(pi - alpha))
        alpha = math.pi + knee_angle  # inter-link angle
        # Foot direction: shoulder_angle - (pi - alpha) = shoulder_angle - pi + alpha
        foot_angle = shoulder_angle - (math.pi - alpha)
        foot_x = elbow_x + L2 * math.cos(foot_angle)
        foot_y = elbow_y + L2 * math.sin(foot_angle)

        return (0.0, 0.0), (elbow_x, elbow_y), (foot_x, foot_y)

    def _ol_set_status(self, text: str, color: str = "#3399ff"):
        self.ol_lbl_status.setText(text)
        self.ol_status_led.set_color(color)

    # ── Motor Actions ────────────────────────────────────────────────────

    def _ol_calibrate(self):
        if not self._check_connected():
            return
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()
        self._append_log(f"[LEG] Calibrating hip={hip}, knee={knee}")

        # Calibrate hip first
        self.bus.set_axis_state(hip, AxisState.FULL_CALIBRATION)
        time.sleep(0.025)
        # Then knee
        self.bus.set_axis_state(knee, AxisState.FULL_CALIBRATION)
        self._ol_set_status("Calibrating...", "#e6a800")
        self._append_log("[LEG] Calibration started — wait for completion")

    def _ol_enter_closed_loop(self):
        if not self._check_connected():
            return
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        for nid in (hip, knee):
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.025)
            self.bus.set_controller_mode(nid, ControlMode.POSITION, InputMode.POS_FILTER)
            time.sleep(0.025)

        self._ol_set_status("Closed Loop", "#00cc44")
        self._append_log(f"[LEG] Closed loop: hip={hip}, knee={knee}")

    def _ol_set_idle(self):
        if not self._check_connected():
            return
        self._ol_stop()
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()
        self.bus.set_axis_state(hip, AxisState.IDLE)
        time.sleep(0.01)
        self.bus.set_axis_state(knee, AxisState.IDLE)
        self._ol_set_status("Idle", "#888888")
        self._append_log("[LEG] Both axes set to IDLE")

    def _ol_clear_errors(self):
        if not self._check_connected():
            return
        self.bus.clear_errors(self._ol_hip_id())
        time.sleep(0.01)
        self.bus.clear_errors(self._ol_knee_id())
        self._append_log("[LEG] Errors cleared")

    def _ol_reboot(self):
        if not self._check_connected():
            return
        self.bus.reboot(self._ol_hip_id())
        time.sleep(0.05)
        self.bus.reboot(self._ol_knee_id())
        self._ol_set_status("Rebooting...", "#922b21")
        self._append_log("[LEG] Reboot sent")

    # ── Offset ───────────────────────────────────────────────────────────

    def _ol_save_offset(self):
        """Save current motor positions as zero offsets (leg fully extended)."""
        if not self._check_connected():
            return
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        self.bus.request_encoder_estimates(hip)
        time.sleep(0.05)
        self.bus.request_encoder_estimates(knee)
        time.sleep(0.05)

        hip_pos = self._ol_get_position(hip)
        knee_pos = self._ol_get_position(knee)

        if not hasattr(self, '_ol_offset'):
            self._ol_offset = [0.0, 0.0]

        self._ol_offset[0] = hip_pos
        self._ol_offset[1] = knee_pos

        self.ol_fb_hip_offset.setText(f"{hip_pos:.3f}")
        self.ol_fb_knee_offset.setText(f"{knee_pos:.3f}")
        self._append_log(f"[LEG] Offsets saved: hip={hip_pos:.3f}, knee={knee_pos:.3f}")

        # Also update the global motor_offsets for the selected axes
        # Map our axis IDs into the global offset array if within range
        hip_axis = hip
        knee_axis = knee
        if hip_axis < 8:
            self._motor_offsets[hip_axis] = hip_pos
        if knee_axis < 8:
            self._motor_offsets[knee_axis] = knee_pos

    def _ol_save_sitting_pose(self):
        if not self._check_connected():
            return
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        self.bus.request_encoder_estimates(hip)
        time.sleep(0.05)
        self.bus.request_encoder_estimates(knee)
        time.sleep(0.05)

        self._ol_sitting_pose[0] = self._ol_get_position(hip)
        self._ol_sitting_pose[1] = self._ol_get_position(knee)

        self.ol_fb_hip_sit.setText(f"{self._ol_sitting_pose[0]:.3f}")
        self.ol_fb_knee_sit.setText(f"{self._ol_sitting_pose[1]:.3f}")
        self._append_log(
            f"[LEG] Sitting pose saved: hip={self._ol_sitting_pose[0]:.3f}, "
            f"knee={self._ol_sitting_pose[1]:.3f}")

    # ── XY Movement ──────────────────────────────────────────────────────

    def _ol_move_xy(self):
        if not self._check_connected():
            return
        x = self.ol_spin_x.value()
        y = self.ol_spin_y.value()
        self._ol_move_leg(x, y)

    def _ol_move_to_preset(self, x: float, y: float):
        if not self._check_connected():
            return
        self.ol_spin_x.setValue(x)
        self.ol_spin_y.setValue(y)
        self._ol_move_leg(x, y)

    def _ol_move_leg(self, x: float, y: float):
        hip_pos = self._ol_ik_angle(0, x, y)
        knee_pos = self._ol_ik_angle(1, x, y)

        if math.isnan(hip_pos) or math.isnan(knee_pos):
            self._append_log(f"[LEG] IK unreachable for ({x:.0f}, {y:.0f})")
            return

        self.bus.set_input_pos(self._ol_hip_id(), hip_pos)
        self.bus.set_input_pos(self._ol_knee_id(), knee_pos)
        self._append_log(f"[LEG] Move to ({x:.0f}, {y:.0f}) → "
                         f"hip={hip_pos:.3f}, knee={knee_pos:.3f}")

    # ── Stand Up ─────────────────────────────────────────────────────────

    def _ol_start_stand(self):
        if not self._check_connected():
            return
        if self._ol_running:
            self._ol_stop()

        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        # Enter closed loop + pos filter
        for nid in (hip, knee):
            self.bus.set_controller_mode(nid, ControlMode.POSITION, InputMode.POS_FILTER)
            time.sleep(0.005)
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)

        # Request and capture start positions
        self.bus.request_encoder_estimates(hip)
        time.sleep(0.05)
        self.bus.request_encoder_estimates(knee)
        time.sleep(0.05)

        self._ol_start_pose[0] = self._ol_get_position(hip)
        self._ol_start_pose[1] = self._ol_get_position(knee)

        # Save current as sitting pose
        self._ol_sitting_pose = list(self._ol_start_pose)
        self.ol_fb_hip_sit.setText(f"{self._ol_sitting_pose[0]:.3f}")
        self.ol_fb_knee_sit.setText(f"{self._ol_sitting_pose[1]:.3f}")

        # Compute IK targets
        stand_x = self.ol_spin_stand_x.value()
        stand_y = self.ol_spin_stand_y.value()
        self._ol_stand_target = [
            self._ol_ik_angle(0, stand_x, stand_y),
            self._ol_ik_angle(1, stand_x, stand_y),
        ]
        for i, t in enumerate(self._ol_stand_target):
            if math.isnan(t):
                self._append_log(f"[LEG] Stand target unreachable for joint {i}")
                return

        self._ol_running = True
        self._ol_mode = 'stand'
        self._ol_start_time = time.monotonic()
        self._ol_stand_duration = self.ol_spin_stand_dur.value()
        self._ol_set_status("Standing Up", "#3399ff")
        self._append_log("[LEG] Stand up sequence started")
        self._ol_timer.start(5)

    def _ol_tick_stand(self):
        elapsed = time.monotonic() - self._ol_start_time
        duration = self._ol_stand_duration

        if elapsed >= duration:
            # Final position
            self.bus.set_input_pos(self._ol_hip_id(), self._ol_stand_target[0])
            self.bus.set_input_pos(self._ol_knee_id(), self._ol_stand_target[1])
            self._ol_stop()
            self._ol_set_status("Standing", "#00cc44")
            self._append_log("[LEG] Standing complete")
            return

        progress = elapsed / duration
        s = progress * progress * (3.0 - 2.0 * progress)  # smoothstep

        for j, nid in enumerate((self._ol_hip_id(), self._ol_knee_id())):
            pos = self._ol_start_pose[j] + (self._ol_stand_target[j] - self._ol_start_pose[j]) * s
            self.bus.set_input_pos(nid, pos)

    # ── Sit Down ─────────────────────────────────────────────────────────

    def _ol_start_sit(self):
        if not self._check_connected():
            return
        if self._ol_running:
            self._ol_stop()

        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        for nid in (hip, knee):
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)
            self.bus.set_controller_mode(nid, ControlMode.POSITION, InputMode.POS_FILTER)
            time.sleep(0.005)

        self.bus.request_encoder_estimates(hip)
        time.sleep(0.05)
        self.bus.request_encoder_estimates(knee)
        time.sleep(0.05)

        self._ol_start_pose[0] = self._ol_get_position(hip)
        self._ol_start_pose[1] = self._ol_get_position(knee)

        self._ol_running = True
        self._ol_mode = 'sit'
        self._ol_start_time = time.monotonic()
        self._ol_sit_duration = self.ol_spin_stand_dur.value()
        self._ol_set_status("Sitting Down", "#b7950b")
        self._append_log("[LEG] Sit down sequence started")
        self._ol_timer.start(10)

    def _ol_tick_sit(self):
        elapsed = time.monotonic() - self._ol_start_time
        duration = self._ol_sit_duration

        if elapsed >= duration:
            self.bus.set_axis_state(self._ol_hip_id(), AxisState.IDLE)
            time.sleep(0.01)
            self.bus.set_axis_state(self._ol_knee_id(), AxisState.IDLE)
            self._ol_stop()
            self._ol_set_status("Sitting", "#ffcc00")
            self._append_log("[LEG] Sit down complete")
            return

        progress = elapsed / duration
        s = progress * progress * (3.0 - 2.0 * progress)

        for j, nid in enumerate((self._ol_hip_id(), self._ol_knee_id())):
            target = self._ol_sitting_pose[j]
            pos = self._ol_start_pose[j] + (target - self._ol_start_pose[j]) * s
            self.bus.set_input_pos(nid, pos)

    # ── Gait ─────────────────────────────────────────────────────────────

    def _ol_start_gait(self):
        if not self._check_connected():
            return
        if self._ol_running:
            self._ol_stop()

        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        for nid in (hip, knee):
            self.bus.set_controller_mode(nid, ControlMode.POSITION, InputMode.POS_FILTER)
            time.sleep(0.005)

        # Move to start position (ground phase start)
        stand_h = self.ol_spin_gait_h.value()
        step_len = self.ol_spin_step_len.value()
        self._ol_move_leg(stand_h, -step_len)
        time.sleep(0.5)

        self._ol_running = True
        self._ol_mode = 'gait'
        self._ol_start_time = time.monotonic()
        self._ol_set_status("Gait Running", "#00cc44")
        self._append_log("[LEG] Single leg gait started")
        self._ol_timer.start(5)

    def _ol_tick_gait(self):
        step_len = self.ol_spin_step_len.value()
        stand_h = self.ol_spin_gait_h.value()
        lift_h = self.ol_spin_gait_lift.value()
        cycle_s = self.ol_spin_gait_cycle.value() / 1000.0

        elapsed = time.monotonic() - self._ol_start_time
        phase = (elapsed % cycle_s) / cycle_s

        if phase < 0.5:
            # Ground phase: foot slides backward
            prog = phase / 0.5
            x = stand_h
            y = -step_len + (2 * step_len * prog)
        else:
            # Swing phase: foot lifts and swings forward
            prog = (phase - 0.5) / 0.5
            x = stand_h - (lift_h * math.sin(math.pi * prog))
            y = step_len - (2 * step_len * prog)

        hip_pos = self._ol_ik_angle(0, x, y)
        knee_pos = self._ol_ik_angle(1, x, y)
        if not math.isnan(hip_pos) and not math.isnan(knee_pos):
            self.bus.set_input_pos(self._ol_hip_id(), hip_pos)
            self.bus.set_input_pos(self._ol_knee_id(), knee_pos)

    # ── Timer dispatch & stop ────────────────────────────────────────────

    def _ol_stop(self):
        if self._ol_running:
            self._ol_timer.stop()
            self._ol_running = False
            self._ol_mode = None
            self._ol_set_status("Stopped", "#ffcc00")
            self._append_log("[LEG] Stopped")

    def _ol_tick(self):
        if not self._ol_running or not self.bus.is_connected:
            self._ol_stop()
            return
        if self._ol_mode == 'stand':
            self._ol_tick_stand()
        elif self._ol_mode == 'sit':
            self._ol_tick_sit()
        elif self._ol_mode == 'gait':
            self._ol_tick_gait()

    # ── Visualization ────────────────────────────────────────────────────

    def _ol_toggle_viz(self, checked: bool):
        self._ol_viz_timer.stop()

    def _ol_update_viz(self):
        # Visualization is intentionally disabled to avoid high-frequency UI work.
        if (not getattr(self, '_leg_visualization_enabled', True)
                or not hasattr(self, 'ol_viz_upper')):
            return
        hip = self._ol_hip_id()
        knee = self._ol_knee_id()

        # Request fresh encoder data
        if self.bus.is_connected:
            self.bus.request_encoder_estimates(hip)
            self.bus.request_encoder_estimates(knee)
            self.bus.request_iq(hip)
            self.bus.request_iq(knee)

        hip_pos = self._ol_get_position(hip)
        knee_pos = self._ol_get_position(knee)

        # Update feedback fields
        hip_fb = self.bus.get_feedback(hip)
        knee_fb = self.bus.get_feedback(knee)
        if hip_fb:
            self.ol_fb_hip_pos.setText(f"{hip_fb.pos_estimate:.3f}")
            self.ol_fb_hip_vel.setText(f"{hip_fb.vel_estimate:.3f}")
            self.ol_fb_hip_iq.setText(f"{hip_fb.iq_measured:.3f}")
            state = hip_fb.axis_state
            from steadywin_can import AXIS_STATE_NAMES
            self.ol_fb_hip_state.setText(AXIS_STATE_NAMES.get(state, f"?{state}"))
        if knee_fb:
            self.ol_fb_knee_pos.setText(f"{knee_fb.pos_estimate:.3f}")
            self.ol_fb_knee_vel.setText(f"{knee_fb.vel_estimate:.3f}")
            self.ol_fb_knee_iq.setText(f"{knee_fb.iq_measured:.3f}")
            state = knee_fb.axis_state
            from steadywin_can import AXIS_STATE_NAMES
            self.ol_fb_knee_state.setText(AXIS_STATE_NAMES.get(state, f"?{state}"))

        # Forward kinematics for visualization
        if not hasattr(self, '_ol_offset'):
            self._ol_offset = [0.0, 0.0]

        shoulder, elbow, foot = self._ol_fk(hip_pos, knee_pos)

        # Update plot — note: the visualization convention is
        # X = forward (horizontal), Y = vertical (down is negative)
        self.ol_viz_upper.setData(
            [shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        self.ol_viz_lower.setData(
            [elbow[0], foot[0]], [elbow[1], foot[1]])
        self.ol_viz_foot.setData([foot[0]], [foot[1]])

        self.ol_lbl_foot_x.setText(f"{foot[0]:.1f}")
        self.ol_lbl_foot_y.setText(f"{foot[1]:.1f}")

        # Update reach circle based on current L1+L2
        L_max = self._robot_L1 + self._robot_L2
        theta_arr = [i * 2 * math.pi / 100 for i in range(101)]
        self.ol_viz_reach.setData(
            [L_max * math.cos(t) for t in theta_arr],
            [L_max * math.sin(t) for t in theta_arr])
