"""
Locomotion tab — IK, trot gait, stand up, sit down sequences.
Used as a mixin for MainWindow.
"""

import math
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QSpinBox, QLabel, QScrollArea, QComboBox, QStackedWidget,
)

from helpers import (
    LEDIndicator, make_label, make_readonly_line, make_double_spin,
)
from steadywin_can import AxisState, ControlMode, InputMode


class LocomotionMixin:
    """Mixin providing the Locomotion tab, IK, and gait/stand/sit logic."""

    def _init_locomotion_state(self):
        """Call from MainWindow.__init__ to set up locomotion state."""
        self._loco_running = False
        self._loco_mode = None  # 'trot', 'trot_align', 'trot_finish', 'stand', 'sit'
        self._loco_timer = QTimer()
        self._loco_timer.timeout.connect(self._loco_tick)
        self._loco_start_time = 0.0
        self._loco_start_pose = [0.0] * 8
        self._sitting_pose = [0.0] * 8
        self._loco_grounded = [False] * 8
        self._loco_step = 0
        self._loco_trot_requested_input_mode = InputMode.POS_FILTER
        self._loco_trot_stand_targets = [0.0] * 8
        self._loco_trot_align_deadline = 0.0
        self._loco_stand_dbg_last_log = 0.0

        # Per-pair phase tracking for the trot gait.
        # Pair A = FL+RR (axes 0,1,6,7), Pair B = FR+RL (axes 2,3,4,5).
        # Phase 0→0.5 = stance (linear ground push), 0.5→1.0 = swing (lift & return).
        # Starting at 0.25 / 0.75 puts each pair mid-phase at gait launch,
        # matching the stand pose (y=0) and avoiding any initial position jump.
        self._trot_pair_a_phase: float = 0.25
        self._trot_pair_b_phase: float = 0.75
        self._trot_pair_a_held: bool = False   # True while waiting at stance end
        self._trot_pair_b_held: bool = False
        self._trot_hold_deadline_a: float = 0.0
        self._trot_hold_deadline_b: float = 0.0
        self._trot_prev_tick_time: float = 0.0
        self._trot_last_enc_req: float = 0.0   # rate-limit encoder requests

    def _build_locomotion_tab(self) -> QWidget:
        widget = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        layout = QVBoxLayout(inner)

        # ── Status ──
        status_grp = QGroupBox("Locomotion Status")
        status_lay = QHBoxLayout(status_grp)
        self.loco_status_led = LEDIndicator(14)
        status_lay.addWidget(self.loco_status_led)
        self.lbl_loco_status = QLabel("Idle")
        self.lbl_loco_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        status_lay.addWidget(self.lbl_loco_status)

        btn_estop_loco = QPushButton("E-STOP ALL")
        btn_estop_loco.setStyleSheet(
            "background: #922b21; font-weight: bold; padding: 8px; min-width: 100px;")
        btn_estop_loco.clicked.connect(self._loco_estop)
        status_lay.addWidget(btn_estop_loco)

        btn_all_idle = QPushButton("All → IDLE")
        btn_all_idle.clicked.connect(lambda: self._loco_stop_and_idle())
        status_lay.addWidget(btn_all_idle)

        status_lay.addStretch()
        layout.addWidget(status_grp)

        # ── Gait Parameters ──
        gait_grp = QGroupBox("Trot Gait")
        gait_lay = QGridLayout(gait_grp)

        row = 0
        gait_lay.addWidget(make_label("Step Length (mm):"), row, 0)
        self.spin_step_len = make_double_spin(10, 200, 1, 90.0, 5.0)
        gait_lay.addWidget(self.spin_step_len, row, 1)

        gait_lay.addWidget(make_label("Stand Height (mm):"), row, 2)
        self.spin_stand_h = make_double_spin(50, 300, 1, 220.0, 5.0)
        gait_lay.addWidget(self.spin_stand_h, row, 3)

        gait_lay.addWidget(make_label("Lift Height (mm):"), row, 4)
        self.spin_lift_h = make_double_spin(10, 200, 1, 100.0, 5.0)
        gait_lay.addWidget(self.spin_lift_h, row, 5)

        row = 1
        gait_lay.addWidget(make_label("Cycle Time (ms):"), row, 0)
        self.spin_cycle_time = QSpinBox()
        self.spin_cycle_time.setRange(200, 5000)
        self.spin_cycle_time.setValue(700)
        self.spin_cycle_time.setSingleStep(50)
        gait_lay.addWidget(self.spin_cycle_time, row, 1)

        gait_lay.addWidget(make_label("Tick Interval (ms):"), row, 2)
        self.spin_tick_interval = QSpinBox()
        self.spin_tick_interval.setRange(2, 50)
        self.spin_tick_interval.setValue(5)
        gait_lay.addWidget(self.spin_tick_interval, row, 3)

        row = 2
        gait_lay.addWidget(make_label("Input Mode:"), row, 0)
        self.cmb_loco_input_mode = QComboBox()
        self.cmb_loco_input_mode.addItem("Passthrough", InputMode.PASSTHROUGH)
        self.cmb_loco_input_mode.addItem("Pos Filter", InputMode.POS_FILTER)
        self.cmb_loco_input_mode.addItem("Trap Traj", InputMode.TRAP_TRAJ)
        self.cmb_loco_input_mode.setCurrentIndex(1)
        self.cmb_loco_input_mode.currentIndexChanged.connect(
            self._loco_on_input_mode_changed)
        gait_lay.addWidget(self.cmb_loco_input_mode, row, 1)

        self._loco_mode_cfg_stack = QStackedWidget()

        cfg_passthrough = QWidget()
        cfg_passthrough_lay = QHBoxLayout(cfg_passthrough)
        cfg_passthrough_lay.setContentsMargins(0, 0, 0, 0)
        cfg_passthrough_lay.addWidget(
            make_label("Passthrough: commands are applied directly."))
        cfg_passthrough_lay.addStretch()
        self._loco_mode_cfg_stack.addWidget(cfg_passthrough)

        cfg_pos_filter = QWidget()
        cfg_pf_lay = QGridLayout(cfg_pos_filter)
        cfg_pf_lay.setContentsMargins(0, 0, 0, 0)
        cfg_pf_lay.addWidget(make_label("Velocity Limit (rev/s):"), 0, 0)
        self.spin_loco_vel_limit = make_double_spin(0.1, 100.0, 2, 20.0, 1.0)
        cfg_pf_lay.addWidget(self.spin_loco_vel_limit, 0, 1)
        cfg_pf_lay.addWidget(make_label("Current Limit (A):"), 0, 2)
        self.spin_loco_current_limit = make_double_spin(0.5, 100.0, 2, 25.0, 1.0)
        cfg_pf_lay.addWidget(self.spin_loco_current_limit, 0, 3)
        self._loco_mode_cfg_stack.addWidget(cfg_pos_filter)

        cfg_trap = QWidget()
        cfg_trap_lay = QGridLayout(cfg_trap)
        cfg_trap_lay.setContentsMargins(0, 0, 0, 0)
        cfg_trap_lay.addWidget(make_label("Velocity Limit (rev/s):"), 0, 0)
        self.spin_loco_traj_vel = make_double_spin(0.1, 100.0, 2, 20.0, 1.0)
        cfg_trap_lay.addWidget(self.spin_loco_traj_vel, 0, 1)
        cfg_trap_lay.addWidget(make_label("Accel (rev/s^2):"), 0, 2)
        self.spin_loco_traj_accel = make_double_spin(0.1, 200.0, 2, 30.0, 1.0)
        cfg_trap_lay.addWidget(self.spin_loco_traj_accel, 0, 3)
        cfg_trap_lay.addWidget(make_label("Decel (rev/s^2):"), 1, 0)
        self.spin_loco_traj_decel = make_double_spin(0.1, 200.0, 2, 30.0, 1.0)
        cfg_trap_lay.addWidget(self.spin_loco_traj_decel, 1, 1)
        cfg_trap_lay.addWidget(make_label("Inertia:"), 1, 2)
        self.spin_loco_traj_inertia = make_double_spin(0.0, 5.0, 3, 0.0, 0.01)
        cfg_trap_lay.addWidget(self.spin_loco_traj_inertia, 1, 3)
        self._loco_mode_cfg_stack.addWidget(cfg_trap)

        gait_lay.addWidget(self._loco_mode_cfg_stack, row, 2, 1, 4)

        row = 3
        gait_lay.addWidget(make_label("Stance Overshoot (%):"), row, 0)
        self.spin_trot_overshoot = make_double_spin(0.0, 100.0, 1, 25.0, 5.0)
        self.spin_trot_overshoot.setToolTip(
            "Extra distance (% of step length) commanded beyond the stance endpoint.\n"
            "Keeps the motor error—and thus torque—high throughout the stride,\n"
            "ensuring the leg actually travels the full step distance.")
        gait_lay.addWidget(self.spin_trot_overshoot, row, 1)

        gait_lay.addWidget(make_label("Hold Timeout (s):"), row, 2)
        self.spin_trot_hold_timeout = make_double_spin(0.05, 2.0, 2, 0.30, 0.05)
        self.spin_trot_hold_timeout.setToolTip(
            "Maximum time to wait for a leg pair to reach the stance endpoint\n"
            "before continuing the gait anyway (logged as a timeout warning).")
        gait_lay.addWidget(self.spin_trot_hold_timeout, row, 3)

        gait_lay.addWidget(make_label("Pos Tolerance (rev):"), row, 4)
        self.spin_trot_pos_tol = make_double_spin(0.05, 1.0, 2, 0.15, 0.05)
        self.spin_trot_pos_tol.setToolTip(
            "Acceptable position error (motor revolutions) for the stance-end\n"
            "hold check. Smaller = stricter; larger = releases sooner.")
        gait_lay.addWidget(self.spin_trot_pos_tol, row, 5)

        btn_trot_start = QPushButton("▶ Start Trot")
        btn_trot_start.setStyleSheet("background: #1e8449; font-weight: bold; padding: 8px;")
        btn_trot_start.clicked.connect(self._loco_start_trot)
        gait_lay.addWidget(btn_trot_start, row + 1, 0, 1, 2)

        btn_trot_stop = QPushButton("■ Stop Trot")
        btn_trot_stop.setStyleSheet("background: #b03a2e; padding: 8px;")
        btn_trot_stop.clicked.connect(self._loco_stop_trot)
        gait_lay.addWidget(btn_trot_stop, row + 1, 2, 1, 2)

        # Label columns stay at content width; spinbox columns absorb extra space.
        # This keeps each label visually tight against its value box.
        for _col in (0, 2, 4):
            gait_lay.setColumnStretch(_col, 0)
        for _col in (1, 3, 5):
            gait_lay.setColumnStretch(_col, 1)

        self._loco_on_input_mode_changed()

        layout.addWidget(gait_grp)

        # ── Stand / Sit Parameters ──
        standsit_grp = QGroupBox("Stand Up / Sit Down")
        ss_lay = QGridLayout(standsit_grp)

        row = 0
        ss_lay.addWidget(make_label("Stand X (mm):"), row, 0)
        self.spin_stand_x = make_double_spin(50, 300, 1, 200.0, 5.0)
        ss_lay.addWidget(self.spin_stand_x, row, 1)

        ss_lay.addWidget(make_label("Stand Y (mm):"), row, 2)
        self.spin_stand_y = make_double_spin(-150, 150, 1, 0.0, 5.0)
        ss_lay.addWidget(self.spin_stand_y, row, 3)

        row = 1
        ss_lay.addWidget(make_label("Stand Duration (s):"), row, 0)
        self.spin_stand_dur = make_double_spin(0.5, 10.0, 1, 2.0, 0.5)
        ss_lay.addWidget(self.spin_stand_dur, row, 1)

        ss_lay.addWidget(make_label("Sit Duration (s):"), row, 2)
        self.spin_sit_dur = make_double_spin(0.5, 10.0, 1, 2.5, 0.5)
        ss_lay.addWidget(self.spin_sit_dur, row, 3)

        row = 2
        ss_lay.addWidget(make_label("Sit Current Thresh (A):"), row, 0)
        self.spin_sit_iq_thresh = make_double_spin(0.1, 20.0, 2, 2.0, 0.1)
        ss_lay.addWidget(self.spin_sit_iq_thresh, row, 1)

        ss_lay.addWidget(make_label("Sit Pos Tolerance (rev):"), row, 2)
        self.spin_sit_pos_tol = make_double_spin(0.1, 10.0, 1, 2.0, 0.5)
        ss_lay.addWidget(self.spin_sit_pos_tol, row, 3)

        row = 3
        btn_stand = QPushButton("▲ Stand Up")
        btn_stand.setStyleSheet("background: #1e8449; font-weight: bold; padding: 8px;")
        btn_stand.clicked.connect(self._loco_start_stand)
        ss_lay.addWidget(btn_stand, row, 0, 1, 2)

        btn_sit = QPushButton("▼ Sit Down")
        btn_sit.setStyleSheet("background: #b7950b; font-weight: bold; padding: 8px;")
        btn_sit.clicked.connect(self._loco_start_sit)
        ss_lay.addWidget(btn_sit, row, 2, 1, 2)

        layout.addWidget(standsit_grp)

        # ── Sitting Pose ──
        pose_grp = QGroupBox("Sitting Pose")
        pose_lay = QGridLayout(pose_grp)

        self.sitting_pose_fields = []
        motor_names = [
            "FL-Hip(0)", "FL-Knee(1)", "FR-Hip(2)", "FR-Knee(3)",
            "RL-Hip(4)", "RL-Knee(5)", "RR-Hip(6)", "RR-Knee(7)",
        ]
        for i, name in enumerate(motor_names):
            r, c = divmod(i, 4)
            pose_lay.addWidget(make_label(name), r * 2, c)
            le = make_readonly_line(100)
            le.setText("0.000")
            pose_lay.addWidget(le, r * 2 + 1, c)
            self.sitting_pose_fields.append(le)

        pose_btn_lay = QHBoxLayout()
        btn_save_pose = QPushButton("Save Current as Sitting Pose")
        btn_save_pose.clicked.connect(self._loco_save_sitting_pose)
        pose_btn_lay.addWidget(btn_save_pose)

        btn_clear_pose = QPushButton("Clear Sitting Pose")
        btn_clear_pose.clicked.connect(self._loco_clear_sitting_pose)
        pose_btn_lay.addWidget(btn_clear_pose)
        pose_btn_lay.addStretch()

        pose_lay.addLayout(pose_btn_lay, 4, 0, 1, 4)
        layout.addWidget(pose_grp)

        # ── Manual XY Move ──
        xy_grp = QGroupBox("Manual XY Move (All Legs)")
        xy_lay = QGridLayout(xy_grp)

        xy_lay.addWidget(make_label("X (mm):"), 0, 0)
        self.spin_xy_x = make_double_spin(0, 300, 1, 200.0, 5.0)
        xy_lay.addWidget(self.spin_xy_x, 0, 1)

        xy_lay.addWidget(make_label("Y (mm):"), 0, 2)
        self.spin_xy_y = make_double_spin(-200, 200, 1, 0.0, 5.0)
        xy_lay.addWidget(self.spin_xy_y, 0, 3)

        xy_lay.addWidget(make_label("Input Mode:"), 1, 0)
        self.cmb_xy_input_mode = QComboBox()
        self.cmb_xy_input_mode.addItem("Passthrough", InputMode.PASSTHROUGH)
        self.cmb_xy_input_mode.addItem("Pos Filter", InputMode.POS_FILTER)
        self.cmb_xy_input_mode.addItem("Trap Traj", InputMode.TRAP_TRAJ)
        self.cmb_xy_input_mode.setCurrentIndex(1)
        self.cmb_xy_input_mode.currentIndexChanged.connect(self._loco_on_xy_input_mode_changed)
        xy_lay.addWidget(self.cmb_xy_input_mode, 1, 1)

        self._xy_mode_cfg_stack = QStackedWidget()

        xy_cfg_passthrough = QWidget()
        xy_cfg_passthrough_lay = QHBoxLayout(xy_cfg_passthrough)
        xy_cfg_passthrough_lay.setContentsMargins(0, 0, 0, 0)
        xy_cfg_passthrough_lay.addWidget(make_label("Passthrough: direct target updates."))
        xy_cfg_passthrough_lay.addStretch()
        self._xy_mode_cfg_stack.addWidget(xy_cfg_passthrough)

        xy_cfg_pos_filter = QWidget()
        xy_cfg_pf_lay = QGridLayout(xy_cfg_pos_filter)
        xy_cfg_pf_lay.setContentsMargins(0, 0, 0, 0)
        xy_cfg_pf_lay.addWidget(make_label("Velocity Limit (rev/s):"), 0, 0)
        self.spin_xy_vel_limit = make_double_spin(0.1, 100.0, 2, 20.0, 1.0)
        xy_cfg_pf_lay.addWidget(self.spin_xy_vel_limit, 0, 1)
        xy_cfg_pf_lay.addWidget(make_label("Current Limit (A):"), 0, 2)
        self.spin_xy_current_limit = make_double_spin(0.5, 100.0, 2, 25.0, 1.0)
        xy_cfg_pf_lay.addWidget(self.spin_xy_current_limit, 0, 3)
        self._xy_mode_cfg_stack.addWidget(xy_cfg_pos_filter)

        xy_cfg_trap = QWidget()
        xy_cfg_trap_lay = QGridLayout(xy_cfg_trap)
        xy_cfg_trap_lay.setContentsMargins(0, 0, 0, 0)
        xy_cfg_trap_lay.addWidget(make_label("Velocity Limit (rev/s):"), 0, 0)
        self.spin_xy_traj_vel = make_double_spin(0.1, 100.0, 2, 20.0, 1.0)
        xy_cfg_trap_lay.addWidget(self.spin_xy_traj_vel, 0, 1)
        xy_cfg_trap_lay.addWidget(make_label("Accel (rev/s^2):"), 0, 2)
        self.spin_xy_traj_accel = make_double_spin(0.1, 200.0, 2, 30.0, 1.0)
        xy_cfg_trap_lay.addWidget(self.spin_xy_traj_accel, 0, 3)
        xy_cfg_trap_lay.addWidget(make_label("Decel (rev/s^2):"), 1, 0)
        self.spin_xy_traj_decel = make_double_spin(0.1, 200.0, 2, 30.0, 1.0)
        xy_cfg_trap_lay.addWidget(self.spin_xy_traj_decel, 1, 1)
        xy_cfg_trap_lay.addWidget(make_label("Inertia:"), 1, 2)
        self.spin_xy_traj_inertia = make_double_spin(0.0, 5.0, 3, 0.0, 0.01)
        xy_cfg_trap_lay.addWidget(self.spin_xy_traj_inertia, 1, 3)
        self._xy_mode_cfg_stack.addWidget(xy_cfg_trap)

        xy_lay.addWidget(self._xy_mode_cfg_stack, 1, 2, 1, 2)

        btn_xy_go = QPushButton("Move All Legs")
        btn_xy_go.clicked.connect(self._loco_move_xy)
        xy_lay.addWidget(btn_xy_go, 0, 4, 2, 1)
        xy_lay.setColumnStretch(5, 1)
        self._loco_on_xy_input_mode_changed()
        layout.addWidget(xy_grp)

        layout.addStretch()
        scroll.setWidget(inner)

        outer = QVBoxLayout(widget)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)
        return widget

    # ── IK (ported from Arduino getAngle) ────────────────────────────────

    def _ik_get_angle(self, axis: int, x: float, y: float) -> float:
        """Inverse kinematics: (x, y) in mm → motor turns for given axis."""
        L1 = self._robot_L1
        L2 = self._robot_L2
        GR = self._robot_GR

        r = math.sqrt(x * x + y * y)
        if r > (L1 + L2) or r < L1 / 2:
            return float('nan')

        if axis % 2 == 0:  # hip
            lam = math.atan2(y, x)
            theta = math.acos(
                max(-1.0, min(1.0, (r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))))
            temp = -(lam + theta)
        else:  # knee
            alpha = math.acos(
                max(-1.0, min(1.0, (L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))))
            temp = -(math.pi - alpha)

        base_turns = -((temp * 180.0 / math.pi) / 360.0 * GR)
        final_turns = (self._motor_offsets[axis]
                       + self._motor_directions[axis] * base_turns)
        return final_turns

    def _loco_set_position(self, axis: int, pos: float):
        nid = self._loco_get_node_id(axis)
        self.bus.set_input_pos(nid, pos)

    def _loco_get_node_id(self, axis: int) -> int:
        leg_idx = axis // 2
        joint_idx = axis % 2
        spin = self.node_id_spins.get((leg_idx, joint_idx))
        return spin.value() if spin else axis

    def _loco_get_position(self, axis: int) -> float:
        nid = self._loco_get_node_id(axis)
        fb = self.bus.get_feedback(nid)
        return fb.pos_estimate if fb else 0.0

    def _loco_get_iq(self, axis: int) -> float:
        nid = self._loco_get_node_id(axis)
        fb = self.bus.get_feedback(nid)
        return fb.iq_measured if fb else 0.0

    def _loco_move_xy(self):
        if not self._check_connected():
            return

        input_mode = self._loco_selected_xy_input_mode()
        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)
            self._loco_apply_xy_input_mode(nid, input_mode)

        x = self.spin_xy_x.value()
        y = self.spin_xy_y.value()
        for axis in range(8):
            pos = self._ik_get_angle(axis, x, y)
            if math.isnan(pos):
                self._append_log(f"[LOCO] IK unreachable for axis {axis}")
                return
            self._loco_set_position(axis, pos)

        self._append_log(f"[LOCO] Manual XY move sent (InputMode={InputMode(input_mode).name})")

    def _loco_move_leg(self, hip_id: int, knee_id: int, x: float, y: float):
        hip_pos = self._ik_get_angle(hip_id, x, y)
        knee_pos = self._ik_get_angle(knee_id, x, y)
        if math.isnan(hip_pos) or math.isnan(knee_pos):
            return
        self._loco_set_position(hip_id, hip_pos)
        self._loco_set_position(knee_id, knee_pos)

    # ── Locomotion controls ──────────────────────────────────────────────

    def _loco_save_sitting_pose(self):
        if not self._check_connected():
            return
        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.request_encoder_estimates(nid)
        time.sleep(0.1)
        for axis in range(8):
            self._sitting_pose[axis] = self._loco_get_position(axis)
            self.sitting_pose_fields[axis].setText(
                f"{self._sitting_pose[axis]:.3f}")
        self._append_log("[LOCO] Sitting pose saved")

    def _loco_clear_sitting_pose(self):
        self._sitting_pose = [0.0] * 8
        for field in self.sitting_pose_fields:
            field.setText("0.000")
        self._append_log("[LOCO] Sitting pose cleared")

    def _loco_estop(self):
        self._loco_stop()
        if self.bus.is_connected:
            for nid in self._get_all_node_ids():
                self.bus.estop(nid)

    def _loco_stop_and_idle(self):
        self._loco_stop()
        if self.bus.is_connected:
            for nid in self._get_all_node_ids():
                self.bus.set_axis_state(nid, AxisState.IDLE)
                time.sleep(0.01)

    def _loco_stop(self):
        if self._loco_running:
            self._loco_timer.stop()
            self._loco_running = False
            self._loco_mode = None
            self.lbl_loco_status.setText("Stopped")
            self.loco_status_led.set_color("#ffcc00")
            self._append_log("[LOCO] Stopped")
            if getattr(self, '_run_logging', False):
                self._run_logging = False
                self._save_run_log()

    def _loco_set_status(self, text: str, color: str = "#3399ff"):
        self.lbl_loco_status.setText(text)
        self.loco_status_led.set_color(color)

    def _loco_debug(self, text: str):
        print(text, flush=True)

    def _loco_debug_stand_setpos(self, axis: int, pos: float, phase: str):
        # Keep all call sites, but print commands for one leg only (FL: axes 0,1).
        if axis in (0, 1):
            nid = self._loco_get_node_id(axis)
            self._loco_debug(
                f"[LOCO][STAND][DBG] cmd {phase}: set_input_pos(nid={nid}, axis={axis}, pos={pos:.4f})"
            )

    def _loco_on_input_mode_changed(self):
        mode = self._loco_selected_input_mode()
        if mode == InputMode.PASSTHROUGH:
            self._loco_mode_cfg_stack.setCurrentIndex(0)
        elif mode == InputMode.POS_FILTER:
            self._loco_mode_cfg_stack.setCurrentIndex(1)
        else:
            self._loco_mode_cfg_stack.setCurrentIndex(2)

    def _loco_selected_input_mode(self) -> int:
        return int(self.cmb_loco_input_mode.currentData())

    def _loco_apply_input_mode(self, nid: int, input_mode: int):
        self.bus.set_controller_mode(nid, ControlMode.POSITION, input_mode)
        time.sleep(0.005)

        if input_mode == InputMode.POS_FILTER:
            self.bus.set_limits(
                nid,
                self.spin_loco_vel_limit.value(),
                self.spin_loco_current_limit.value(),
            )
            time.sleep(0.005)
        elif input_mode == InputMode.TRAP_TRAJ:
            self.bus.set_traj_vel_limit(nid, self.spin_loco_traj_vel.value())
            time.sleep(0.005)
            self.bus.set_traj_accel_limits(
                nid,
                self.spin_loco_traj_accel.value(),
                self.spin_loco_traj_decel.value(),
            )
            time.sleep(0.005)
            self.bus.set_traj_inertia(nid, self.spin_loco_traj_inertia.value())
            time.sleep(0.005)

    def _loco_on_xy_input_mode_changed(self):
        mode = self._loco_selected_xy_input_mode()
        if mode == InputMode.PASSTHROUGH:
            self._xy_mode_cfg_stack.setCurrentIndex(0)
        elif mode == InputMode.POS_FILTER:
            self._xy_mode_cfg_stack.setCurrentIndex(1)
        else:
            self._xy_mode_cfg_stack.setCurrentIndex(2)

    def _loco_selected_xy_input_mode(self) -> int:
        return int(self.cmb_xy_input_mode.currentData())

    def _loco_apply_xy_input_mode(self, nid: int, input_mode: int):
        self.bus.set_controller_mode(nid, ControlMode.POSITION, input_mode)
        time.sleep(0.005)

        if input_mode == InputMode.POS_FILTER:
            self.bus.set_limits(
                nid,
                self.spin_xy_vel_limit.value(),
                self.spin_xy_current_limit.value(),
            )
            time.sleep(0.005)
        elif input_mode == InputMode.TRAP_TRAJ:
            self.bus.set_traj_vel_limit(nid, self.spin_xy_traj_vel.value())
            time.sleep(0.005)
            self.bus.set_traj_accel_limits(
                nid,
                self.spin_xy_traj_accel.value(),
                self.spin_xy_traj_decel.value(),
            )
            time.sleep(0.005)
            self.bus.set_traj_inertia(nid, self.spin_xy_traj_inertia.value())
            time.sleep(0.005)

    def _loco_get_trot_stand_targets(self):
        stand_h = self.spin_stand_h.value()
        targets = [self._ik_get_angle(axis, stand_h, 0.0) for axis in range(8)]
        if any(math.isnan(v) for v in targets):
            return None
        return targets

    def _loco_refresh_encoder_feedback(self):
        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.request_encoder_estimates(nid)
        time.sleep(0.05)

    def _loco_max_target_error(self, targets: list[float]) -> float:
        self._loco_refresh_encoder_feedback()
        max_err = 0.0
        for axis in range(8):
            err = abs(self._loco_get_position(axis) - targets[axis])
            if err > max_err:
                max_err = err
        return max_err

    def _loco_move_all_to_targets_trap(self, targets: list[float]):
        vel = self.spin_loco_traj_vel.value()
        accel = self.spin_loco_traj_accel.value()
        decel = self.spin_loco_traj_decel.value()
        inertia = self.spin_loco_traj_inertia.value()

        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)
            self.bus.set_controller_mode(nid, ControlMode.POSITION, InputMode.TRAP_TRAJ)
            time.sleep(0.005)
            self.bus.set_traj_vel_limit(nid, vel)
            time.sleep(0.005)
            self.bus.set_traj_accel_limits(nid, accel, decel)
            time.sleep(0.005)
            self.bus.set_traj_inertia(nid, inertia)
            time.sleep(0.005)
            self.bus.set_input_pos(nid, targets[axis])
            time.sleep(0.005)

    def _loco_start_trot_streaming(self, input_mode: int):
        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self._loco_apply_input_mode(nid, input_mode)

        # Reset per-pair phase tracking
        self._trot_pair_a_phase = 0.25
        self._trot_pair_b_phase = 0.75
        self._trot_pair_a_held = False
        self._trot_pair_b_held = False
        self._trot_hold_deadline_a = 0.0
        self._trot_hold_deadline_b = 0.0
        self._trot_prev_tick_time = 0.0
        self._trot_last_enc_req = 0.0

        # Start run log — every _append_log call is captured until trot stops.
        import datetime as _dt
        self._run_logging = True
        self._run_log = []
        self._run_log.append(
            f"=== Trot run started: {_dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
        self._run_log.append(
            f"step={self.spin_step_len.value():.0f}mm  "
            f"stand_h={self.spin_stand_h.value():.0f}mm  "
            f"lift_h={self.spin_lift_h.value():.0f}mm  "
            f"cycle={self.spin_cycle_time.value()}ms  "
            f"overshoot={self.spin_trot_overshoot.value():.0f}%  "
            f"hold_timeout={self.spin_trot_hold_timeout.value():.2f}s  "
            f"pos_tol={self.spin_trot_pos_tol.value():.2f}rev  "
            f"input_mode={InputMode(input_mode).name}")

        self._loco_running = True
        self._loco_mode = 'trot'
        self._loco_start_time = time.monotonic()
        self._loco_set_status("Trot Gait Running", "#00cc44")
        self._append_log(
            f"[LOCO] Trot gait started from stand pose (InputMode={InputMode(input_mode).name})"
        )
        self._loco_timer.start(self.spin_tick_interval.value())

    def _loco_stop_trot(self):
        if not self._loco_running:
            return
        if self._loco_mode != 'trot':
            self._loco_stop()
            return

        targets = self._loco_get_trot_stand_targets()
        if targets is None:
            self._append_log("[LOCO] Stop trot: stand target unreachable; stopping immediately")
            self._loco_stop()
            return

        self._loco_move_all_to_targets_trap(targets)
        self._loco_trot_stand_targets = targets
        self._loco_mode = 'trot_finish'
        self._loco_trot_align_deadline = time.monotonic() + 4.0
        self._loco_set_status("Finishing gait to stand...", "#3399ff")
        self._append_log("[LOCO] Stop requested — returning to stand pose")
        self._loco_timer.start(20)

    def _loco_tick_trot_align(self):
        err = self._loco_max_target_error(self._loco_trot_stand_targets)
        if err <= 0.15:
            self._append_log("[LOCO] Reached stand pose, starting trot")
            self._loco_start_trot_streaming(self._loco_trot_requested_input_mode)
            return
        if time.monotonic() >= self._loco_trot_align_deadline:
            self._append_log(
                f"[LOCO] Stand align timeout (max err={err:.3f} rev), starting trot anyway"
            )
            self._loco_start_trot_streaming(self._loco_trot_requested_input_mode)

    def _loco_tick_trot_finish(self):
        err = self._loco_max_target_error(self._loco_trot_stand_targets)
        if err <= 0.15:
            self._loco_stop()
            self._loco_set_status("Standing", "#00cc44")
            self._append_log("[LOCO] Trot finished at stand pose")
            return
        if time.monotonic() >= self._loco_trot_align_deadline:
            self._loco_stop()
            self._loco_set_status("Standing (timeout)", "#ffcc00")
            self._append_log(
                f"[LOCO] Trot finish timeout (max err={err:.3f} rev); stopped"
            )

    # ── Trot Gait ────────────────────────────────────────────────────────

    def _loco_start_trot(self):
        if not self._check_connected():
            return
        if self._loco_running:
            self._loco_stop()

        input_mode = self._loco_selected_input_mode()
        targets = self._loco_get_trot_stand_targets()
        if targets is None:
            self._append_log("[LOCO] Cannot start trot: stand target unreachable")
            return

        self._loco_trot_requested_input_mode = input_mode
        self._loco_trot_stand_targets = targets

        err = self._loco_max_target_error(targets)
        if err > 0.15:
            self._loco_move_all_to_targets_trap(targets)
            self._loco_running = True
            self._loco_mode = 'trot_align'
            self._loco_trot_align_deadline = time.monotonic() + 4.0
            self._loco_set_status("Aligning to stand before trot...", "#3399ff")
            self._append_log(
                f"[LOCO] Pre-align to stand (max err={err:.3f} rev), then start trot"
            )
            self._loco_timer.start(20)
            return

        self._loco_start_trot_streaming(input_mode)

    def _loco_move_xy_internal(self, x: float, y: float):
        for axis in range(8):
            pos = self._ik_get_angle(axis, x, y)
            if not math.isnan(pos):
                self._loco_set_position(axis, pos)

    def _loco_tick_trot(self):
        now = time.monotonic()

        # First tick: just record the time reference and return.
        if self._trot_prev_tick_time == 0.0:
            self._trot_prev_tick_time = now
            return

        dt = now - self._trot_prev_tick_time
        self._trot_prev_tick_time = now

        step_len = self.spin_step_len.value()
        stand_h = self.spin_stand_h.value()
        lift_h = self.spin_lift_h.value()
        cycle_time_s = self.spin_cycle_time.value() / 1000.0
        overshoot_mm = step_len * (self.spin_trot_overshoot.value() / 100.0)
        hold_timeout = self.spin_trot_hold_timeout.value()
        pos_tol = self.spin_trot_pos_tol.value()

        # Ramp up swing lift height over the first cycle to avoid initial jump.
        elapsed = now - self._loco_start_time
        lift_eff = lift_h * min(1.0, elapsed / cycle_time_s)

        def compute_xy(phase):
            if phase < 0.5:
                prog = phase / 0.5
                return stand_h, -step_len + 2.0 * step_len * prog
            else:
                prog = (phase - 0.5) / 0.5
                return (stand_h - lift_eff * math.sin(math.pi * prog),
                        step_len - 2.0 * step_len * prog)

        phase_inc = dt / cycle_time_s

        # ── Rate-limited encoder requests during holds ─────────────────────
        # Request encoder estimates for any held pair at ≤ 20 Hz so the
        # position check below sees reasonably fresh data.
        if self._trot_pair_a_held or self._trot_pair_b_held:
            if now - self._trot_last_enc_req >= 0.05:
                self._trot_last_enc_req = now
                held_axes = []
                if self._trot_pair_a_held:
                    held_axes.extend([0, 1, 6, 7])
                if self._trot_pair_b_held:
                    held_axes.extend([2, 3, 4, 5])
                for axis in held_axes:
                    try:
                        self.bus.request_encoder_estimates(
                            self._loco_get_node_id(axis))
                    except Exception:
                        pass

        # ── Pair A: FL (axes 0,1) + RR (axes 6,7) ─────────────────────────
        if not self._trot_pair_a_held:
            prev_a = self._trot_pair_a_phase
            self._trot_pair_a_phase += phase_inc

            # Detect stance→swing crossing (phase crosses 0.5 from below).
            if prev_a < 0.5 <= self._trot_pair_a_phase:
                self._trot_pair_a_phase = 0.5          # snap to boundary
                self._trot_pair_a_held = True
                self._trot_hold_deadline_a = now + hold_timeout
                # Command extended target: keeps motor error large → more torque.
                ext_y = step_len + overshoot_mm
                for axis in (0, 1, 6, 7):
                    pos = self._ik_get_angle(axis, stand_h, ext_y)
                    if not math.isnan(pos):
                        self._loco_set_position(axis, pos)

            if self._trot_pair_a_phase >= 1.0:
                self._trot_pair_a_phase -= 1.0
        else:
            # Check if pair A has reached the real stance endpoint.
            axes_a = (0, 1, 6, 7)
            real_a = [self._ik_get_angle(ax, stand_h, step_len) for ax in axes_a]
            reached_a = all(
                not math.isnan(real_a[i])
                and abs(self._loco_get_position(axes_a[i]) - real_a[i]) <= pos_tol
                for i in range(4)
            )
            if reached_a:
                self._trot_pair_a_held = False
            elif now >= self._trot_hold_deadline_a:
                self._trot_pair_a_held = False
                errs = [
                    abs(self._loco_get_position(axes_a[i]) - real_a[i])
                    if not math.isnan(real_a[i]) else float('nan')
                    for i in range(4)
                ]
                self._append_log(
                    f"[LOCO] Pair A hold timeout — errors: "
                    + ", ".join(f"{e:.3f}" for e in errs) + " rev")

        # ── Pair B: FR (axes 2,3) + RL (axes 4,5) ─────────────────────────
        if not self._trot_pair_b_held:
            prev_b = self._trot_pair_b_phase
            self._trot_pair_b_phase += phase_inc

            if prev_b < 0.5 <= self._trot_pair_b_phase:
                self._trot_pair_b_phase = 0.5
                self._trot_pair_b_held = True
                self._trot_hold_deadline_b = now + hold_timeout
                ext_y = step_len + overshoot_mm
                for axis in (2, 3, 4, 5):
                    pos = self._ik_get_angle(axis, stand_h, ext_y)
                    if not math.isnan(pos):
                        self._loco_set_position(axis, pos)

            if self._trot_pair_b_phase >= 1.0:
                self._trot_pair_b_phase -= 1.0
        else:
            axes_b = (2, 3, 4, 5)
            real_b = [self._ik_get_angle(ax, stand_h, step_len) for ax in axes_b]
            reached_b = all(
                not math.isnan(real_b[i])
                and abs(self._loco_get_position(axes_b[i]) - real_b[i]) <= pos_tol
                for i in range(4)
            )
            if reached_b:
                self._trot_pair_b_held = False
            elif now >= self._trot_hold_deadline_b:
                self._trot_pair_b_held = False
                errs = [
                    abs(self._loco_get_position(axes_b[i]) - real_b[i])
                    if not math.isnan(real_b[i]) else float('nan')
                    for i in range(4)
                ]
                self._append_log(
                    f"[LOCO] Pair B hold timeout — errors: "
                    + ", ".join(f"{e:.3f}" for e in errs) + " rev")

        # ── Command positions for non-held pairs ───────────────────────────
        if not self._trot_pair_a_held:
            x_a, y_a = compute_xy(self._trot_pair_a_phase)
            self._loco_move_leg(0, 1, x_a, y_a)   # FL
            self._loco_move_leg(6, 7, x_a, y_a)   # RR

        if not self._trot_pair_b_held:
            x_b, y_b = compute_xy(self._trot_pair_b_phase)
            self._loco_move_leg(2, 3, x_b, y_b)   # FR
            self._loco_move_leg(4, 5, x_b, y_b)   # RL

    # ── Stand Up ─────────────────────────────────────────────────────────

    def _loco_start_stand(self):
        if not self._check_connected():
            return
        if self._loco_running:
            self._loco_stop()

        input_mode = self._loco_selected_input_mode()
        self._loco_debug(
            f"[LOCO][STAND][DBG] Start requested (InputMode={InputMode(input_mode).name})"
        )

        # Compute one IK target pose used by both stand phases.
        stand_x = self.spin_stand_x.value()
        stand_y = self.spin_stand_y.value()
        self._loco_stand_target = [
            self._ik_get_angle(axis, stand_x, stand_y) for axis in range(8)
        ]
        for axis in range(8):
            if math.isnan(self._loco_stand_target[axis]):
                self._append_log(
                    f"[LOCO] Stand target unreachable for axis {axis} (x={stand_x:.1f}, y={stand_y:.1f})")
                return
        self._loco_debug(
            "[LOCO][STAND][DBG] stand_target="
            + ", ".join(f"{v:.3f}" for v in self._loco_stand_target)
        )

        # Prepare only front joints first, then measure true start pose for step 0.
        for axis in range(4):
            nid = self._loco_get_node_id(axis)
            self._loco_apply_input_mode(nid, input_mode)
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)
            self._loco_set_position(axis, self._loco_get_position(axis))
            self._loco_debug_stand_setpos(
                axis, self._loco_get_position(axis), "step0-hold"
            )
            self._loco_debug(
                f"[LOCO][STAND][DBG] axis={axis} nid={nid} -> CLOSED_LOOP"
            )

        # Keep rear legs disabled during front-leg stand phase.
        for axis in range(4, 8):
            nid = self._loco_get_node_id(axis)
            self.bus.set_axis_state(nid, AxisState.IDLE)
            time.sleep(0.01)
            self._loco_debug(
                f"[LOCO][STAND][DBG] axis={axis} nid={nid} -> IDLE (rear hold-off)"
            )

        time.sleep(0.1)

        for axis in range(4):
            nid = self._loco_get_node_id(axis)
            self.bus.request_encoder_estimates(nid)
        time.sleep(0.1)

        for axis in range(4):
            self._loco_start_pose[axis] = self._loco_get_position(axis)
            self._sitting_pose[axis] = self._loco_start_pose[axis]
            self.sitting_pose_fields[axis].setText(
                f"{self._sitting_pose[axis]:.3f}")
        self._loco_debug(
            "[LOCO][STAND][DBG] step0_start(front)="
            + ", ".join(f"{self._loco_start_pose[i]:.3f}" for i in range(4))
        )

        self._loco_running = True
        self._loco_mode = 'stand'
        self._loco_step = 0
        self._loco_start_time = time.monotonic()
        self._loco_stand_duration = self.spin_stand_dur.value()
        self._loco_stand_dbg_last_log = 0.0
        self._loco_set_status("Standing Up — Front Legs", "#3399ff")
        self._append_log("[LOCO] Stand up — moving front legs")
        self._loco_debug(
            f"[LOCO][STAND][DBG] duration={self._loco_stand_duration:.2f}s, step=0"
        )
        self._loco_timer.start(5)

    def _loco_tick_stand(self):
        elapsed = time.monotonic() - self._loco_start_time
        duration = self._loco_stand_duration

        if elapsed - self._loco_stand_dbg_last_log >= 0.2:
            p = min(1.0, elapsed / duration) if duration > 0 else 1.0
            self._loco_debug(
                f"[LOCO][STAND][DBG] step={self._loco_step} elapsed={elapsed:.3f}s progress={p:.3f}"
            )
            self._loco_stand_dbg_last_log = elapsed

        # Step 0: move front legs to stand target while rear legs stay idle.
        if self._loco_step == 0:
            if elapsed >= duration:
                for axis in range(4):
                    self._loco_set_position(axis, self._loco_stand_target[axis])
                    self._loco_debug_stand_setpos(
                        axis, self._loco_stand_target[axis], "step0-final"
                    )
                time.sleep(0.5)
                # Re-enable rear legs, then continue with step 1.
                for axis in range(4, 8):
                    nid = self._loco_get_node_id(axis)
                    self._loco_apply_input_mode(nid, self._loco_selected_input_mode())
                    self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
                    time.sleep(0.005)
                    self._loco_set_position(axis, self._loco_get_position(axis))
                    self._loco_debug_stand_setpos(
                        axis, self._loco_get_position(axis), "step1-hold"
                    )
                    self._loco_debug(
                        f"[LOCO][STAND][DBG] axis={axis} nid={nid} -> CLOSED_LOOP (step 1 enable)"
                    )
                for axis in range(4, 8):
                    nid = self._loco_get_node_id(axis)
                    self.bus.request_encoder_estimates(nid)
                time.sleep(0.1)
                for axis in range(4, 8):
                    self._loco_start_pose[axis] = self._loco_get_position(axis)
                    self._sitting_pose[axis] = self._loco_start_pose[axis]
                    self.sitting_pose_fields[axis].setText(
                        f"{self._sitting_pose[axis]:.3f}")
                self._loco_debug(
                    "[LOCO][STAND][DBG] step1_start(rear)="
                    + ", ".join(f"{self._loco_start_pose[i]:.3f}" for i in range(4, 8))
                )
                time.sleep(0.2)
                self._loco_step = 1
                self._loco_start_time = time.monotonic()
                self._loco_stand_dbg_last_log = 0.0
                self._loco_set_status("Standing Up — Back Legs", "#3399ff")
                self._append_log("[LOCO] Stand up — moving back legs")
                self._loco_debug("[LOCO][STAND][DBG] step=1 started")
                return

            progress = elapsed / duration
            # Smoothstep profile to avoid jerk at start/end of each phase.
            s = progress * progress * (3.0 - 2.0 * progress)
            for axis in range(4):
                pos = (self._loco_start_pose[axis]
                       + (self._loco_stand_target[axis]
                          - self._loco_start_pose[axis]) * s)
                self._loco_set_position(axis, pos)
                self._loco_debug_stand_setpos(axis, pos, "step0")

        # Step 1: move rear legs to stand target and finish.
        elif self._loco_step == 1:
            if elapsed >= duration:
                for axis in range(4, 8):
                    self._loco_set_position(axis, self._loco_stand_target[axis])
                    self._loco_debug_stand_setpos(
                        axis, self._loco_stand_target[axis], "step1-final"
                    )
                self._loco_stop()
                self._loco_set_status("Standing", "#00cc44")
                self._append_log("[LOCO] Dog is standing")
                self._loco_debug("[LOCO][STAND][DBG] complete")
                return

            progress = elapsed / duration
            s = progress * progress * (3.0 - 2.0 * progress)
            for axis in range(4, 8):
                pos = (self._loco_start_pose[axis]
                       + (self._loco_stand_target[axis]
                          - self._loco_start_pose[axis]) * s)
                self._loco_set_position(axis, pos)
                self._loco_debug_stand_setpos(axis, pos, "step1")

    # ── Sit Down ─────────────────────────────────────────────────────────

    def _loco_start_sit(self):
        if not self._check_connected():
            return
        if self._loco_running:
            self._loco_stop()

        input_mode = self._loco_selected_input_mode()

        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.set_axis_state(nid, AxisState.CLOSED_LOOP)
            time.sleep(0.005)
            self._loco_apply_input_mode(nid, input_mode)

        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.request_encoder_estimates(nid)
        time.sleep(0.1)

        for axis in range(8):
            self._loco_start_pose[axis] = self._loco_get_position(axis)
        self._loco_grounded = [False] * 8

        for axis in range(8):
            nid = self._loco_get_node_id(axis)
            self.bus.request_iq(nid)

        self._loco_running = True
        self._loco_mode = 'sit'
        self._loco_start_time = time.monotonic()
        self._loco_sit_duration = self.spin_sit_dur.value()
        self._loco_set_status("Sitting Down", "#b7950b")
        self._append_log("[LOCO] Sit down sequence started")
        self._loco_timer.start(10)

    def _loco_tick_sit(self):
        elapsed = time.monotonic() - self._loco_start_time
        duration = self._loco_sit_duration

        if elapsed >= duration:
            for axis in range(8):
                nid = self._loco_get_node_id(axis)
                self.bus.set_axis_state(nid, AxisState.IDLE)
                time.sleep(0.01)
            self._loco_stop()
            self._loco_set_status("Sitting", "#ffcc00")
            self._append_log("[LOCO] Sit down complete")
            return

        progress = elapsed / duration
        s = progress * progress * (3.0 - 2.0 * progress)

        iq_thresh = self.spin_sit_iq_thresh.value()
        pos_tol = self.spin_sit_pos_tol.value()

        for axis in range(8):
            if self._loco_grounded[axis]:
                continue

            target = self._sitting_pose[axis]
            pos = (self._loco_start_pose[axis]
                   + (target - self._loco_start_pose[axis]) * s)
            self._loco_set_position(axis, pos)

            if progress > 0.5:
                nid = self._loco_get_node_id(axis)
                self.bus.request_iq(nid)
                time.sleep(0.002)
                iq = abs(self._loco_get_iq(axis))
                dist = abs(pos - target)
                if iq > iq_thresh and dist < pos_tol:
                    self._append_log(
                        f"[LOCO] Ground detected: axis {axis} (Iq={iq:.2f}A)")
                    self.bus.set_axis_state(nid, AxisState.IDLE)
                    self._loco_grounded[axis] = True

    # ── Locomotion Timer Dispatch ────────────────────────────────────────

    def _loco_tick(self):
        if not self._loco_running or not self.bus.is_connected:
            self._loco_stop()
            return
        if self._loco_mode == 'trot':
            self._loco_tick_trot()
        elif self._loco_mode == 'trot_align':
            self._loco_tick_trot_align()
        elif self._loco_mode == 'trot_finish':
            self._loco_tick_trot_finish()
        elif self._loco_mode == 'stand':
            self._loco_tick_stand()
        elif self._loco_mode == 'sit':
            self._loco_tick_sit()
