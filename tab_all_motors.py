"""
All Motors tab — node ID assignment, status overview, robot leg visualization.
Used as a mixin for MainWindow.
"""

import math
import time
from functools import partial

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QSpinBox, QLabel, QMessageBox,
)

from helpers import (
    LEDIndicator, make_label, make_double_spin,
    LEG_NAMES, JOINT_NAMES, DEFAULT_NODE_IDS,
)
from steadywin_can import AxisState


class AllMotorsMixin:
    """Mixin providing the All Motors tab, status overview, and robot visualization."""

    def _build_all_motors_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Node ID configuration
        config_grp = QGroupBox("Motor Node ID Assignment")
        config_lay = QGridLayout(config_grp)
        self.node_id_spins = {}

        for leg_idx, leg_name in enumerate(LEG_NAMES):
            config_lay.addWidget(make_label(leg_name, bold=True),
                                  0, leg_idx * 2, 1, 2)
            for joint_idx, joint_name in enumerate(JOINT_NAMES):
                row = joint_idx + 1
                col = leg_idx * 2
                config_lay.addWidget(make_label(f"  {joint_name}:"), row, col)
                spin = QSpinBox()
                spin.setRange(0, 63)
                spin.setValue(DEFAULT_NODE_IDS[leg_idx * 2 + joint_idx])
                config_lay.addWidget(spin, row, col + 1)
                self.node_id_spins[(leg_idx, joint_idx)] = spin

        layout.addWidget(config_grp)

        # Quick actions for all
        all_grp = QGroupBox("All Motors — Quick Actions")
        all_lay = QHBoxLayout(all_grp)

        btn_all_idle = QPushButton("All → IDLE")
        btn_all_idle.clicked.connect(partial(self._set_all_state, AxisState.IDLE))
        all_lay.addWidget(btn_all_idle)

        btn_all_closed = QPushButton("All → Closed Loop")
        btn_all_closed.setStyleSheet("background: #1e8449;")
        btn_all_closed.clicked.connect(partial(self._set_all_state, AxisState.CLOSED_LOOP))
        all_lay.addWidget(btn_all_closed)

        btn_all_calib = QPushButton("All → Full Calib")
        btn_all_calib.setStyleSheet("background: #b7950b;")
        btn_all_calib.clicked.connect(partial(self._set_all_state, AxisState.FULL_CALIBRATION))
        all_lay.addWidget(btn_all_calib)

        btn_all_estop = QPushButton("E-STOP ALL")
        btn_all_estop.setStyleSheet("background: #922b21; font-weight: bold; padding: 8px;")
        btn_all_estop.clicked.connect(self._estop_all)
        all_lay.addWidget(btn_all_estop)

        btn_all_clear = QPushButton("Clear All Errors")
        btn_all_clear.clicked.connect(self._clear_all_errors)
        all_lay.addWidget(btn_all_clear)

        btn_all_save = QPushButton("Save All Config")
        btn_all_save.clicked.connect(self._save_all_config)
        all_lay.addWidget(btn_all_save)

        self.btn_all_reboot = QPushButton("Reboot All Motors")
        self.btn_all_reboot.setStyleSheet("background: #7d6608; font-weight: bold;")
        self.btn_all_reboot.clicked.connect(self._reboot_all_motors)
        all_lay.addWidget(self.btn_all_reboot)

        layout.addWidget(all_grp)

        # Status overview per motor
        status_grp = QGroupBox("Motor Status Overview")
        status_lay = QGridLayout(status_grp)

        headers = ["Leg/Joint", "Node", "State", "Error", "Pos (rev)",
                    "Vel (rev/s)", "Vbus (V)", "HB"]
        for col, h in enumerate(headers):
            status_lay.addWidget(make_label(h, bold=True), 0, col)

        self.all_motors_labels = {}
        row = 1
        for leg_idx, leg_name in enumerate(LEG_NAMES):
            for joint_idx, joint_name in enumerate(JOINT_NAMES):
                motor_key = (leg_idx, joint_idx)
                nid = DEFAULT_NODE_IDS[leg_idx * 2 + joint_idx]
                short_name = f"{leg_name.split('(')[1].rstrip(')')}-{joint_name}"

                status_lay.addWidget(make_label(short_name), row, 0)
                lbl_node = make_label(str(nid))
                lbl_state = make_label("—")
                lbl_error = make_label("—")
                lbl_pos = make_label("—")
                lbl_vel = make_label("—")
                lbl_vbus = make_label("—")
                led = LEDIndicator(10)

                status_lay.addWidget(lbl_node, row, 1)
                status_lay.addWidget(lbl_state, row, 2)
                status_lay.addWidget(lbl_error, row, 3)
                status_lay.addWidget(lbl_pos, row, 4)
                status_lay.addWidget(lbl_vel, row, 5)
                status_lay.addWidget(lbl_vbus, row, 6)
                status_lay.addWidget(led, row, 7)

                self.all_motors_labels[motor_key] = {
                    'node': lbl_node, 'state': lbl_state,
                    'error': lbl_error, 'pos': lbl_pos,
                    'vel': lbl_vel, 'vbus': lbl_vbus, 'led': led,
                }
                row += 1

        layout.addWidget(status_grp)

        # ── Robot Geometry / Visualization Status ──
        viz_grp = QGroupBox("Robot Geometry")
        viz_lay = QVBoxLayout(viz_grp)

        viz_note = QLabel(
            "Leg side-view visualization and background all-motor polling "
            "are disabled to keep the GUI responsive.")
        viz_note.setWordWrap(True)
        viz_lay.addWidget(viz_note)

        viz_config = QHBoxLayout()
        btn_set_offsets = QPushButton("Set Current as Zero")
        btn_set_offsets.setToolTip("Use current motor positions as zero offsets")
        btn_set_offsets.clicked.connect(self._set_offsets_from_current)
        viz_config.addWidget(btn_set_offsets)

        btn_reset_offsets = QPushButton("Reset Offsets")
        btn_reset_offsets.clicked.connect(self._reset_offsets)
        viz_config.addWidget(btn_reset_offsets)

        viz_config.addSpacing(20)
        viz_config.addWidget(make_label("L1 (mm):"))
        self.spin_L1 = make_double_spin(10, 500, 1, 133.0, 1.0)
        self.spin_L1.valueChanged.connect(lambda v: setattr(self, '_robot_L1', v))
        viz_config.addWidget(self.spin_L1)
        viz_config.addWidget(make_label("L2 (mm):"))
        self.spin_L2 = make_double_spin(10, 500, 1, 133.0, 1.0)
        self.spin_L2.valueChanged.connect(lambda v: setattr(self, '_robot_L2', v))
        viz_config.addWidget(self.spin_L2)
        viz_config.addWidget(make_label("GR:"))
        self.spin_GR = make_double_spin(1, 100, 1, 8.0, 1.0)
        self.spin_GR.valueChanged.connect(lambda v: setattr(self, '_robot_GR', v))
        viz_config.addWidget(self.spin_GR)
        viz_config.addStretch()
        viz_lay.addLayout(viz_config)
        layout.addWidget(viz_grp, 1)
        return widget

    # ── Robot Visualization FK ───────────────────────────────────────────

    def _compute_leg_fk(self, hip_motor_pos: float, knee_motor_pos: float,
                         leg_idx: int) -> list:
        """Forward kinematics: motor turns → display coords."""
        L1 = self._robot_L1
        L2 = self._robot_L2
        GR = self._robot_GR

        hip_idx = leg_idx * 2
        knee_idx = leg_idx * 2 + 1

        hip_base = (hip_motor_pos - self._motor_offsets[hip_idx]) / \
                    self._motor_directions[hip_idx]
        knee_base = (knee_motor_pos - self._motor_offsets[knee_idx]) / \
                     self._motor_directions[knee_idx]

        hip_temp = -hip_base * 2 * math.pi / GR
        knee_temp = -knee_base * 2 * math.pi / GR

        link1_angle = -hip_temp
        elbow_x = L1 * math.cos(link1_angle)
        elbow_y = L1 * math.sin(link1_angle)

        link2_angle = link1_angle + knee_temp
        foot_x = elbow_x + L2 * math.cos(link2_angle)
        foot_y = elbow_y + L2 * math.sin(link2_angle)

        return [(0, 0),
                (elbow_y, -elbow_x),
                (foot_y, -foot_x)]

    def _update_robot_viz(self):
        """Update the robot leg visualization from motor feedback."""
        if (not getattr(self, '_leg_visualization_enabled', True)
                or not hasattr(self, 'viz_leg_lines')):
            return

        hip_x_positions = [-250, -100, 100, 250]

        body_xs = []
        body_ys = []

        for leg_idx in range(4):
            hip_spin = self.node_id_spins.get((leg_idx, 0))
            knee_spin = self.node_id_spins.get((leg_idx, 1))
            if not hip_spin or not knee_spin:
                continue

            hip_nid = hip_spin.value()
            knee_nid = knee_spin.value()
            hip_fb = self.bus.get_feedback(hip_nid)
            knee_fb = self.bus.get_feedback(knee_nid)

            bx = hip_x_positions[leg_idx]
            body_xs.append(bx)
            body_ys.append(0)

            if hip_fb and knee_fb:
                joints = self._compute_leg_fk(
                    hip_fb.pos_estimate, knee_fb.pos_estimate, leg_idx)

                xs = [bx + j[0] for j in joints]
                ys = [j[1] for j in joints]

                self.viz_leg_lines[leg_idx].setData(xs, ys)
                self.viz_foot_markers[leg_idx].setData([xs[-1]], [ys[-1]])
                self.viz_leg_labels[leg_idx].setPos(bx, 15)
            else:
                xs = [bx, bx, bx]
                ys = [0, -self._robot_L1, -(self._robot_L1 + self._robot_L2)]
                self.viz_leg_lines[leg_idx].setData(xs, ys)
                self.viz_foot_markers[leg_idx].setData([bx], [ys[-1]])
                self.viz_leg_labels[leg_idx].setPos(bx, 15)

        if body_xs:
            self.viz_body_line.setData(
                [min(body_xs) - 20, max(body_xs) + 20], [0, 0])

    def _set_offsets_from_current(self):
        """Use current motor positions as zero offsets."""
        if not self.bus.is_connected:
            QMessageBox.warning(self, "Not Connected",
                                "Connect to CAN bus first.")
            return
        for leg_idx in range(4):
            for joint_idx in range(2):
                spin = self.node_id_spins.get((leg_idx, joint_idx))
                if not spin:
                    continue
                nid = spin.value()
                fb = self.bus.get_feedback(nid)
                if fb:
                    motor_idx = leg_idx * 2 + joint_idx
                    self._motor_offsets[motor_idx] = fb.pos_estimate
        self.statusBar().showMessage("Offsets set from current positions")
        self._update_robot_viz()

    def _reset_offsets(self):
        self._motor_offsets = [0.0] * 8
        self.statusBar().showMessage("Offsets reset to zero")
        self._update_robot_viz()

    def _update_all_motors_status(self, node_id: int, fb):
        from steadywin_can import AXIS_STATE_NAMES
        for (leg_idx, joint_idx), labels in self.all_motors_labels.items():
            spin = self.node_id_spins[(leg_idx, joint_idx)]
            nid = spin.value()
            labels['node'].setText(str(nid))
            if nid != node_id:
                continue
            state_name = AXIS_STATE_NAMES.get(fb.axis_state, "?")
            labels['state'].setText(state_name)
            labels['error'].setText(f"0x{fb.axis_error:04X}" if fb.axis_error else "OK")
            labels['pos'].setText(f"{fb.pos_estimate:.3f}")
            labels['vel'].setText(f"{fb.vel_estimate:.2f}")
            labels['vbus'].setText(f"{fb.bus_voltage:.1f}")
            if fb.axis_state == AxisState.CLOSED_LOOP:
                labels['led'].set_color("#00cc44")
            elif fb.axis_error:
                labels['led'].set_color("#ff3333")
            else:
                labels['led'].set_color("#ffcc00")

    def _reboot_all_motors(self):
        if not self._check_connected():
            return
        if getattr(self, '_all_reboot_in_progress', False):
            return

        node_ids = self._get_all_node_ids()
        if not node_ids:
            return

        self._all_reboot_in_progress = True
        self.btn_all_reboot.setEnabled(False)
        self.statusBar().showMessage("Rebooting all motors... wait 5 seconds")
        self._append_log(f"[ALL] Rebooting {len(node_ids)} motors")

        for nid in node_ids:
            self.bus.reboot(nid)
            time.sleep(0.05)

        if hasattr(self, '_poll_timer') and self._poll_timer.isActive():
            self._poll_timer.stop()

        QTimer.singleShot(5000, self._finish_reboot_all_motors)

    def _finish_reboot_all_motors(self):
        self._all_reboot_in_progress = False
        self.btn_all_reboot.setEnabled(True)

        if (self.bus.is_connected and hasattr(self, '_poll_timer')
                and getattr(self, '_can_monitoring_enabled', True)):
            self._poll_timer.start(self._poll_interval_ms)

        self.statusBar().showMessage("All motors reboot complete")
        self._append_log("[ALL] Reboot complete")
