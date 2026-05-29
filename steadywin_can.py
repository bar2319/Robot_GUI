"""
SteadyWin GIM6010-8 CAN Simple Protocol Driver
Based on the official datasheet rev2.0

CAN ID format (11 bits): node_id (bits 10-5) | cmd_id (bits 4-0)
Data: 8 bytes, little-endian, IEEE 754 floats
Default baud rate: 500 kbps
"""

import struct
import threading
import time
from enum import IntEnum
from typing import Optional, Callable

import can


# ─── CAN Simple CMD IDs ───────────────────────────────────────────────────────

class CmdID(IntEnum):
    HEARTBEAT              = 0x001
    ESTOP                  = 0x002
    GET_ERROR              = 0x003
    RXSDO                  = 0x004
    TXSDO                  = 0x005
    SET_AXIS_NODE_ID       = 0x006
    SET_AXIS_STATE         = 0x007
    MIT_CONTROL            = 0x008
    GET_ENCODER_ESTIMATES  = 0x009
    GET_ENCODER_COUNT      = 0x00A
    SET_CONTROLLER_MODE    = 0x00B
    SET_INPUT_POS          = 0x00C
    SET_INPUT_VEL          = 0x00D
    SET_INPUT_TORQUE       = 0x00E
    SET_LIMITS             = 0x00F
    START_ANTICOGGING      = 0x010
    SET_TRAJ_VEL_LIMIT     = 0x011
    SET_TRAJ_ACCEL_LIMITS  = 0x012
    SET_TRAJ_INERTIA       = 0x013
    GET_IQ                 = 0x014
    REBOOT                 = 0x016
    GET_BUS_VOLTAGE_CURRENT = 0x017
    CLEAR_ERRORS           = 0x018
    SET_LINEAR_COUNT       = 0x019
    SET_POS_GAIN           = 0x01A
    SET_VEL_GAINS          = 0x01B
    GET_TORQUES            = 0x01C
    GET_POWERS             = 0x01D
    DISABLE_CAN            = 0x01E
    SAVE_CONFIGURATION     = 0x01F


class AxisState(IntEnum):
    UNDEFINED              = 0
    IDLE                   = 1
    FULL_CALIBRATION       = 3
    MOTOR_CALIBRATION      = 4
    ENCODER_CALIBRATION    = 7
    CLOSED_LOOP            = 8
    HOMING                 = 11


class ControlMode(IntEnum):
    VOLTAGE  = 0
    TORQUE   = 1
    VELOCITY = 2
    POSITION = 3


class InputMode(IntEnum):
    INACTIVE      = 0
    PASSTHROUGH   = 1
    VEL_RAMP      = 2
    POS_FILTER    = 3
    TRAP_TRAJ     = 5
    TORQUE_RAMP   = 6
    MIT           = 9


AXIS_STATE_NAMES = {
    0: "Undefined", 1: "Idle", 3: "Full Calibration",
    4: "Motor Calibration", 7: "Encoder Calibration",
    8: "Closed Loop", 11: "Homing",
}

CONTROL_MODE_NAMES = {
    0: "Voltage", 1: "Torque", 2: "Velocity", 3: "Position",
}

INPUT_MODE_NAMES = {
    0: "Inactive", 1: "Passthrough", 2: "Vel Ramp",
    3: "Pos Filter", 5: "Trap Traj", 6: "Torque Ramp", 9: "MIT",
}


# ─── Motor Feedback Data ──────────────────────────────────────────────────────

class MotorFeedback:
    """Stores the latest feedback from a motor."""
    def __init__(self):
        self.axis_error: int = 0
        self.axis_state: int = 0
        self.motor_flag: int = 0
        self.encoder_flag: int = 0
        self.controller_flag: int = 0
        self.traj_done: bool = False
        self.life: int = 0

        self.pos_estimate: float = 0.0
        self.vel_estimate: float = 0.0
        self.shadow_count: int = 0
        self.count_in_cpr: int = 0

        self.iq_setpoint: float = 0.0
        self.iq_measured: float = 0.0

        self.bus_voltage: float = 0.0
        self.bus_current: float = 0.0

        self.torque_setpoint: float = 0.0
        self.torque_measured: float = 0.0

        self.electrical_power: float = 0.0
        self.mechanical_power: float = 0.0

        self.last_heartbeat_time: float = 0.0


# ─── CAN Bus Manager ──────────────────────────────────────────────────────────

class SteadyWinBus:
    """Manages the CAN bus and communicates with SteadyWin motors."""

    def __init__(self):
        self.bus: Optional[can.Bus] = None
        self._listener_thread: Optional[threading.Thread] = None
        self._running = False
        self._motors: dict[int, MotorFeedback] = {}  # node_id -> feedback
        self._on_heartbeat: Optional[Callable] = None
        self._on_feedback: Optional[Callable] = None
        self._on_message_log: Optional[Callable] = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self.bus is not None and self._running

    def connect(self, interface: str, channel: str, bitrate: int = 500000,
                **kwargs) -> None:
        """Connect to CAN bus.
        
        interface: 'socketcan', 'slcan', 'gs_usb', 'virtual', etc.
        channel: 'can0', '/dev/ttyACM0', etc.
        """
        self.bus = can.Bus(interface=interface, channel=channel,
                           bitrate=bitrate, **kwargs)
        self._running = True
        self._listener_thread = threading.Thread(target=self._listen_loop,
                                                  daemon=True)
        self._listener_thread.start()

    def disconnect(self) -> None:
        self._running = False
        if self._listener_thread:
            self._listener_thread.join(timeout=2.0)
            self._listener_thread = None
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def register_motor(self, node_id: int) -> MotorFeedback:
        if node_id not in self._motors:
            self._motors[node_id] = MotorFeedback()
        return self._motors[node_id]

    def get_feedback(self, node_id: int) -> Optional[MotorFeedback]:
        return self._motors.get(node_id)

    # ── Callback setters ──

    def set_heartbeat_callback(self, cb: Callable):
        self._on_heartbeat = cb

    def set_feedback_callback(self, cb: Callable):
        self._on_feedback = cb

    def set_message_log_callback(self, cb: Callable):
        self._on_message_log = cb

    # ── CAN ID helpers ──

    @staticmethod
    def make_can_id(node_id: int, cmd_id: int) -> int:
        return (node_id << 5) | cmd_id

    @staticmethod
    def parse_can_id(can_id: int) -> tuple[int, int]:
        node_id = (can_id >> 5) & 0x3F
        cmd_id = can_id & 0x1F
        return node_id, cmd_id

    # ── Send helpers ──

    def _send(self, node_id: int, cmd_id: int, data: bytes = b'') -> None:
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        padded = data.ljust(8, b'\x00')[:8]
        msg = can.Message(
            arbitration_id=self.make_can_id(node_id, cmd_id),
            data=padded,
            is_extended_id=False,
        )
        self.bus.send(msg)
        if self._on_message_log:
            self._on_message_log(f"TX  ID=0x{msg.arbitration_id:03X}  "
                                  f"Data={msg.data.hex(' ').upper()}")

    def _request(self, node_id: int, cmd_id: int) -> None:
        """Send RTR frame to request data from motor."""
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        msg = can.Message(
            arbitration_id=self.make_can_id(node_id, cmd_id),
            is_remote_frame=True,
            is_extended_id=False,
            dlc=8,
        )
        self.bus.send(msg)
        if self._on_message_log:
            self._on_message_log(f"TX RTR  ID=0x{msg.arbitration_id:03X}")

    # ── Motor Commands ──

    def estop(self, node_id: int) -> None:
        self._send(node_id, CmdID.ESTOP)

    def set_axis_state(self, node_id: int, state: int) -> None:
        data = struct.pack('<I', state) + b'\x00' * 4
        self._send(node_id, CmdID.SET_AXIS_STATE, data)

    def set_controller_mode(self, node_id: int, control_mode: int,
                             input_mode: int) -> None:
        data = struct.pack('<II', control_mode, input_mode)
        self._send(node_id, CmdID.SET_CONTROLLER_MODE, data)

    def set_input_pos(self, node_id: int, pos: float,
                       vel_ff: float = 0.0, torque_ff: float = 0.0) -> None:
        vel_ff_int = int(vel_ff * 1000)    # 0.001 rev/s units
        torque_ff_int = int(torque_ff * 1000)  # 0.001 Nm units
        data = struct.pack('<fhh', pos, vel_ff_int, torque_ff_int)
        self._send(node_id, CmdID.SET_INPUT_POS, data)

    def set_input_vel(self, node_id: int, vel: float,
                       torque_ff: float = 0.0) -> None:
        data = struct.pack('<ff', vel, torque_ff)
        self._send(node_id, CmdID.SET_INPUT_VEL, data)

    def set_input_torque(self, node_id: int, torque: float) -> None:
        data = struct.pack('<f', torque) + b'\x00' * 4
        self._send(node_id, CmdID.SET_INPUT_TORQUE, data)

    def set_limits(self, node_id: int, vel_limit: float,
                    current_limit: float) -> None:
        data = struct.pack('<ff', vel_limit, current_limit)
        self._send(node_id, CmdID.SET_LIMITS, data)

    def set_traj_vel_limit(self, node_id: int, vel_limit: float) -> None:
        data = struct.pack('<f', vel_limit) + b'\x00' * 4
        self._send(node_id, CmdID.SET_TRAJ_VEL_LIMIT, data)

    def set_traj_accel_limits(self, node_id: int, accel_limit: float,
                               decel_limit: float) -> None:
        data = struct.pack('<ff', accel_limit, decel_limit)
        self._send(node_id, CmdID.SET_TRAJ_ACCEL_LIMITS, data)

    def set_traj_inertia(self, node_id: int, inertia: float) -> None:
        data = struct.pack('<f', inertia) + b'\x00' * 4
        self._send(node_id, CmdID.SET_TRAJ_INERTIA, data)

    def set_pos_gain(self, node_id: int, pos_gain: float) -> None:
        data = struct.pack('<f', pos_gain) + b'\x00' * 4
        self._send(node_id, CmdID.SET_POS_GAIN, data)

    def set_vel_gains(self, node_id: int, vel_gain: float,
                       vel_integrator_gain: float) -> None:
        data = struct.pack('<ff', vel_gain, vel_integrator_gain)
        self._send(node_id, CmdID.SET_VEL_GAINS, data)

    def set_linear_count(self, node_id: int, count: int) -> None:
        data = struct.pack('<i', count) + b'\x00' * 4
        self._send(node_id, CmdID.SET_LINEAR_COUNT, data)

    def start_anticogging(self, node_id: int) -> None:
        self._send(node_id, CmdID.START_ANTICOGGING)

    def clear_errors(self, node_id: int) -> None:
        self._send(node_id, CmdID.CLEAR_ERRORS)

    def reboot(self, node_id: int) -> None:
        self._send(node_id, CmdID.REBOOT)

    def save_configuration(self, node_id: int) -> None:
        self._send(node_id, CmdID.SAVE_CONFIGURATION)

    def request_encoder_estimates(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_ENCODER_ESTIMATES)

    def request_encoder_count(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_ENCODER_COUNT)

    def request_bus_voltage_current(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_BUS_VOLTAGE_CURRENT)

    def request_iq(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_IQ)

    def request_torques(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_TORQUES)

    def request_powers(self, node_id: int) -> None:
        self._request(node_id, CmdID.GET_POWERS)

    def get_error(self, node_id: int, error_type: int = 0) -> None:
        data = struct.pack('<B', error_type) + b'\x00' * 7
        self._send(node_id, CmdID.GET_ERROR, data)

    def mit_control(self, node_id: int, pos_rad: float, vel_rad_s: float,
                     kp: float, kd: float, torque_nm: float) -> None:
        """Send MIT control frame. Units are output-shaft side:
        pos in rad, vel in rad/s, torque in Nm."""
        pos_int = int((pos_rad + 12.5) * 65535 / 25) & 0xFFFF
        vel_int = int((vel_rad_s + 65) * 4095 / 130) & 0xFFF
        kp_int = int(kp * 4095 / 500) & 0xFFF
        kd_int = int(kd * 4095 / 5) & 0xFFF
        t_int = int((torque_nm + 50) * 4095 / 100) & 0xFFF

        b0 = (pos_int >> 8) & 0xFF
        b1 = pos_int & 0xFF
        b2 = (vel_int >> 4) & 0xFF
        b3 = ((vel_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
        b4 = kp_int & 0xFF
        b5 = (kd_int >> 4) & 0xFF
        b6 = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
        b7 = t_int & 0xFF

        data = bytes([b0, b1, b2, b3, b4, b5, b6, b7])
        self._send(node_id, CmdID.MIT_CONTROL, data)

    # ── Listener ──

    def _listen_loop(self) -> None:
        while self._running and self.bus:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                self._process_message(msg)
            except can.CanError:
                if self._running:
                    time.sleep(0.01)

    def _process_message(self, msg: can.Message) -> None:
        node_id, cmd_id = self.parse_can_id(msg.arbitration_id)
        fb = self._motors.get(node_id)
        if fb is None:
            return

        if self._on_message_log:
            self._on_message_log(f"RX  ID=0x{msg.arbitration_id:03X}  "
                                  f"Data={msg.data.hex(' ').upper()}")

        data = msg.data

        if cmd_id == CmdID.HEARTBEAT:
            fb.axis_error = struct.unpack_from('<I', data, 0)[0]
            fb.axis_state = data[4]
            # firmware >= 0.5.12 format
            flags = data[5]
            fb.motor_flag = flags & 0x01
            fb.encoder_flag = (flags >> 1) & 0x01
            fb.controller_flag = (flags >> 2) & 0x01
            fb.traj_done = bool((flags >> 7) & 0x01)
            fb.life = data[7]
            fb.last_heartbeat_time = time.time()
            if self._on_heartbeat:
                self._on_heartbeat(node_id)

        elif cmd_id == CmdID.GET_ENCODER_ESTIMATES:
            fb.pos_estimate, fb.vel_estimate = struct.unpack_from('<ff', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'encoder')

        elif cmd_id == CmdID.GET_ENCODER_COUNT:
            fb.shadow_count, fb.count_in_cpr = struct.unpack_from('<ii', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'encoder_count')

        elif cmd_id == CmdID.GET_IQ:
            fb.iq_setpoint, fb.iq_measured = struct.unpack_from('<ff', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'iq')

        elif cmd_id == CmdID.GET_BUS_VOLTAGE_CURRENT:
            fb.bus_voltage, fb.bus_current = struct.unpack_from('<ff', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'bus')

        elif cmd_id == CmdID.GET_TORQUES:
            fb.torque_setpoint, fb.torque_measured = struct.unpack_from('<ff', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'torques')

        elif cmd_id == CmdID.GET_POWERS:
            fb.electrical_power, fb.mechanical_power = struct.unpack_from('<ff', data, 0)
            if self._on_feedback:
                self._on_feedback(node_id, 'powers')
