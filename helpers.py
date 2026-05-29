"""
Shared widgets, helpers, and constants for the motor GUI.
"""

from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QLabel, QLineEdit, QDoubleSpinBox, QFrame,
)


# ─── Constants ────────────────────────────────────────────────────────────────

LEG_NAMES = [
    "Front-Left (FL)", "Front-Right (FR)",
    "Rear-Left (RL)", "Rear-Right (RR)",
]
JOINT_NAMES = ["Hip", "Knee"]

# Default node IDs: FL-hip=0, FL-knee=1, FR-hip=2, FR-knee=3, ...
DEFAULT_NODE_IDS = list(range(8))


# ─── Thread-safe signal bridge ────────────────────────────────────────────────

class SignalBridge(QObject):
    heartbeat_signal = pyqtSignal(int)
    feedback_signal = pyqtSignal(int, str)
    log_signal = pyqtSignal(str)
    pid_data_signal = pyqtSignal(float, float, float)  # time, target, estimated


# ─── LED Indicator ────────────────────────────────────────────────────────────

class LEDIndicator(QFrame):
    def __init__(self, size: int = 14):
        super().__init__()
        self.setFixedSize(size, size)
        self._color = "gray"
        self._update_style()

    def set_color(self, color: str):
        self._color = color
        self._update_style()

    def _update_style(self):
        self.setStyleSheet(
            f"background-color: {self._color}; border-radius: 7px; "
            f"border: 1px solid #888;"
        )


# ─── Helper Functions ─────────────────────────────────────────────────────────

def make_label(text: str, bold: bool = False) -> QLabel:
    lbl = QLabel(text)
    if bold:
        f = lbl.font()
        f.setBold(True)
        lbl.setFont(f)
    return lbl


def make_readonly_line(width: int = 120) -> QLineEdit:
    le = QLineEdit("—")
    le.setReadOnly(True)
    le.setFixedWidth(width)
    le.setAlignment(Qt.AlignCenter)
    le.setStyleSheet("background: #2b2b2b; color: #e0e0e0; border: 1px solid #555;")
    return le


def make_double_spin(min_val: float, max_val: float, decimals: int = 3,
                      value: float = 0.0, step: float = 0.1) -> QDoubleSpinBox:
    spin = QDoubleSpinBox()
    spin.setRange(min_val, max_val)
    spin.setDecimals(decimals)
    spin.setValue(value)
    spin.setSingleStep(step)
    return spin
