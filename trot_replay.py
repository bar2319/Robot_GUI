"""
Trot position replay window.

Opens after a trot run stops and shows — for each of the 8 motors — the
expected trajectory (dashed) alongside what the encoder actually reported
(solid).  Gaps between the two lines are tracking errors.
"""

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

MOTOR_NAMES = [
    "FL-Hip (0)", "FL-Knee (1)",
    "FR-Hip (2)", "FR-Knee (3)",
    "RL-Hip (4)", "RL-Knee (5)",
    "RR-Hip (6)", "RR-Knee (7)",
]

# One distinct colour per motor
COLORS = [
    "#ff6b6b",  # FL-Hip   — red
    "#ffa94d",  # FL-Knee  — orange
    "#51cf66",  # FR-Hip   — green
    "#339af0",  # FR-Knee  — blue
    "#cc5de8",  # RL-Hip   — purple
    "#f06595",  # RL-Knee  — pink
    "#20c997",  # RR-Hip   — teal
    "#fab005",  # RR-Knee  — yellow
]


class TrotReplayWindow(pg.GraphicsLayoutWidget):
    """4-column × 2-row grid of per-motor position plots."""

    def __init__(self, recording: list, params: dict):
        super().__init__(title="Trot Position Replay")
        self.setWindowTitle("Trot Position Replay")
        self.setWindowFlags(Qt.Window)
        self.resize(1280, 720)
        self.setBackground("#1a1a1a")

        if not recording:
            return

        times    = [s[0] for s in recording]
        expected = [[s[1][i] for s in recording] for i in range(8)]
        actual   = [[s[2][i] for s in recording] for i in range(8)]

        # ── Header label (params summary) ────────────────────────────────
        param_txt = (
            f"step={params.get('step_len', 0):.0f} mm  "
            f"stand_h={params.get('stand_h', 0):.0f} mm  "
            f"lift_h={params.get('lift_h', 0):.0f} mm  "
            f"cycle={params.get('cycle_ms', 0)} ms  "
            f"overshoot={params.get('overshoot_pct', 0):.0f}%  "
            f"     dashed = expected trajectory     solid = actual encoder"
        )
        self.addLabel(
            f'<span style="color:#d4d4d4;font-size:11px">{param_txt}</span>',
            row=0, col=0, colspan=4
        )

        dash = Qt.DashLine

        for i, (name, color) in enumerate(zip(MOTOR_NAMES, COLORS)):
            row = (i // 4) + 1   # rows 1 and 2 (row 0 = header)
            col = i % 4

            plot = self.addPlot(row=row, col=col, title=f"<b>{name}</b>")
            plot.setTitle(f'<span style="color:{color}">{name}</span>')
            plot.showGrid(x=True, y=True, alpha=0.3)

            if col == 0:
                plot.setLabel("left", "position (rev)", color="#aaa")
            if row == 2:
                plot.setLabel("bottom", "time (s)", color="#aaa")

            # Expected — thin dashed
            plot.plot(
                times, expected[i],
                pen=pg.mkPen(color=color, width=1, style=dash),
                name="expected",
            )
            # Actual — solid, slightly thicker
            plot.plot(
                times, actual[i],
                pen=pg.mkPen(color=color, width=2),
                name="actual",
            )

            # Shade the error region between expected and actual
            fill = pg.FillBetweenItem(
                pg.PlotDataItem(times, expected[i]),
                pg.PlotDataItem(times, actual[i]),
                brush=pg.mkBrush(color + "33"),   # 20 % opacity
            )
            plot.addItem(fill)
