import sys
import threading
import math
import collections
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QFrame, QSizePolicy,
)
from PyQt5.QtCore import QObject, pyqtSignal as Signal, Qt, QTimer, QPointF
from PyQt5.QtGui import QFont, QPen, QColor, QPainter
import pyqtgraph as pg


# ── palette (softer pink) ────────────────
BG       = "#141414"
PANEL_BG = "#1c1c1c"
ACCENT   = "#d37d98"
ACC_DIM  = "#7A2E45"
ACC_LO   = "#3a1525"
TEXT     = "#d37d98"
TEXT_DIM = "#5e3040"
HISTORY  = 200

# motor indicator colors
MTR_ON   = "#C43D60"
MTR_OFF  = "#2a1520"
MTR_REV  = "#8B3A5E"


STYLESHEET = f"""
QMainWindow, QWidget {{
    background: {BG};
    color: {TEXT};
}}
QLabel {{
    color: {TEXT};
    font-family: "Menlo", "SF Mono", "Menlo", monospace;
}}
"""


# ─────────────────────────────────────────
# ROS2 Bridge
# ─────────────────────────────────────────
class RosBridge(QObject):
    lidar_updated   = Signal(object)
    cmd_vel_updated = Signal(object)


class GuiNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("ros2_gui_node")
        self.bridge = bridge
        self.create_subscription(LaserScan, "/scan", self._lidar_cb, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._cmd_vel_cb, 10)
        self.get_logger().info("GUI node started")

    def _lidar_cb(self, msg):
        self.bridge.lidar_updated.emit(msg)

    def _cmd_vel_cb(self, msg):
        self.bridge.cmd_vel_updated.emit(msg)


# ─────────────────────────────────────────
# Motor indicator widget
# ─────────────────────────────────────────
class MotorIndicator(QWidget):
    """Binary bar indicator for a single motor channel.

    Shows a row of cells that light up based on power level.
    Supports forward (bright) and reverse (dim accent) display.
    """
    CELLS = 10

    LERP_SPEED = 0.15   # how fast the display chases the target per tick

    def __init__(self, label="MTR"):
        super().__init__()
        self._label = label
        self._target = 0.0      # where we want to be
        self._display = 0.0     # where we are visually
        self._firing = False
        self.setMinimumHeight(32)
        self.setMaximumHeight(40)

    def set_value(self, v):
        self._target = max(-1.0, min(1.0, v))
        self._firing = abs(v) > 0.02

    def tick(self):
        """Call each frame to animate toward target."""
        diff = self._target - self._display
        if abs(diff) < 0.005:
            self._display = self._target
        else:
            self._display += diff * self.LERP_SPEED
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # label
        p.setPen(QColor(TEXT_DIM))
        p.setFont(QFont("Menlo", 8, QFont.Bold))
        label_w = 50
        p.drawText(0, 0, label_w, h, Qt.AlignVCenter | Qt.AlignLeft, self._label)

        # cells
        cell_area = w - label_w - 4
        cell_w = cell_area / self.CELLS
        gap = 2
        level = abs(self._display)
        lit_cells = int(level * self.CELLS + 0.5)
        reverse = self._display < -0.02

        for i in range(self.CELLS):
            x = label_w + i * cell_w + gap / 2
            rect = QPointF(x, 4)

            if i < lit_cells:
                color = QColor(MTR_REV) if reverse else QColor(MTR_ON)
                # brighter toward the end
                if i >= self.CELLS * 0.7:
                    color = color.lighter(130)
            else:
                color = QColor(MTR_OFF)

            p.setPen(Qt.NoPen)
            p.setBrush(color)
            p.drawRect(int(x), 4, int(cell_w - gap), h - 8)

        # border
        p.setPen(QPen(QColor(ACC_DIM), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(label_w, 2, int(cell_area), h - 4)

        # value text
        p.setPen(QColor(TEXT) if self._firing else QColor(TEXT_DIM))
        p.setFont(QFont("Menlo", 8))
        txt = f"{'REV ' if reverse else 'FWD '}{abs(self._display):.2f}" if self._firing else "IDLE"
        p.drawText(label_w, 0, int(cell_area), h,
                   Qt.AlignCenter | Qt.AlignVCenter, txt)

        p.end()


# ─────────────────────────────────────────
# Motor bank — shows L/R motors + throttle + steering
# ─────────────────────────────────────────
class MotorBank(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.left_mtr  = MotorIndicator("L  MTR")
        self.right_mtr = MotorIndicator("R  MTR")
        self.throttle  = MotorIndicator("THRTL")
        self.steering  = MotorIndicator("STEER")

        for ind in [self.left_mtr, self.right_mtr, self.throttle, self.steering]:
            layout.addWidget(ind)

    def update_from_twist(self, msg):
        lin = msg.twist.linear.x
        ang = msg.twist.angular.z

        # differential drive approximation
        left  = lin - ang * 0.5
        right = lin + ang * 0.5

        self.left_mtr.set_value(left)
        self.right_mtr.set_value(right)
        self.throttle.set_value(lin)
        self.steering.set_value(ang)

    def tick(self):
        for ind in [self.left_mtr, self.right_mtr, self.throttle, self.steering]:
            ind.tick()


# ─────────────────────────────────────────
# Radar view — live lidar polar plot
# ─────────────────────────────────────────
class RadarWidget(QWidget):
    """Top-down polar render of the lidar scan."""
    def __init__(self):
        super().__init__()
        self._xs = []
        self._ys = []
        self._max_range = 3.5
        self.setMinimumSize(200, 200)

    def set_scan(self, angles, ranges, max_range):
        self._max_range = max_range
        self._xs = [r * math.cos(a) for a, r in zip(angles, ranges)]
        self._ys = [r * math.sin(a) for a, r in zip(angles, ranges)]
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        scale = min(w, h) / 2 - 10
        mr = self._max_range if self._max_range > 0 else 3.5

        # grid rings
        p.setPen(QPen(QColor(ACC_LO), 1))
        for i in range(1, 5):
            r = scale * i / 4
            p.drawEllipse(QPointF(cx, cy), r, r)

        # grid cross
        p.drawLine(int(cx - scale), int(cy), int(cx + scale), int(cy))
        p.drawLine(int(cx), int(cy - scale), int(cx), int(cy + scale))

        # range labels
        p.setPen(QPen(QColor(TEXT_DIM), 1))
        p.setFont(QFont("Menlo", 7))
        for i in range(1, 5):
            val = mr * i / 4
            r = scale * i / 4
            p.drawText(int(cx + 3), int(cy - r + 12), f"{val:.1f}m")

        # scan points
        if self._xs:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(ACCENT))
            for x, y in zip(self._xs, self._ys):
                px = cx + (x / mr) * scale
                py = cy - (y / mr) * scale
                p.drawEllipse(QPointF(px, py), 1.5, 1.5)

        # bot marker
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QPointF(cx, cy), 4, 4)

        p.end()


# ─────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────
def _make_panel(title_text):
    frame = QFrame()
    frame.setStyleSheet(f"""
        QFrame {{
            background: {PANEL_BG};
            border: 1px solid {ACC_DIM};
        }}
    """)
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(10, 6, 10, 6)
    title = QLabel(title_text)
    title.setFont(QFont("Menlo", 10, QFont.Bold))
    title.setStyleSheet(f"color: {ACCENT}; border: none;")
    layout.addWidget(title)
    return frame, layout, title


def _make_plot(labels, colors):
    pw = pg.PlotWidget()
    pw.setBackground(PANEL_BG)
    pw.showGrid(x=True, y=True, alpha=0.25)
    pw.getAxis("bottom").setPen(pg.mkPen(ACC_DIM))
    pw.getAxis("left").setPen(pg.mkPen(ACC_DIM))
    pw.getAxis("bottom").setTextPen(pg.mkPen(TEXT_DIM))
    pw.getAxis("left").setTextPen(pg.mkPen(TEXT_DIM))
    pw.getAxis("bottom").setGrid(200)
    pw.getAxis("left").setGrid(200)
    pw.setMouseEnabled(x=False, y=False)
    pw.hideButtons()

    curves = {}
    for label, color in zip(labels, colors):
        curves[label] = pw.plot(pen=pg.mkPen(color=color, width=2), name=label)

    pw.addLegend(
        offset=(10, 5),
        brush=pg.mkBrush(PANEL_BG),
        pen=pg.mkPen(ACC_DIM),
        labelTextColor=TEXT,
    )
    return pw, curves


# ─────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.setWindowTitle("MALEFICENT // ROS2 MONITOR")
        self.resize(1060, 720)
        self.setStyleSheet(STYLESHEET)

        bridge.lidar_updated.connect(self._on_lidar)
        bridge.cmd_vel_updated.connect(self._on_cmd_vel)

        # rolling buffers
        self._lidar_min = collections.deque(maxlen=HISTORY)
        self._lidar_max = collections.deque(maxlen=HISTORY)
        self._vel_throttle = collections.deque(maxlen=HISTORY)
        self._vel_steering = collections.deque(maxlen=HISTORY)
        self._last_scan = None

        # ── root layout ──────────────────
        root = QWidget()
        self.setCentralWidget(root)
        grid = QGridLayout(root)
        grid.setSpacing(8)
        grid.setContentsMargins(14, 14, 14, 14)

        # ── ROW 0: header ────────────────
        header = QLabel("MALEFICENT")
        header.setFont(QFont("Menlo", 36, QFont.Black))
        header.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        header.setStyleSheet(f"color: {ACCENT}; letter-spacing: 8px;")
        grid.addWidget(header, 0, 0, 1, 2)

        sub = QLabel("ROS2 MONITOR  //  LIVE TELEMETRY")
        sub.setFont(QFont("Menlo", 10))
        sub.setStyleSheet(f"color: {TEXT_DIM};")
        grid.addWidget(sub, 0, 2, 1, 1, Qt.AlignRight | Qt.AlignBottom)

        # ── ROW 1 left: lidar ────────────
        lf, ll, _ = _make_panel("LIDAR  RANGE")
        self.lidar_value = QLabel("waiting...")
        self.lidar_value.setFont(QFont("Menlo", 12))
        self.lidar_value.setStyleSheet(f"color: {ACCENT}; border: none;")
        ll.addWidget(self.lidar_value)
        self.lidar_pw, self.lidar_curves = _make_plot(
            ["min", "max"], [ACCENT, ACC_DIM])
        ll.addWidget(self.lidar_pw)
        grid.addWidget(lf, 1, 0)

        # ── ROW 1 center: cmd_vel ────────
        cf, cl, _ = _make_panel("CMD_VEL  LINEAR")
        self.vel_value = QLabel("waiting...")
        self.vel_value.setFont(QFont("Menlo", 12))
        self.vel_value.setStyleSheet(f"color: {ACCENT}; border: none;")
        cl.addWidget(self.vel_value)
        self.vel_pw, self.vel_curves = _make_plot(
            ["throttle", "steering"], [ACCENT, "#9444A0"])
        cl.addWidget(self.vel_pw)
        grid.addWidget(cf, 1, 1)

        # ── ROW 1 right: motor bank ──────
        mf, ml, _ = _make_panel("MOTOR  SIGNALS")
        self.motor_bank = MotorBank()
        ml.addWidget(self.motor_bank)
        ml.addStretch()
        grid.addWidget(mf, 1, 2)

        # ── ROW 2: radar ─────────────────
        rf, rl, _ = _make_panel("SCAN  RADAR")
        self.radar = RadarWidget()
        rl.addWidget(self.radar)
        grid.addWidget(rf, 2, 0, 1, 2)

        # ── ROW 3: status bar ────────────
        status_bar = QHBoxLayout()
        tags = ["SYS:NOMINAL", "SCAN:ACTIVE", "NAV:STANDBY", "COMM:LINKED"]
        for t in tags:
            lbl = QLabel(f"[ {t} ]")
            lbl.setFont(QFont("Menlo", 8))
            lbl.setStyleSheet(f"color: {TEXT_DIM};")
            status_bar.addWidget(lbl)
        status_bar.addStretch()
        sig = QLabel("MALEFICENT  v1.0")
        sig.setFont(QFont("Menlo", 8))
        sig.setAlignment(Qt.AlignRight)
        sig.setStyleSheet(f"color: {TEXT_DIM};")
        status_bar.addWidget(sig)

        status_w = QWidget()
        status_w.setLayout(status_bar)
        grid.addWidget(status_w, 3, 0, 1, 3)

        # column stretch
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 3)
        grid.setColumnStretch(2, 2)
        grid.setRowStretch(1, 2)
        grid.setRowStretch(2, 2)

        # 20 Hz refresh
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

        self._tick_count = 0

    # ── callbacks ─────────────────────────
    def _on_lidar(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            return
        mn, mx = min(valid), max(valid)
        self._lidar_min.append(mn)
        self._lidar_max.append(mx)
        self.lidar_value.setText(f"MIN {mn:.2f}m    MAX {mx:.2f}m")
        self._last_scan = msg

    def _on_cmd_vel(self, msg):
        self._vel_throttle.append(msg.twist.linear.x)
        self._vel_steering.append(msg.twist.angular.z)
        self.vel_value.setText(
            f"THRTL {msg.linear.x:+.2f}   "
            f"STEER {msg.angular.z:+.2f}")

        # update motor bank
        self.motor_bank.update_from_twist(msg)

    def _tick(self):
        if self._lidar_min:
            self.lidar_curves["min"].setData(list(self._lidar_min))
            self.lidar_curves["max"].setData(list(self._lidar_max))
        if self._vel_throttle:
            self.vel_curves["throttle"].setData(list(self._vel_throttle))
            self.vel_curves["steering"].setData(list(self._vel_steering))
        self.motor_bank.tick()

        # radar
        if self._last_scan is not None:
            msg = self._last_scan
            n = len(msg.ranges)
            angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
            valid_a, valid_r = [], []
            for a, r in zip(angles, msg.ranges):
                if msg.range_min < r < msg.range_max:
                    valid_a.append(a)
                    valid_r.append(r)
            self.radar.set_scan(valid_a, valid_r, msg.range_max)


# ─────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    bridge = RosBridge()
    node   = GuiNode(bridge)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    window = MainWindow(bridge)
    window.show()

    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
