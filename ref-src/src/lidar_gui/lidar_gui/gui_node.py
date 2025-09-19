import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class LiDARVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_gui_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher to control robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Simulated robot state
        self.robot_pose = [0.0, 0.0, 0.0]
        self.current_target = 0
        self.patrolling = False

        # LiDAR data
        self.ranges = None
        self.angles = None
        self.target_path = []

        # Setup GUI
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("LiDAR Patrol GUI")
        self.window.resize(800, 600)

        self.canvas = FigureCanvas(Figure())
        self.ax = self.canvas.figure.add_subplot(111)
        self.canvas.mpl_connect('button_press_event', self.on_click)

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        self.window.setCentralWidget(central_widget)

        # Timer to refresh plot
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)  # 10 Hz

        # Timer for motion updates
        self.motion_timer = QTimer()
        self.motion_timer.timeout.connect(self.update_motion)
        self.motion_timer.start(100)  # 10 Hz

        self.window.show()

    def scan_callback(self, msg):
        print("[DEBUG] Received LaserScan with", len(msg.ranges), "points")

        self.ranges = np.array(msg.ranges)
        self.angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        min_len = min(len(self.ranges), len(self.angles))
        self.ranges = self.ranges[:min_len]
        self.angles = self.angles[:min_len]

    def update_plot(self):
        self.ax.clear()

        if self.ranges is None or self.angles is None:
            print("[DEBUG] No LiDAR data yet")
            return

        try:
            x = self.ranges * np.cos(self.angles)
            y = self.ranges * np.sin(self.angles)
            self.ax.plot(x, y, '.', markersize=2, color='black')
            print(f"[DEBUG] Plotting {len(x)} points")

            if self.target_path:
                tx, ty = zip(*self.target_path)
                self.ax.plot(tx, ty, 'ro-')

            rx, ry, rt = self.robot_pose
            self.ax.plot(rx, ry, 'bo')

            self.ax.legend(['Scan', 'Path', 'Robot'])
            self.canvas.draw()
        except Exception as e:
            print(f"[ERROR] update_plot failed: {e}")

    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            self.target_path.append((event.xdata, event.ydata))
            print(f"[INFO] Clicked: ({event.xdata:.2f}, {event.ydata:.2f})")
            if len(self.target_path) >= 3:
                self.patrolling = True
                print("[INFO] Patrolling started.")

    def update_motion(self):
        if not self.patrolling or len(self.target_path) == 0:
            return

        x, y, theta = self.robot_pose
        tx, ty = self.target_path[self.current_target]

        dx = tx - x
        dy = ty - y
        distance = np.hypot(dx, dy)
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - theta
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        twist = Twist()

        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.5 * angle_diff
        elif distance > 0.05:
            twist.linear.x = 0.1
        else:
            self.current_target += 1
            if self.current_target >= len(self.target_path):
                self.current_target = 0

        self.cmd_vel_pub.publish(twist)

        # Simulate motion
        dt = 0.1
        x += twist.linear.x * np.cos(theta) * dt
        y += twist.linear.x * np.sin(theta) * dt
        theta += twist.angular.z * dt
        self.robot_pose = [x, y, theta]

    def run(self):
        timer = QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.1))
        timer.start(10)
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    node = LiDARVisualizer()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
