#!/usr/bin/env python3
"""
Dock Controller — 3-phase visual-servo docking
Phase 1 SEARCHING  : rotate slowly until marker detected
Phase 2 APPROACHING: visual servo — align lateral, close depth
Phase 3 DOCKED     : stop, declare success
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg    import Bool, String
import numpy as np

# -- tuneable gains -------------------------------------------------------
K_ANG       = 1.8    # angular gain on lateral error  (rad/s per meter-offset)
K_LIN       = 0.35   # linear gain on depth error     (m/s per meter)
SEARCH_RATE = 0.35   # rotation speed while searching  (rad/s)
STOP_DIST   = 0.55   # stop when this close to marker  (m)
SLOW_DIST   = 1.20   # slow-approach threshold         (m)
MAX_LIN     = 0.20   # max linear speed                (m/s)
MAX_ANG     = 0.60   # max angular speed               (rad/s)
TIMEOUT     = 3.0    # seconds without detection → back to SEARCHING
# -------------------------------------------------------------------------


class DockController(Node):
    def __init__(self):
        super().__init__('dock_controller')

        self.declare_parameter('stop_distance', STOP_DIST)
        self.stop_dist = self.get_parameter('stop_distance').value

        self.state         = 'SEARCHING'
        self.lateral       = 0.0      # tvec[0] from camera frame (+ = marker right)
        self.depth         = 999.0    # tvec[2] from camera frame (forward distance)
        self.detected      = False
        self.last_detect_t = self.get_clock().now()

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',        10)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/aruco/pose',     self.pose_cb,     10)
        self.create_subscription(Bool,        '/aruco/detected', self.detected_cb, 10)

        # 20 Hz control loop
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Dock Controller ready. State: SEARCHING')

    # ------------------------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        self.lateral       = msg.pose.position.x   # camera-x = right
        self.depth         = msg.pose.position.z   # camera-z = depth
        self.last_detect_t = self.get_clock().now()

    def detected_cb(self, msg: Bool):
        self.detected = msg.data

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.state == 'DOCKED':
            self.cmd_pub.publish(Twist())
            self.status_pub.publish(String(data=self.state))
            return

        dt = (self.get_clock().now() - self.last_detect_t).nanoseconds * 1e-9
        fresh = (self.detected and dt < TIMEOUT)

        # ---- State transitions
        if self.state == 'SEARCHING' and fresh:
            self.state = 'APPROACHING'
            self.get_logger().info('Marker found → APPROACHING')
        elif self.state == 'APPROACHING':
            if not fresh:
                self.state = 'SEARCHING'
                self.get_logger().warn('Lost marker → SEARCHING')
            elif self.depth < self.stop_dist:
                self.state = 'DOCKED'
                self.cmd_pub.publish(Twist())
                self.get_logger().info('✓ DOCKED SUCCESSFULLY!')
                self.status_pub.publish(String(data='DOCKED'))
                return

        # ---- Control output
        cmd = Twist()
        if self.state == 'SEARCHING':
            cmd.angular.z = SEARCH_RATE
            cmd.linear.x  = 0.0

        elif self.state == 'APPROACHING':
            ang = -K_ANG * self.lateral
            cmd.angular.z = float(np.clip(ang, -MAX_ANG, MAX_ANG))

            depth_err = max(0.0, self.depth - self.stop_dist)
            lin = K_LIN * depth_err
            if self.depth < SLOW_DIST:
                lin = min(lin, MAX_LIN * 0.4)
            cmd.linear.x = float(np.clip(lin, 0.0, MAX_LIN))

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=self.state))

        self.get_logger().info(
            f'[{self.state}] depth={self.depth:.2f}m  lateral={self.lateral:.2f}m',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
