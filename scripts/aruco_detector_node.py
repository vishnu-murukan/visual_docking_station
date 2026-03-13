#!/usr/bin/env python3
"""
ArUco Detector Node
Detects ArUco marker (DICT_4X4_50, id=0) using the robot camera.
Publishes marker pose in camera frame for use by the dock controller.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('tag_size', 0.45)          # marker physical size in meters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('marker_id', 0)

        self.tag_size   = self.get_parameter('tag_size').value
        self.cam_topic  = self.get_parameter('camera_topic').value
        self.marker_id  = self.get_parameter('marker_id').value

        # Camera intrinsics (640x480, HFOV=80°)
        #   fx = fy = W / (2 * tan(HFOV/2)) = 640 / (2*tan(0.69813)) = 381.36
        self.camera_matrix = np.array([
            [381.36,    0.0,  320.0],
            [   0.0, 381.36,  240.0],
            [   0.0,    0.0,    1.0]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ArUco detector
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.adaptiveThreshWinSizeMin  = 3
        aruco_params.adaptiveThreshWinSizeMax  = 53
        aruco_params.adaptiveThreshWinSizeStep = 4
        aruco_params.minMarkerPerimeterRate    = 0.02
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # 3D object points for a square marker (OpenCV convention):
        # corners returned clockwise from top-left in image:
        # TL→TR→BR→BL  ↔  (-x,+y)→(+x,+y)→(+x,-y)→(-x,-y)
        h = self.tag_size / 2.0
        self.obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0]], dtype=np.float64)

        # CvBridge
        self.bridge = CvBridge()

        # Publishers
        self.pose_pub      = self.create_publisher(PoseStamped, '/aruco/pose',         10)
        self.detected_pub  = self.create_publisher(Bool,        '/aruco/detected',     10)
        self.debug_pub     = self.create_publisher(Image,       '/docking/debug_image', 10)

        # Subscribers
        self.create_subscription(Image, self.cam_topic, self.image_cb, 10)

        self.get_logger().info(f'ArUco Detector ready  ·  dict=4X4_50  id={self.marker_id}  size={self.tag_size}m')

    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        detected = False
        debug = frame.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            for i, corner in enumerate(corners):
                mid = int(ids[i][0])
                if mid != self.marker_id:
                    continue

                img_pts = corner[0].astype(np.float64)          # (4,2)
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_pts, img_pts,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                detected = True

                # Draw pose axis
                cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.tag_size * 0.4)

                # tvec in camera_optical_frame:
                #   x → right,  y → down,  z → depth (away from camera)
                lateral = float(tvec[0][0])
                depth   = float(tvec[2][0])

                # HUD overlay
                cx = int(np.mean(img_pts[:, 0]))
                cy = int(np.mean(img_pts[:, 1]))
                cv2.circle(debug, (cx, cy), 6, (0, 255, 0), -1)
                cv2.putText(debug, f'd={depth:.2f}m  lat={lateral:.2f}m',
                            (cx - 80, cy - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                # Convert rvec → quaternion
                R, _ = cv2.Rodrigues(rvec)
                quat = self._rot_to_quat(R)

                # Publish PoseStamped (pose in camera_optical_frame)
                ps = PoseStamped()
                ps.header = msg.header
                ps.header.frame_id = 'camera_optical_frame'
                ps.pose.position.x = lateral
                ps.pose.position.y = float(tvec[1][0])
                ps.pose.position.z = depth
                ps.pose.orientation.x = quat[0]
                ps.pose.orientation.y = quat[1]
                ps.pose.orientation.z = quat[2]
                ps.pose.orientation.w = quat[3]
                self.pose_pub.publish(ps)

                self.get_logger().info(
                    f'Marker {mid}: depth={depth:.3f}m  lateral={lateral:.3f}m',
                    throttle_duration_sec=1.0)
                break   # use first matching marker only

        if not detected:
            cv2.putText(debug, 'SEARCHING for ArUco marker...',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)

        self.detected_pub.publish(Bool(data=detected))
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    @staticmethod
    def _rot_to_quat(R):
        """Rotation matrix → quaternion [x,y,z,w]"""
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1] - R[1,2]) * s
            y = (R[0,2] - R[2,0]) * s
            z = (R[1,0] - R[0,1]) * s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
