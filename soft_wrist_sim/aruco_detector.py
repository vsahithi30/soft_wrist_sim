#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

# Camera intrinsics — update these once you run calibration in sim
# For now using approximate values for a typical 640x480 camera
CAMERA_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0]
], dtype=np.float64)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

# ArUco marker physical size in meters (match what you put in the world)
MARKER_SIZE = 0.05  # 5cm

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ArUco dictionary — DICT_4X4_50 is simple and robust
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()

        # Subscribe to simulated camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detected pose
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)

        # Publish annotated image for debugging
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 10)

        self.get_logger().info('ArUco detector node started, listening on /camera/image_raw')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            self.get_logger().info(f'Detected marker IDs: {ids.flatten().tolist()}')

            # Draw detections on debug image
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each detected marker
            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
                )

                # Draw axis on debug image
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS,
                                  rvec[0], tvec[0], MARKER_SIZE * 0.5)

                # Log position
                tx, ty, tz = tvec[0][0]
                self.get_logger().info(
                    f'Marker {marker_id} position — X: {tx:.3f}m  Y: {ty:.3f}m  Z: {tz:.3f}m'
                )

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.header.frame_id = 'wrist_camera_link'
                pose_msg.pose.position.x = float(tx)
                pose_msg.pose.position.y = float(ty)
                pose_msg.pose.position.z = float(tz)
                self.pose_pub.publish(pose_msg)

        # Always publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_pub_msg = debug_msg
        debug_pub_msg.header = msg.header
        self.debug_pub.publish(debug_pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
