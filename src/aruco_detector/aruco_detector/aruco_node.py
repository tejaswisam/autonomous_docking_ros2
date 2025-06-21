import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.marker_length = 0.05  # meters

        # ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Camera parameters (fill these after calibration)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

    def camera_info_callback(self, msg):
        # Initialize once
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics loaded.')
            self.cam_info_sub.destroy()  # Only need once

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return  # Wait for camera info first

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)

                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = "camera_link"

                pose.pose.position.x = tvec[0][0][0]
                pose.pose.position.y = tvec[0][0][1]
                pose.pose.position.z = tvec[0][0][2]

                rotM, _ = cv2.Rodrigues(rvec)
                q = self.rotation_matrix_to_quaternion(rotM)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.pose_pub.publish(pose)
                self.get_logger().info(f"Marker {ids[i][0]} Pose: {pose.pose.position}")

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        q = np.empty(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            i = np.argmax(np.diagonal(R))
            if i == 0:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q[:3].tolist() + [q[3]]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
