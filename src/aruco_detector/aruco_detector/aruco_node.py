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
        self.marker_length = 0.05  # Marker size in meters

        # ArUco dictionary and detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Camera intrinsics (populated from /camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # ROS interfaces
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

        self.get_logger().info("ArucoDetector node initialized")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics loaded")

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        # Convert image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs
                )

                # Publish pose
                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = "camera_link"
                pose.pose.position.x = float(tvec[0][0][0])
                pose.pose.position.y = float(tvec[0][0][1])
                pose.pose.position.z = float(tvec[0][0][2])

                rotM, _ = cv2.Rodrigues(rvec[0])
                q = self.rotation_matrix_to_quaternion(rotM)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.pose_pub.publish(pose)

                # Draw on frame
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.03)

                text = f"ID: {ids[i]} x:{pose.pose.position.x:.2f} y:{pose.pose.position.y:.2f} z:{pose.pose.position.z:.2f}"
                center_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2.0)
                center_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2.0)
                cv2.putText(frame, text, (center_x - 60, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the image with annotations
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        imS = cv2.resize(frame, (960, 540))
        cv2.imshow("output", imS)
        cv2.waitKey(1)

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
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
