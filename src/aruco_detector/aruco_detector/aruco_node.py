import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf_transformations
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.marker_length = 0.18  # Marker size in meters

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # ROS interfaces
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs
                )

                pose_cam = PoseStamped()
                pose_cam.header.stamp = msg.header.stamp
                pose_cam.header.frame_id = 'camera_link'
                pose_cam.pose.position.x = float(tvec[0][0][0])
                pose_cam.pose.position.y = float(tvec[0][0][1])
                pose_cam.pose.position.z = float(tvec[0][0][2])

                rot_matrix, _ = cv2.Rodrigues(rvec[0])
                quat = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]
                pose_cam.pose.orientation.x = quat[0]
                pose_cam.pose.orientation.y = quat[1]
                pose_cam.pose.orientation.z = quat[2]
                pose_cam.pose.orientation.w = quat[3]

                # try:
                #     transform = self.tf_buffer.lookup_transform(
                #         'map',  # target
                #         'camera_link',  # source
                #         rclpy.time.Time()
                #     )

                # pose_map_stamped = do_transform_pose_stamped(pose_cam, transform)
                self.pose_pub.publish(pose_cam)

                self.get_logger().info(
                f" Marker {ids[i]} in map → x: {pose_cam.pose.position.x:.2f}, y: {pose_cam.pose.position.y:.2f}"
                )

                # except TransformException as e:
                #     self.get_logger().warn(f"TF transform failed: {str(e)}")

                cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.03)

                cx = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2.0)
                cy = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2.0)
                label = f"ID:{ids[i]} z:{pose_cam.pose.position.z:.2f}m"
                cv2.putText(frame, label, (cx - 60, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.namedWindow("Aruco Output", cv2.WINDOW_NORMAL)
        imS = cv2.resize(frame, (960, 540))
        cv2.imshow("Aruco Output", imS)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
