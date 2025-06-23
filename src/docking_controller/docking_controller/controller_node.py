import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int64
from sensor_msgs.msg import LaserScan, Imu
import tf_transformations
import math
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.navigator = BasicNavigator()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.aruco_pose_sub = self.create_subscription(PoseStamped, '/aruco_marker_pose', self.aruco_pose_callback, 10)
        self.aruco_id_sub = self.create_subscription(Int64, '/aruco_marker_id', self.aruco_id_callback, 10)

        self.yaw = 0.0
        self.front_distance = float('inf')

        self.current_aruco_id = None
        self.desired_marker_id = None  # <-- no param, wait for first ID
        self.marker_detected = False
        self.pose_cam = None

        self.state = 'START'
        self.goal_queue = [
            {'x': 3.3, 'y': -0.8, 'yaw': 0.0},
            {'x': 0.1, 'y': 4.0, 'yaw': 1.57}
        ]
        self.current_goal_index = 0

        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("Aruco Closed-Loop Docking Controller Initialized")

    def create_pose_stamped(self, x, y, yaw):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def send_goal(self, x, y, yaw):
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        goal = self.create_pose_stamped(x, y, yaw)
        self.navigator.waitUntilNav2Active()
        self.navigator.goToPose(goal)
        self.get_logger().info(f"Navigating to goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")

    def control_loop(self):
        if self.state == 'START':
            if self.current_goal_index < len(self.goal_queue):
                goal = self.goal_queue[self.current_goal_index]
                self.send_goal(goal['x'], goal['y'], goal['yaw'])
                self.state = 'NAVIGATING'
            else:
                self.get_logger().warn("No more waypoints in queue.")
        elif self.state == 'NAVIGATING':
            if self.navigator.isTaskComplete():
                self.get_logger().info("Goal reached.")
                self.current_goal_index += 1
                if self.current_goal_index < len(self.goal_queue):
                    self.state = 'START'
                else:
                    self.get_logger().info("Navigation complete. Waiting for ArUco marker...")
                    self.state = 'WAIT_FOR_MARKER'
        elif self.state == 'WAIT_FOR_MARKER':
            if self.desired_marker_id is not None and self.marker_detected and self.pose_cam is not None:
                self.get_logger().info(f"Marker {self.desired_marker_id} detected. Starting closed-loop docking.")
                self.state = 'DOCK'
        elif self.state == 'DOCK':
            self.perform_closed_loop_docking()
        elif self.state == 'IDLE':
            pass

    def perform_closed_loop_docking(self):
        if self.pose_cam is None:
            self.get_logger().warn("No ArUco pose received.")
            return

        target_z = 0.3
        target_x = 0.0
        z_threshold = 0.1
        x_threshold = 0.1
        final_lidar_stop = 0.3
        max_linear_speed = 0.2
        max_angular_speed = 0.3

        z = self.pose_cam.pose.position.z
        x = self.pose_cam.pose.position.x

        q = self.pose_cam.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        err_z = z - target_z
        err_x = x - target_x

        self.get_logger().info(f"[DOCK] Δz={err_z:.3f}, Δx={err_x:.3f}, yaw={yaw:.2f}, front_lidar={self.front_distance:.2f}")

        twist = Twist()

        aligned_x = abs(err_x) < x_threshold
        aligned_z = abs(err_z) < z_threshold

        if aligned_x and aligned_z and self.front_distance <= final_lidar_stop:
            self.get_logger().info("Docked successfully (pose + LIDAR).")
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'
            return

        if not aligned_x:
            twist.angular.z = -0.6 * err_x
            twist.angular.z = max(min(twist.angular.z, max_angular_speed), -max_angular_speed)
        else:
            twist.angular.z = 0.0

        if not aligned_z:
            twist.linear.x = 0.4 * err_z
            twist.linear.x = max(min(twist.linear.x, max_linear_speed), -max_linear_speed)
        else:
            twist.linear.x = 0.0

        if self.front_distance < 0.1:
            self.get_logger().warn("Obstacle too close. Aborting.")
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'
            return

        self.cmd_pub.publish(twist)

    def aruco_pose_callback(self, msg: PoseStamped):
        self.pose_cam = msg
        if self.marker_detected:
            self.get_logger().info(f"[POSE] z: {msg.pose.position.z:.2f}, x: {msg.pose.position.x:.2f}")

    def aruco_id_callback(self, msg: Int64):
        self.get_logger().info(f"[ARUCO DETECTED] ID: {msg.data}")
        if self.desired_marker_id is None:
            self.desired_marker_id = msg.data
            self.get_logger().info(f" Latched to ArUco ID: {msg.data}")
        if msg.data == self.desired_marker_id:
            self.marker_detected = True

    def lidar_callback(self, msg: LaserScan):
        center = len(msg.ranges) // 2
        window = msg.ranges[center - 3:center + 3]
        valid = [d for d in window if not math.isinf(d) and not math.isnan(d)]
        self.front_distance = min(valid) if valid else float('inf')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
