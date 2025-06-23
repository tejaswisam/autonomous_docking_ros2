import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class ArucoDockStaticPose(Node):
    def __init__(self):
        super().__init__('aruco_dock_static_pose')

        self.pose_sub = self.create_subscription(PoseStamped, '/aruco_marker_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Flags and memory
        self.got_marker_pose = False
        self.marker_pose = None
        self.stage = 0  # 0=wait, 1=go to target, 2=align yaw, 3=lidar dock
        self.front_distance = float('inf')

        # Parameters
        self.offset_from_marker = 0.60  # 60 cm from marker
        self.lidar_dock_distance = 0.08
        self.distance_tolerance = 0.02
        self.angle_tolerance = 0.03  # radians

        # Gains
        self.linear_k = 0.5
        self.angular_k = 2.0
        self.max_linear = 0.25
        self.max_angular = 1.0

    def pose_callback(self, msg: PoseStamped):
        if not self.got_marker_pose:
            self.get_logger().info("📍 ArUco pose captured.")
            self.marker_pose = msg
            self.got_marker_pose = True
            self.stage = 1

    def lidar_callback(self, msg: LaserScan):
        # Average front range
        center = len(msg.ranges) // 2
        self.front_distance = min(msg.ranges[center - 3:center + 3])

    def control_loop(self):
        if not self.got_marker_pose or self.stage == 0:
            return

        twist = Twist()
        marker = self.marker_pose

        # Extract position and orientation
        px = marker.pose.position.x
        py = marker.pose.position.y
        pz = marker.pose.position.z
        q = marker.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Compute target point 60 cm in front of ArUco marker in its frame
        # In camera frame: forward = +z, right = +x (so +x in marker = -x in camera)
        # Step 1: Compute virtual docking point in camera frame
        target_x = px + self.offset_from_marker * math.sin(yaw)
        target_z = pz + self.offset_from_marker * math.cos(yaw)

        # Step 2: Compute vector from camera to target
        dx = target_x
        dz = target_z

        # Step 3: Use atan2 and distance to control
        angle_to_target = math.atan2(dx, dz)
        distance = math.sqrt(dx**2 + dz**2)


        # -------------------
        # Stage 1: Go to target point
        # -------------------
        if self.stage == 1:
            if distance < self.distance_tolerance and abs(angle_to_target) < self.angle_tolerance:
                self.get_logger().info("✅ Reached 60 cm point in front of marker.")
                self.stage = 2
                return
            twist.linear.x = min(self.max_linear, self.linear_k * distance)
            twist.angular.z = max(-self.max_angular, min(self.max_angular, self.angular_k * angle_to_target))

        # -------------------
        # Stage 2: Align yaw
        # -------------------
        elif self.stage == 2:
            if abs(yaw) < self.angle_tolerance:
                self.get_logger().info("✅ Yaw aligned with marker.")
                self.stage = 3
                return
            twist.angular.z = max(-self.max_angular, min(self.max_angular, self.angular_k * yaw))

        # -------------------
        # Stage 3: LIDAR-based docking
        # -------------------
        elif self.stage == 3:
            if self.front_distance < self.lidar_dock_distance:
                self.get_logger().info("✅ Final docking complete using LIDAR.")
                self.cmd_pub.publish(Twist())
                self.stage = 4
                return
            twist.linear.x = 0.1
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def run(self):
        self.create_timer(0.1, self.control_loop)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockStaticPose()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
