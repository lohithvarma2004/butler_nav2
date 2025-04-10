#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import math
from collections import deque
from rclpy.action import ActionClient

def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class ButlerManagerNav2(Node):
    def __init__(self):
        super().__init__('butler_manager_nav2')
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(String, '/order', self.order_callback, 10)
        self.create_subscription(String, '/confirmation_topic', self.confirmation_callback, 10)
        self.create_subscription(String, '/cancel', self.cancel_callback, 10)
        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # State
        self.state = 'at_home'
        self.task_queue = deque()
        self.current_task = None
        self.nav_goal_sent = False

        # Pose
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.goal_x = self.goal_y = None

        # Locations
        self.locations = {
            'home':    (0.0,  0.0),
            'kitchen': (-0.5, -1.5),
            'table_1': (6.0,  -1.0),
            'table_2': (6.0,  -6.0),
            'table_3': (6.0,   2.45),
        }

        # Timing
        self.waiting_timeout = 20.0
        self.waiting_start_time = None

        # Main loop
        self.create_timer(0.1, self.control_loop)

    # --- Callbacks ---
    def scan_callback(self, msg: LaserScan):
        if msg.ranges and min(msg.ranges) < 0.5:
            self.get_logger().warn("🚧 Obstacle detected! (Close range)")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def order_callback(self, msg: String):
        table = msg.data.strip().lower()
        if table not in self.locations:
            return self.get_logger().warn(f"⚠️ Invalid order: {table}")
        if table == self.current_task or table in self.task_queue:
            return self.get_logger().warn(f"⚠️ Duplicate order: {table}")
        self.task_queue.append(table)
        self.get_logger().info(f"📦 Received order for {table}")
        if self.state == 'at_home' and not self.nav_goal_sent:
            self.process_next_order()

    def confirmation_callback(self, msg: String):
        loc = msg.data.strip().lower()
        if self.state != 'waiting':
            return
        self.get_logger().info(f"Received confirmation: {loc}")
        if loc == 'kitchen':
            self.get_logger().info("✅ Kitchen confirmation received.")
            if self.current_task:
                self.move_to_location(self.current_task)
                self.state = 'to_location'
        elif loc == self.current_task:
            self.get_logger().info(f"✅ Delivery confirmed at {loc}.")
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_location(self.current_task)
                self.state = 'to_location'
            else:
                self.move_to_location('home')
                self.state = 'to_home'

    def cancel_callback(self, msg: String):
        canceled = msg.data.strip().lower()
        if canceled == self.current_task:
            self.get_logger().info(f"❌ Current task {canceled} canceled. Returning home.")
            self.move_to_location('home')
            self.state = 'to_home'
        elif canceled in self.task_queue:
            self.task_queue.remove(canceled)
            self.get_logger().info(f"❌ Canceled queued task: {canceled}")

    # --- State transitions ---
    def process_next_order(self):
        if not self.task_queue:
            self.state = 'at_home'
            return self.get_logger().info("✅ No pending orders.")
        self.current_task = self.task_queue.popleft()
        self.get_logger().info(f"🚀 Processing order: {self.current_task}")
        self.move_to_location('kitchen')
        self.state = 'to_location'

    def move_to_location(self, loc: str):
        gx, gy = self.locations[loc]
        self.goal_x, self.goal_y = gx, gy
        self.send_goal(gx, gy)
        self.nav_goal_sent = True
        self.waiting_start_time = None
        self.get_logger().info(f"🚗 Moving to {loc} at ({gx:.2f}, {gy:.2f})")

    def send_goal(self, gx: float, gy: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        dx, dy = gx - self.current_x, gy - self.current_y
        yaw = math.atan2(dy, dx)
        goal_pose.pose.orientation.z = math.sin(yaw/2.0)
        goal_pose.pose.orientation.w = math.cos(yaw/2.0)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.nav_to_pose_client.wait_for_server()
        fut = self.nav_to_pose_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            self.state = 'at_home'
            self.nav_goal_sent = False
            return
        self.get_logger().info("Navigation goal accepted. Waiting for result...")
        handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
            self.nav_goal_sent = False
            self.stop_navigation()
        else:
            self.get_logger().error(f"Navigation failed: {status}")
            self.state = 'at_home'
            self.nav_goal_sent = False

    def stop_navigation(self):
        self.get_logger().info("Navigation stopped.")
        now = self.get_clock().now()
        if self.state == 'to_location':
            self.state = 'waiting'
            self.waiting_start_time = now
            self.get_logger().info("⏳ Arrived. Waiting for confirmation/timeout...")
        elif self.state == 'to_home':
            self.state = 'at_home'
            self.get_logger().info("✅ Returned home.")
            if self.task_queue:
                self.process_next_order()

    def handle_timeout(self):
        self.get_logger().info("⏰ Waiting timed out. Handling next step.")
        if self.current_task == 'kitchen':
            self.move_to_location('home')
            self.state = 'to_home'
        else:
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_location(self.current_task)
                self.state = 'to_location'
            else:
                self.move_to_location('home')
                self.state = 'to_home'

    def control_loop(self):
        now = self.get_clock().now()
        if self.state == 'waiting' and self.waiting_start_time:
            elapsed = (now - self.waiting_start_time).nanoseconds / 1e9
            if elapsed > self.waiting_timeout:
                self.handle_timeout()
                return

        if self.state == 'at_home' and self.task_queue and not self.nav_goal_sent:
            self.get_logger().info("📢 Pending order found; starting next.")
            self.process_next_order()

def main(args=None):
    rclpy.init(args=args)
    node = ButlerManagerNav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
