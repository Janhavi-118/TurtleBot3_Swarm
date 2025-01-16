import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.node import Node
import time
import math
import numpy as np

class BallFoundSubscriber(Node):
    def __init__(self):
        super().__init__('ball_found_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/ball_found',
            self.ball_found_callback,
            10
        )
        self.ball_found_namespace = None

    def ball_found_callback(self, msg):
        self.ball_found_namespace = msg.data
        self.get_logger().info(f"Ball found by: {self.ball_found_namespace}")
    
class LaserScanSubscriber(Node):
    def __init__(self, robot_namespace):
        super().__init__(f'{robot_namespace}_laser_scan_subscriber')
        self.get_logger().info(f"The laser scan subscriber is made for {robot_namespace}")
        self.robot_namespace = robot_namespace
        self.laser_ranges = []
        self.subscription = self.create_subscription(
            LaserScan,
            f'/{robot_namespace}/scan',
            self.laser_scan_callback,
            10
        )

    def laser_scan_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
    
    
class CurrentPoseSubscriber(Node):
    def __init__(self, robot_namespace):
        super().__init__(f'{robot_namespace}_current_pose_subscriber')
        self.get_logger().info(f"The current pose subscriber is made for {robot_namespace}")
        self.robot_namespace = robot_namespace
        self.current_pose = None
        self.subscription = self.create_subscription(
            Odometry,
            f'/{robot_namespace}/odom',
            self.current_pose_callback,
            10
        )

    def current_pose_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"Received current pose: x={self.current_pose.position.x}, y={self.current_pose.position.y}")


class GoalPoseSubscriber(Node):
    def __init__(self, robot_namespace: String, ball_found_namespace:String):
        super().__init__(f'{robot_namespace}_goal_pose_subscriber')
        self.get_logger().info(f"The goal pose subscriber is made for {robot_namespace}")

        self.robot_namespace         = robot_namespace
        self.ball_found_by           = ball_found_namespace
        self.goal_pose               = Odometry()
        self.goal_pose_sub           = self.create_subscription(Odometry, '/goal_pose', self.goal_pose_callback, 10)


    def goal_pose_callback(self, msg: Odometry):
        self.goal_pose = msg
        self.get_logger().info(f"Received new goal pose for {self.robot_namespace}: x={self.goal_pose.pose.pose.position.x}, y={self.goal_pose.pose.pose.position.y}")
    
class DWAController(Node):
    def __init__(self, robot_namespace:String, goal_pose_subscriber: GoalPoseSubscriber, current_pose_subscriber: CurrentPoseSubscriber, laser_scan_subscriber: LaserScanSubscriber):
        super().__init__(f'{robot_namespace}_dwa_controller')
        self.get_logger().info(f"The DWA controller is made for {robot_namespace}")
        self.robot_namespace = robot_namespace
        self.current_pose_subscriber = current_pose_subscriber
        self.laser_scan_subscriber = laser_scan_subscriber
        self.goal_pose_subscriber = goal_pose_subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{robot_namespace}/cmd_vel', 10)
        self.dt = 0.05  # seconds
        self.horizon = 1.0  # seconds

        self.alpha_goal = 1.0
        self.alpha_safety = 1.2
        self.alpha_velocity = 0.5

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        self.goal_x = -3.0
        self.goal_y = 1.0

        self.max_linear_velocity = 0.5  # m/s
        self.min_linear_velocity = -0.5  # m/s
        self.max_angular_velocity = 0.2  # rad/s
        self.min_angular_velocity = -0.2  # rad/s
        self.max_linear_acceleration = 0.5  # m/s^2
        self.max_angular_acceleration = 0.5  # rad/s^2

        if self. goal_pose_subscriber.ball_found_by == self.robot_namespace:
            self.timer1 = self.create_timer(0.1, self.stop_callback)
        else:
            self.timer = self.create_timer(0.8, self.dwa_callback)

        
    def stop_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def dwa_callback(self):
        best_v = 0.0
        best_w = 0.0
        min_cost = float('inf')
        current_distance = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
        if current_distance < 0.5:
            self.stop_callback()
            return
        
        self.current_x = self.current_pose_subscriber.current_pose.position.x
        self.current_y = self.current_pose_subscriber.current_pose.position.y
        orientation = self.current_pose_subscriber.current_pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.get_logger().info(f"{self.robot_namespace} moving towards the goal pose x = {self.goal_x}, y = {self.goal_y}")
        for v in np.arange(self.min_linear_velocity, self.max_linear_velocity, 0.05):
            for w in np.arange(self.min_angular_velocity, self.max_angular_velocity, 0.05):
                trajectory = self.simulate_trajectory(v, w)

                cost = self.evaluate_cost(trajectory)

                if cost < min_cost:
                    min_cost = cost
                    best_v = v
                    best_w = w
        
        best_v = max(min(best_v, self.max_linear_velocity), self.min_linear_velocity)
        best_w = max(min(best_w, self.max_angular_velocity), self.min_angular_velocity)
        self.current_linear_velocity = best_v
        self.current_angular_velocity = best_w
        cmd_vel = Twist()
        cmd_vel.linear.x = best_v
        cmd_vel.angular.z = best_w
        self.cmd_vel_pub.publish(cmd_vel)
    
    def simulate_trajectory(self, v, w):
        trajectory = []
        x = self.current_pose_subscriber.current_pose.position.x
        y = self.current_pose_subscriber.current_pose.position.y
        orientation = self.current_pose_subscriber.current_pose.orientation
        _, _, theta = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        for t in np.arange(0, self.horizon, self.dt):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append((x, y))

        return trajectory

    def evaluate_cost(self, trajectory):
        goal_cost = 0.0
        safety_cost = 0.0
        velocity_cost = 0.0

        final_point = trajectory[-1]
        goal_distance = math.sqrt((final_point[0] - self.goal_x)**2 + (final_point[1] - self.goal_y)**2)
        current_distance = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
        if goal_distance > current_distance:
            goal_cost = self.alpha_goal * goal_distance * 10  # Heavy penalty
        else:
            goal_cost = self.alpha_goal * goal_distance

        goal_theta = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        heading_error = abs(goal_theta - self.current_theta)
        heading_cost = self.alpha_goal * heading_error

        for point in trajectory:
            min_distance = self.check_for_collisions(point[0], point[1])
            safety_cost += self.alpha_safety / min_distance if min_distance > 0 else float('inf')
        
        velocity_cost = self.alpha_velocity * ((self.max_linear_velocity - self.current_linear_velocity) + abs(self.current_angular_velocity))

        total_cost = goal_cost + safety_cost + velocity_cost + heading_cost
        return total_cost
    
    def check_for_collisions(self, x, y):
        min_distance = float('inf')
        self.current_x = self.current_pose_subscriber.current_pose.position.x
        self.current_y = self.current_pose_subscriber.current_pose.position.y
        orientation = self.current_pose_subscriber.current_pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if self.laser_scan_subscriber.laser_ranges:
            for i, distance in enumerate(self.laser_scan_subscriber.laser_ranges):
                angle = self.laser_scan_subscriber.angle_min + i * self.laser_scan_subscriber.angle_increment
                obs_x = self.current_x + distance * math.cos(self.current_theta + angle)
                obs_y = self.current_y + distance * math.sin(self.current_theta + angle)
                dist_to_point = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                min_distance = min(min_distance, dist_to_point)

        return min_distance  # Avoid division by zero
    
    def euler_from_quaternion(self,quaternion):
        x, y, z, w = quaternion

        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

        
def main(args=None):
    rclpy.init(args=args)
    ball_found_subscriber = BallFoundSubscriber()
    robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    goal_nodes = []
    curr_nodes = []
    executor = MultiThreadedExecutor()
    executor.add_node(ball_found_subscriber)
    def check_ball_found():
        if ball_found_subscriber.ball_found_namespace in ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']:
            curr_nodes = [CurrentPoseSubscriber(robot) for robot in robots]
            laser_nodes = [LaserScanSubscriber(robot) for robot in robots]
            goal_nodes = [GoalPoseSubscriber(robots[i], ball_found_subscriber.ball_found_namespace) for i in range (4)]
            dwa_nodes = [DWAController(robots[i], goal_nodes[i], curr_nodes[i], laser_nodes[i]) for i in range (4)]
            for node in curr_nodes:
                ball_found_subscriber.get_logger().info(f"Ball found! Starting curr pose nodes {node.robot_namespace}.")
                executor.add_node(node)
            for node in laser_nodes:
                ball_found_subscriber.get_logger().info(f"Ball found! Starting laser scan nodes {node.robot_namespace}.")
                executor.add_node(node)
            for node in goal_nodes:
                ball_found_subscriber.get_logger().info(f"Ball found! Starting goal pose nodes {node.robot_namespace}.")
                executor.add_node(node)
            for node in dwa_nodes:
                ball_found_subscriber.get_logger().info(f"Ball found! Starting dwa nodes {node.robot_namespace}.")
                executor.add_node(node)
            ball_found_subscriber.destroy_timer(timer2)
    timer2 = ball_found_subscriber.create_timer(0.5, check_ball_found)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in goal_nodes:
            node.destroy_node()
        for node in curr_nodes:
            node.destroy_node()
        rclpy.shutdown()