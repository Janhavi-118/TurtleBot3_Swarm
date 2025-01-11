import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import time
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor

class BallFoundSubscriber(Node):
    def __init__(self):
        super().__init__('ball_found_subscriber')

        # Subscription to the common ball found topic
        self.subscription = self.create_subscription(
            Bool,
            '/ball_found',
            self.ball_found_callback,
            10
        )

        # Variable to store the ball detection status
        self.ball_found = False

    def ball_found_callback(self, msg: Bool):
        """Callback function to handle the ball detection signal."""
        self.ball_found = msg.data

class GoalPoseSubscriber(Node):
    def __init__(self, robot_namespace: str):
        super().__init__(f'{robot_namespace}_goal_pose_subscriber')

        # Subscription to the common goal pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Publisher to send movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{robot_namespace}/cmd_vel', 10)

        # Subscription to the laser scan data for obstacle avoidance
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            f'/{robot_namespace}/scan',
            self.laser_scan_callback,
            10
        )

        self.laser_ranges = []

    def laser_scan_callback(self, msg: LaserScan):
        """Callback function to handle laser scan data."""
        self.laser_ranges = msg.ranges  # Get the laser scan data (distances)


    def goal_pose_callback(self, msg: PoseStamped):
        """Callback function to handle the goal pose and navigation."""
        goal_pose = msg.pose
        self.get_logger().info(f"Received goal pose: x={goal_pose.position.x}, y={goal_pose.position.y}")

        move_cmd = Twist()

        # Logic for obstacle avoidance using laser scan data
        if self.is_obstacle_ahead():
            self.get_logger().info("Obstacle detected! Turning to avoid.")
            self.turn_90_degrees()
        else:
            # Move towards the goal if no obstacles
            move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
            self.cmd_vel_pub.publish(move_cmd)

    def is_obstacle_ahead(self):
        """Basic obstacle detection logic using laser scan data."""
        # Look at the front half of the robot's laser scan data (you can adjust the indices depending on the robot)
        front_range = self.laser_ranges[0:10] + self.laser_ranges[-10:]  # Check front 20 degrees
        min_distance = min(front_range)

        # If the minimum distance is less than a threshold, we assume there's an obstacle
        return min_distance < 0.5  # 0.5 meters threshold for obstacle detection

    def turn_90_degrees(self):
        """Make the robot turn 90 degrees (clockwise or counterclockwise) when obstacle detected."""
        turn_cmd = Twist()
        
        # Start rotating counterclockwise (you can change the direction to clockwise if needed)
        turn_cmd.angular.z = 0.5  # Angular velocity for rotation (counterclockwise)
        self.cmd_vel_pub.publish(turn_cmd)

        # Wait for the robot to turn 90 degrees
        time.sleep(1.5)  # Adjust this time based on robot's turning speed

        # Stop rotating after 90 degrees
        turn_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(turn_cmd)

        # Move forward again
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    ball_found_subscriber = BallFoundSubscriber()
    if ball_found_subscriber.ball_found:
        robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']  # List of robots
        node_0 = GoalPoseSubscriber(robots[0])
        node_1 = GoalPoseSubscriber(robots[1])
        node_2 = GoalPoseSubscriber(robots[2])
        node_3 = GoalPoseSubscriber(robots[3])

        executor = MultiThreadedExecutor()

        executor.add_node(node_0)
        executor.add_node(node_1)
        executor.add_node(node_2)
        executor.add_node(node_3)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # Shutdown nodes and the executor
            node_0.destroy_node()
            node_1.destroy_node()
            node_2.destroy_node()
            node_3.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()
