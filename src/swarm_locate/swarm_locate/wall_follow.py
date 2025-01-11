import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor

class WallTracing(Node):
    def __init__(self, robot_namespace: str):
        super().__init__(f'{robot_namespace}_wall_tracing')
        self.get_logger().info(f"the bot tracing the wall is {self.get_name()}")
        # Publisher for cmd_vel within each robot's namespace
        self.cmd_vel_pub = self.create_publisher(Twist, f'{robot_namespace}/cmd_vel', 10)
        
        # Subscription to ball detection (shared topic for all robots)
        self.ball_found_sub = self.create_subscription(
            Bool, '/ball_found', self.ball_found_callback, 10)
        
        self.ball_found = False
        
        # Subscription to laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan, f'{robot_namespace}/scan', self.laser_callback, 10)
        
        self.twist = Twist()
        
        # Timer for wall-following behavior
        self.timer = self.create_timer(0.1, self.follow_wall)

    def ball_found_callback(self, msg):
        """Handle ball detection signal to stop robot."""
        if msg.data:
            self.ball_found = True
            self.stop_robot()

    def laser_callback(self, scan):
        """Implement basic wall-following logic using scan ranges."""
        # Example: follow the left wall
        front = min(min(scan.ranges[:10]), min(scan.ranges[-10:]))
        left = min(scan.ranges[45:90])

        # Obstacle avoidance logic
        if front < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.0  # Turn to avoid obstacle
        elif left > 0.5:
            self.twist.linear.x = 0.3  # Move forward
            self.twist.angular.z = 0.3  # Turn towards wall
        else:
            self.twist.linear.x = 0.3  # Move forward
            self.twist.angular.z = 0.0  # Move straight

    def follow_wall(self):
        """Publish twist command to move robot, unless ball is found."""
        if not self.ball_found:
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info(f"Published twist for {self.get_name()}: {self.twist}")

    def stop_robot(self):
        """Stop the robot if the ball is found."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)

    # List of robot namespaces
    robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    # Create a node for each robot with its unique namespace
    node_0 = WallTracing(robots[0])
    node_1 = WallTracing(robots[1])
    node_2 = WallTracing(robots[2])
    node_3 = WallTracing(robots[3])

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
