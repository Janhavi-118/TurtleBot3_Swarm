import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import time

class WallTracing(Node):
    def __init__(self, robot_namespace: str):
        super().__init__(f'{robot_namespace}_wall_tracing')
        self.get_logger().info(f"the bot tracing the wall is {self.get_name()}")

        self.cmd_vel_pub       = self.create_publisher(Twist, f'{robot_namespace}/cmd_vel', 10)
        self.ball_found_sub    = self.create_subscription(String, '/ball_found', self.ball_found_callback, 10)
        
        self.ball_found        = False
        
        self.laser_sub         = self.create_subscription(LaserScan, f'{robot_namespace}/scan', self.laser_callback, 10)
        
        self.twist             = Twist()
        
        self.timer             = self.create_timer(0.1, self.follow_wall)

    def ball_found_callback(self, msg):
        if msg.data in ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']:
            self.ball_found = True
            self.stop_robot()

    def laser_callback(self, scan):
        front = min(min(scan.ranges[:10]), min(scan.ranges[-10:]))
        left = min(scan.ranges[45:90])
        time.sleep(12)
        if front < 0.5:
            self.twist.linear.x    = 0.0
            self.twist.angular.z   = -1.0  
        elif left > 0.5:
            self.twist.linear.x    = 0.3  
            self.twist.angular.z   = 0.3  
        else:
            self.twist.linear.x    = 0.3  
            self.twist.angular.z   = 0.0  

    def follow_wall(self):
        if not self.ball_found:
            self.cmd_vel_pub.publish(self.twist)

    def stop_robot(self):
        self.twist.linear.x   = 0.0
        self.twist.angular.z  = 0.0
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)

    robots       = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    wall_nodes   = [WallTracing(robot) for robot in robots]

    executor = MultiThreadedExecutor()
    for node in wall_nodes:
        executor.add_node(node)
    
    def check_ball_found():
        for node in wall_nodes:
            if node.ball_found:
                for node in wall_nodes:
                    node.get_logger().info(f"Ball found! Stopping {node}.")
                    node.destroy_timer(node.timer)
                    node.stop_robot()
                    node.destroy_node()
                break
    
    timer = wall_nodes[3].create_timer(0.5, check_ball_found)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in wall_nodes:
            node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
