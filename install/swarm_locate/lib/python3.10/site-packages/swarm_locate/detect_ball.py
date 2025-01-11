import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

class BallDetection(Node):
    def __init__(self, robot_namespace):
        super().__init__(f'{robot_namespace}_ball_detection')
        
        # Subscribe to the robot's camera feed
        self.image_sub = self.create_subscription(Image, f'/{robot_namespace}/camera/image_raw', self.image_callback, 10)
        
        # Publisher for ball detection status
        self.ball_found_pub = self.create_publisher(Bool, '/ball_found', 10)
        
        # Publisher for the robot's pose
        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Convert ROS images to OpenCV
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert image to HSV for color filtering
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the green color range in HSV
        lower_green = (36, 100, 100)
        upper_green = (86, 255, 255)
        
        # Mask the image to extract the green color
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Ball found, publish the status
            self.ball_found_pub.publish(Bool(data=True))
            
            # If the ball is detected, get the robot's current pose
            self.publish_robot_pose()

    def publish_robot_pose(self):
        # Get the robot's pose (for simplicity, we will simulate this pose here)
        # In a real scenario, you can use the robot's actual pose from odometry or TF
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = 'base_link'  # Robot's frame of reference
        robot_pose.pose.position.x = 1.0  # Simulated position (X)
        robot_pose.pose.position.y = 2.0  # Simulated position (Y)
        robot_pose.pose.orientation.w = 1.0  # No rotation
        
        # Publish the robot's pose
        self.pose_pub.publish(robot_pose)
        #self.get_logger().info(f"Published pose: x={robot_pose.pose.position.x}, y={robot_pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    
    # List of robots, adjust the namespace accordingly
    robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    node_0 = BallDetection(robots[0])
    node_1 = BallDetection(robots[1])
    node_2 = BallDetection(robots[2])
    node_3 = BallDetection(robots[3])

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
