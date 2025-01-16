import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from roboflow import Roboflow

class BotPoseSubscriber(Node):
    def __init__(self, robot_namespace):
        super().__init__(f'{robot_namespace}')
        self.get_logger().info(f"the bot pose subscriber is {self.get_name()}")

        self.subscription = self.create_subscription(
            Odometry,
            f'/{robot_namespace}/odom',
            self.bot_pose_callback,
            10
        )
        
        self.robot_poses = {}
        
    def bot_pose_callback(self, msg: Odometry):
        self.robot_poses[self.get_name()] = msg

    def get_robot_pose(self, robot_namespace):
        # Retrieve pose for a specific robot namespace
        return self.robot_poses.get(robot_namespace, None)
class BallDetection(Node):
    def __init__(self, robot_namespace, pose_subscriber:BotPoseSubscriber):
        super().__init__(f'{robot_namespace}_ball_detection')
        
        self.robot_namespace = robot_namespace
        self.pose_subscriber = pose_subscriber

        self.image_sub = self.create_subscription(
            Image, 
            f'/{robot_namespace}/camera/image_raw', 
            self.image_callback, 
            10,
        )

        self.get_logger().info(f"Subscribed to {self.image_sub.topic_name}")
        
        self.ball_found_pub = self.create_publisher(String, '/ball_found', 10)
        self.pose_pub = self.create_publisher(Odometry, '/goal_pose', 10)

        self.bridge = CvBridge()
        self.ball_found_by = None
        self.rf = Roboflow(api_key="PmxZX6G7dTur7WCKDRa4")
        self.project = self.rf.workspace().project("green-ball-v17ti")
        self.model = self.project.version(1).model

        self.current_frame = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        result = self.model.predict(cv_image, confidence=40, overlap=30).json()
        detections = result.get("predictions", [])

        for detection in detections:
            x_min = int(detection["x"] - detection["width"] / 2)
            y_min = int(detection["y"] - detection["height"] / 2)
            x_max = int(detection["x"] + detection["width"] / 2)
            y_max = int(detection["y"] + detection["height"] / 2)

            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(
                cv_image, 
                f"{detection['class']} ({detection['confidence']:.2f})", 
                (x_min, y_min - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 255, 0), 
                2,
            )

            self.get_logger().info(f"Detected {detection['class']} at ({detection['x']}, {detection['y']})")

            if detection["class"] == "Green Ball":
                self.ball_found_by = self.robot_namespace
                self.ball_found_pub.publish(String(data=f'{self.robot_namespace}'))
                self.publish_robot_pose()

        self.current_frame = cv_image


    def publish_robot_pose(self):
        robot_pose = self.pose_subscriber.get_robot_pose(self.robot_namespace)

        self.get_logger().info(f"Publishing robot pose for {self.robot_namespace}")
        if robot_pose:
            self.pose_pub.publish(robot_pose)
            self.get_logger().info(f"Published robot pose for {self.robot_namespace}")
        else:
            self.get_logger().warn(f"No pose data for {self.robot_namespace}")

def display_camera_feeds(nodes, grid_shape=(2, 2)):
    while rclpy.ok():
        rows, cols               = grid_shape
        feed_height, feed_width  = 240, 320 
        grid_image               = np.zeros((rows * feed_height, cols * feed_width, 3), dtype=np.uint8)

        for idx, node in enumerate(nodes):
            if node.current_frame is not None:
                resized_frame  = cv2.resize(node.current_frame, (feed_width, feed_height))
                row_idx        = idx // cols
                col_idx        = idx % cols
                grid_image[
                    row_idx * feed_height:(row_idx + 1) * feed_height,
                    col_idx * feed_width:(col_idx + 1) * feed_width
                ] = resized_frame

        cv2.imshow("Robot Camera Feeds", grid_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    robots         = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
    pose_nodes     = [BotPoseSubscriber(robot) for robot in robots]
    detect_nodes   = [BallDetection(robots[i], pose_nodes[i]) for i in range (4)]

    executor = MultiThreadedExecutor()
    for node in detect_nodes:
        executor.add_node(node)
    for node in pose_nodes:
        executor.add_node(node)

    def after_detection():
        flag = False
        for node in detect_nodes:
            if node.ball_found_by in ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']:
                flag = True
                break
        if flag:
            for node in detect_nodes:
                executor.remove_node(node)
            timer.destroy()
    
    timer = pose_nodes[0].create_timer(0.1, after_detection)

    try:
        import threading
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        display_camera_feeds(detect_nodes)

    except KeyboardInterrupt:
        pass
    finally:
        for node in detect_nodes:
            node.destroy_node()
        for node in pose_nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()