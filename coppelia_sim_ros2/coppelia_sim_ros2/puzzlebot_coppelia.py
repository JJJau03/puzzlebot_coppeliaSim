import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import sys

client_path = '/home/jjj/Documents/Programs/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src'

if client_path not in sys.path:
    sys.path.append(client_path)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class RobotController(Node):
    def __init__(self):
        super().__init__('coppelia_ros_bridge')

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.camera_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Connect to CoppeliaSim
        time.sleep(2)
        try:
            client = RemoteAPIClient('127.0.0.1', 23000)
            self.sim = client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim')
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')
            exit(1)

        # Get joint handles
        self.left_joint = self.sim.getObject("/wheel_L_joint")
        self.right_joint = self.sim.getObject("/wheel_R_joint")
        self.wheel_radius = 0.5
        self.wheel_distance = 1.81

        # Initialize wheels to zero velocity
        self.sim.setJointTargetVelocity(self.left_joint, 0.0)
        self.sim.setJointTargetVelocity(self.right_joint, 0.0)

        # Setup camera if available
        self.vision_sensor = self.sim.getObject("/visionSensor")
        if self.vision_sensor == -1:
            self.get_logger().warning("Vision sensor not found! Camera disabled")
        else:
            self.camera_timer = self.create_timer(0.05, self.camera_callback)

    def cmd_vel_callback(self, msg):
        
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_distance / 2)
        v_right = v + (w * self.wheel_distance / 2)

        w_left = v_left / -self.wheel_radius
        w_right = v_right / self.wheel_radius

        self.sim.setJointTargetVelocity(self.left_joint, w_left)
        self.sim.setJointTargetVelocity(self.right_joint, w_right)

    def camera_callback(self):
        try:
            image, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if image:
                img = np.frombuffer(image, dtype=np.uint8)
                img = img.reshape((resolution[1], resolution[0], 3))
                img = cv2.cvtColor(np.flip(img, 0), cv2.COLOR_RGB2BGR)
                ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.camera_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()