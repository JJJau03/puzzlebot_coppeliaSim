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

class SmoothPIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()
        self.integral_limit = max_output * 0.3
        self.output_smoothing = 0.2
        self.smoothed_output = 0
        
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0:
            return self.smoothed_output
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0 and self.previous_time > 0:
            d_term = self.kd * (error - self.previous_error) / dt
        else:
            d_term = 0
            
        # Compute raw output
        output = p_term + i_term + d_term
        output = np.clip(output, -self.max_output, self.max_output)
        
        # Smooth the output
        self.smoothed_output = (self.output_smoothing * output + 
                               (1 - self.output_smoothing) * self.smoothed_output)
        
        self.previous_error = error
        self.previous_time = current_time
        
        return self.smoothed_output
        
    def reset(self):
        self.integral = 0
        self.previous_error = 0
        self.smoothed_output = 0

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

        # Setup camera
        self.vision_sensor = self.sim.getObject("/visionSensor")
        if self.vision_sensor == -1:
            self.get_logger().error("Vision sensor not found!")
            exit(1)
            
        # PID controllers with smoother parameters
        self.pid_rotation = SmoothPIDController(kp=0.25, ki=0.4, kd=0.15, max_output=0.8)
        self.pid_distance = SmoothPIDController(kp=0.3, ki=0.0, kd=0.1, max_output=0.8)
        
        # Target position (center of image)
        _, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
        self.target_x = resolution[0] // 2
        self.target_y = resolution[1] // 2
        
        # Ball detection parameters
        self.green_lower = np.array([35, 50, 50])
        self.green_upper = np.array([85, 255, 255])
        
        # Movement parameters
        self.rotation_deadzone = 30  # pixels
        self.min_ball_radius = 5    # Minimum radius to approach
        self.max_ball_radius = 100   # Stop when ball reaches this size
        self.approach_threshold = 30 # Only approach when ball is within this many pixels of center
        
        # State machine states
        self.SEARCHING = 0
        self.ALIGNING = 1
        self.APPROACHING = 2
        self.REACHED = 3
        self.state = self.SEARCHING
        self.state_start_time = time.time()
        self.max_state_time = 3.0  # seconds
        
        # Smoothing variables
        self.filtered_ball_x = None
        self.filtered_ball_y = None
        self.filter_alpha = 0.4
        
        # Search parameters
        self.search_rotation_speed = 0.4  # Rad/s for searching
        self.search_direction = 1  # 1 for right, -1 for left
        
        # Start camera timer
        self.camera_timer = self.create_timer(0.1, self.camera_callback)  # Reduced frequency

    def cmd_vel_callback(self, msg):
        # Manual override if needed
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_distance / 2)
        v_right = v + (w * self.wheel_distance / 2)

        w_left = v_left / -self.wheel_radius
        w_right = v_right / self.wheel_radius

        self.sim.setJointTargetVelocity(self.left_joint, w_left)
        self.sim.setJointTargetVelocity(self.right_joint, w_right)

    def detect_green_ball(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            return (int(x), int(y)), int(radius)
        return None, None

    def camera_callback(self):
        try:
            image, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not image:
                return
                
            img = np.frombuffer(image, dtype=np.uint8)
            img = img.reshape((resolution[1], resolution[0], 3))
            img = cv2.cvtColor(np.flip(img, 0), cv2.COLOR_RGB2BGR)
            
            # Initialize velocities (FIX: ensures variables are always defined)
            linear_vel = 0.0
            angular_vel = 0.0
            state_text = "UNKNOWN"
            
            # Detect green ball
            (x, y), radius = self.detect_green_ball(img)
            
            # Filter ball position
            if x is not None:
                if self.filtered_ball_x is None:
                    self.filtered_ball_x = x
                    self.filtered_ball_y = y
                else:
                    self.filtered_ball_x = self.filter_alpha * x + (1-self.filter_alpha) * self.filtered_ball_x
                    self.filtered_ball_y = self.filter_alpha * y + (1-self.filter_alpha) * self.filtered_ball_y
                x, y = self.filtered_ball_x, self.filtered_ball_y
                
                # State machine logic
                if radius >= self.max_ball_radius:
                    new_state = self.REACHED
                elif abs(x - self.target_x) < self.approach_threshold and radius > self.min_ball_radius:
                    new_state = self.APPROACHING
                elif abs(x - self.target_x) < self.rotation_deadzone:
                    new_state = self.ALIGNING
                else:
                    new_state = self.ALIGNING
                    
                # Reset timer if state changed
                if new_state != self.state:
                    self.state_start_time = time.time()
                    self.state = new_state
                    
                # State timeout
                if time.time() - self.state_start_time > self.max_state_time:
                    self.state = self.SEARCHING
                    self.state_start_time = time.time()
                
                # State actions
                if self.state == self.ALIGNING:
                    output_rot = self.pid_rotation.compute(x - self.target_x)
                    linear_vel = 0.0
                    angular_vel = -output_rot * 0.5
                    state_text = "ALIGNING"
                    
                elif self.state == self.APPROACHING:
                    output_rot = self.pid_rotation.compute(x - self.target_x)
                    output_dist = self.pid_distance.compute(self.max_ball_radius - radius)
                    linear_vel = output_dist * 0.6
                    angular_vel = -output_rot * 0.3
                    state_text = "APPROACHING"
                    
                elif self.state == self.REACHED:
                    linear_vel = 0.0
                    angular_vel = 0.0
                    self.pid_rotation.reset()
                    self.pid_distance.reset()
                    state_text = "REACHED"
                    
                # Draw visualization
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
                
            else:  # No ball detected
                self.state = self.SEARCHING
                self.filtered_ball_x = None
                self.filtered_ball_y = None
                self.pid_rotation.reset()
                self.pid_distance.reset()
                
                # Alternate search direction periodically
                if time.time() - self.state_start_time > self.max_state_time:
                    self.search_direction *= -1
                    self.state_start_time = time.time()
                
                linear_vel = 0.0
                angular_vel = self.search_rotation_speed * self.search_direction
                state_text = "SEARCHING"
            
            
            # Calculate wheel velocities
            v_left = linear_vel - (angular_vel * self.wheel_distance / 2)
            v_right = linear_vel + (angular_vel * self.wheel_distance / 2)
            
            w_left = v_left / -self.wheel_radius
            w_right = v_right / self.wheel_radius
            
            self.sim.setJointTargetVelocity(self.left_joint, w_left)
            self.sim.setJointTargetVelocity(self.right_joint, w_right)
            
            # Add debug info to image
            cv2.putText(img, f"LinVel: {linear_vel:.2f}, AngVel: {angular_vel:.2f}", 
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Publish the image
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
        node.sim.setJointTargetVelocity(node.left_joint, 0.0)
        node.sim.setJointTargetVelocity(node.right_joint, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()