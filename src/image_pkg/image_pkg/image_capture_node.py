import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
import cv2
import os
import json
import math
from cv_bridge import CvBridge
from datetime import datetime

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # Subscribe to viewpoint notifications
        self.create_subscription(
            String,
            '/viewpoint_reached',
            self.viewpoint_reached_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher to notify when capture is complete
        self.capture_complete_pub = self.create_publisher(String, '/capture_complete', 10)
        
        # Publisher to notify object detection
        self.new_image_pub = self.create_publisher(String, '/new_image_for_detection', 10)
        
        # Subscribe to camera feed
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Camera capture state
        self.is_capturing = False
        self.capture_queue = []
        self.current_angle = 0
        self.target_angles = [0, 90, 180, 270]  # 4 directions (in degrees)
        self.current_viewpoint = None
        self.last_captured_image = None
        
        # Directory to save images
        self.image_dir = "/home/jetson/Documents/mark/images"
        os.makedirs(self.image_dir, exist_ok=True)

        # Timer for rotation control
        self.rotation_timer = None
        
        self.get_logger().info('Image Capture Node initialized')

    def viewpoint_reached_callback(self, msg):
        # Parse viewpoint data
        try:
            viewpoint_data = json.loads(msg.data)
            self.current_viewpoint = viewpoint_data
            self.get_logger().info(f'Viewpoint reached: {viewpoint_data["viewpoint_id"]}')
            
            # Start the 360° image capture procedure
            self.start_360_capture()
        except Exception as e:
            self.get_logger().error(f'Error processing viewpoint data: {str(e)}')

    def start_360_capture(self):
        self.is_capturing = True
        self.current_angle = 0
        self.capture_queue = self.target_angles.copy()
        
        # Start the capture sequence
        self.process_next_capture()

    def process_next_capture(self):
        if not self.capture_queue:
            # All angles captured, notify completion
            self.is_capturing = False
            self.notify_capture_complete()
            return
            
        # Get the next angle to capture
        target_angle = self.capture_queue.pop(0)
        self.get_logger().info(f'Rotating to angle: {target_angle}°')
        
        # Calculate how much we need to rotate by
        angle_diff = target_angle - self.current_angle
        if angle_diff < 0:
            angle_diff += 360
        
        # Update current angle
        self.current_angle = target_angle
        
        # Perform rotation
        self.rotate_robot(angle_diff)

    def rotate_robot(self, angle_degrees):
        # Convert degrees to radians
        angle_radians = math.radians(angle_degrees)
        
        # Calculate duration based on rotation speed
        rotation_speed = 0.5  # rad/s
        duration = abs(angle_radians / rotation_speed)
        
        # Create and send the rotation command
        twist = Twist()
        twist.angular.z = rotation_speed if angle_degrees > 0 else -rotation_speed
        self.cmd_vel_pub.publish(twist)
        
        # Set a timer to stop rotation and capture image
        self.get_logger().info(f'Rotating for {duration:.2f} seconds')
        self.rotation_timer = self.create_timer(duration, self.stop_rotation)

    def stop_rotation(self):
        # Stop the robot
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Rotation complete, capturing image')
        
        # Cancel the timer
        self.rotation_timer.cancel()
        self.rotation_timer = None
        
        # Brief pause to let the robot settle
        time.sleep(0.5)
        
        # Flag that we're ready to capture on the next camera frame
        self.is_capturing = True

    def image_callback(self, msg):
        # Only process if we're in the capturing state
        if self.is_capturing and self.current_viewpoint is not None:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_captured_image = cv_image
            
            # Save the image
            self.save_current_image()
            
            # Move to the next capture or finish
            self.process_next_capture()

    def save_current_image(self):
        if self.last_captured_image is None:
            self.get_logger().error('No image to save')
            return
        
        # Generate timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create filename with viewpoint ID and angle
        filename = f"{self.current_viewpoint['viewpoint_id']}_a{self.current_angle}_{timestamp}.jpg"
        filepath = os.path.join(self.image_dir, filename)
        
        # Save the image
        cv2.imwrite(filepath, self.last_captured_image)
        self.get_logger().info(f'Image saved: {filepath}')
        
        # Notify object detection node
        image_msg = String()
        image_data = {
            "image_path": filepath,
            "viewpoint_id": self.current_viewpoint["viewpoint_id"],
            "angle": self.current_angle,
            "timestamp": timestamp,
            "robot_pose": {
                "x": self.current_viewpoint["x"],
                "y": self.current_viewpoint["y"],
                "theta": self.current_viewpoint["theta"] + math.radians(self.current_angle)
            }
        }
        image_msg.data = json.dumps(image_data)
        self.new_image_pub.publish(image_msg)
        
        # Reset the capture flag
        self.is_capturing = False

    def notify_capture_complete(self):
        # Notify viewpoint planner that capture is complete
        self.get_logger().info(f'All angles captured for viewpoint {self.current_viewpoint["viewpoint_id"]}')
        complete_msg = String()
        complete_msg.data = self.current_viewpoint["viewpoint_id"]
        self.capture_complete_pub.publish(complete_msg)
        self.current_viewpoint = None

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
