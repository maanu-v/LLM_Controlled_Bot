import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
import random
from time import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        self.last_capture_time = 0
        self.capture_interval = 3  # seconds
        self.timestamp_folder = str(int(time()))
        self.base_path = "/home/jetson/Documents/mark/img_data"
        self.image_path = os.path.join(self.base_path, "images", self.timestamp_folder)
        self.metadata_path = os.path.join(self.base_path, "metadata", f"{self.timestamp_folder}.json")

        os.makedirs(self.image_path, exist_ok=True)
        os.makedirs(os.path.dirname(self.metadata_path), exist_ok=True)

        self.get_logger().info("Image saver node started")

    def listener_callback(self, msg):
        curr_time = time()
        if curr_time - self.last_capture_time < self.capture_interval:
            return

        self.last_capture_time = curr_time

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = f"{int(curr_time)}.jpg"
        full_image_path = os.path.join(self.image_path, filename)

        cv2.imwrite(full_image_path, frame)
        self.get_logger().info(f"Saved image: {full_image_path}")

        self.save_metadata(full_image_path)

    def save_metadata(self, image_path):
        x = random.randint(1, 100)
        y = random.randint(1, 100)
        theta = random.randint(1, 100)

        metadata = {
            "x": x,
            "y": y,
            "theta": theta,
            "image": image_path
        }

        try:
            if os.path.exists(self.metadata_path):
                with open(self.metadata_path, "r") as f:
                    data = json.load(f)
            else:
                data = []
        except:
            data = []

        data.append(metadata)

        with open(self.metadata_path, "w") as f:
            json.dump(data, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

