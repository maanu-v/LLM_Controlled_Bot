import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import cv2
import json
import os
from datetime import datetime
from cv_bridge import CvBridge

from image_pkg.semantic_memory import SemanticMemorySaver

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize the YOLOv11 model (adjust path as necessary)
        try:
            self.model = torch.hub.load('ultralytics/yolov11', 'yolov11s')  # Using YOLOv5 if YOLOv11 not available
            self.get_logger().info("YOLOv11 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Error loading model: {str(e)}")
            
        self.bridge = CvBridge()

        # Directory for metadata
        self.metadata_dir = '/home/jetson/Documents/mark/metadata'
        os.makedirs(self.metadata_dir, exist_ok=True)

        # Initialize semantic memory
        self.memory_saver = SemanticMemorySaver(self.metadata_dir)
        
        # Subscribe to notifications about new images
        self.create_subscription(
            String, 
            '/new_image_for_detection',
            self.new_image_callback,
            10
        )
        
        # Metadata storage
        self.metadata = {}
        self.get_logger().info('Object Detection Node initialized')

    def new_image_callback(self, msg):
        try:
            # Parse image metadata
            image_data = json.loads(msg.data)
            self.get_logger().info(f"Processing new image: {os.path.basename(image_data['image_path'])}")
            
            # Load the image
            image_path = image_data['image_path']
            if not os.path.exists(image_path):
                self.get_logger().error(f"Image not found: {image_path}")
                return
                
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                return
                
            # Run object detection on the image
            results = self.model(cv_image)
            
            # Parse detections
            detected_objects = self.parse_detections(results)
            
            # Generate image_id
            image_id = os.path.basename(image_path).split('.')[0]
            
            # Add metadata to semantic memory
            self.update_semantic_memory(image_data, image_id, detected_objects)
            
            self.get_logger().info(f"Detected {len(detected_objects)} objects in {image_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def parse_detections(self, results):
        """Extract detected objects from the YOLO results"""
        detected_objects = []
        
        # Process results (adapt depending on the model's output format)
        for *xyxy, conf, cls in results.xyxy[0]:  # xyxy[0] contains detections in the first image
            label = self.model.names[int(cls)]  # Class name
            confidence = float(conf)
            
            # Bounding box coordinates
            bbox = {
                "xmin": int(xyxy[0].item()),
                "ymin": int(xyxy[1].item()),
                "xmax": int(xyxy[2].item()),
                "ymax": int(xyxy[3].item())
            }
            
            detected_objects.append({
                "label": label,
                "confidence": confidence,
                "bbox": bbox
            })
            
        return detected_objects
        
    def update_semantic_memory(self, image_data, image_id, detected_objects):
        # Get viewpoint info
        viewpoint_id = image_data['viewpoint_id']
        timestamp = image_data['timestamp']
        
        # Create image capture data
        image_capture = {
            "image_id": image_id,
            "relative_angle_deg": image_data['angle'],
            "timestamp": timestamp,
            "image_path": image_data['image_path'],
            "detected_object_instance_ids": []
        }
        
        # Add viewpoint if it doesn't exist
        if viewpoint_id not in self.memory_saver.memory['viewpoints']:
            robot_pose = image_data['robot_pose']
            self.memory_saver.add_viewpoint(
                viewpoint_id=viewpoint_id,
                robot_pose=robot_pose,
                timestamp=timestamp,
                image_captures=[],
                semantic_summary=f"Viewpoint {viewpoint_id} with {len(detected_objects)} detected objects"
            )
            
        # Add image capture to viewpoint
        self.memory_saver.memory['viewpoints'][viewpoint_id]['image_captures'].append(image_capture)
        
        # Process detected objects
        self.memory_saver.update_object_detection(image_id, viewpoint_id, detected_objects, image_data['image_path'])
        
        # Save the updated memory
        self.memory_saver.save_memory()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
