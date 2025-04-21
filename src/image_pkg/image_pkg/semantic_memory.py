import json
import os
from datetime import datetime

class SemanticMemorySaver:
    def __init__(self, memory_directory='/home/jetson/Documents/mark/metadata'):
        self.memory_directory = memory_directory
        os.makedirs(self.memory_directory, exist_ok=True)
        
        # Initialize memory structure
        self.memory = {
            "memory_creation_time": datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
            "viewpoints": {},
            "object_instances": {}
        }
        
    def add_viewpoint(self, viewpoint_id, robot_pose, timestamp, image_captures, semantic_summary):
        """Add a new viewpoint entry to the memory."""
        viewpoint_data = {
            "entry_id": viewpoint_id,
            "entry_type": "viewpoint_context",
            "map_coordinates": robot_pose,  # Robot's position and orientation (x, y, theta)
            "timestamp": timestamp,
            "semantic_summary": semantic_summary,
            "image_captures": image_captures  # List of images captured from this viewpoint
        }
        self.memory['viewpoints'][viewpoint_id] = viewpoint_data
        
    def add_object_instance(self, object_id, label, confidence, map_coordinates, timestamp, detected_in_images):
        """Add a new object instance to the memory."""
        object_data = {
            "entry_id": object_id,
            "entry_type": "object",
            "label": label,
            "confidence": confidence,
            "map_coordinates": map_coordinates,  # Position of the object in the map
            "timestamp": timestamp,
            "detected_in_images": detected_in_images,  # List of image captures where the object was detected
            "best_view_image_id": detected_in_images[0]["image_id"] if detected_in_images else None,
            "image_snippet_path": f"{self.memory_directory}/snippets/{object_id}.jpg" if detected_in_images else None,
            "relative_position_description": "Relative to viewpoint"
        }
        self.memory['object_instances'][object_id] = object_data

    def save_memory(self):
        """Save the entire structured memory to a JSON file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        memory_filename = os.path.join(self.memory_directory, f"semantic_memory_{timestamp}.json")
        with open(memory_filename, 'w') as f:
            json.dump(self.memory, f, indent=2)
        
        # Also save a "latest" version for convenience
        latest_filename = os.path.join(self.memory_directory, "semantic_memory_latest.json")
        with open(latest_filename, 'w') as f:
            json.dump(self.memory, f, indent=2)
            
        print(f"Semantic memory saved to {memory_filename}")
        
    def update_object_detection(self, image_id, viewpoint_id, detected_objects, image_path):
        """Update object detection results from a specific image."""
        # For each detected object, check if it exists in the object_instances
        for obj in detected_objects:
            object_id = f"obj_{obj['label']}_{image_id}"
            
            # Get viewpoint coordinates from memory
            vp_coords = None
            if viewpoint_id in self.memory['viewpoints']:
                vp_coords = self.memory['viewpoints'][viewpoint_id]['map_coordinates']
            
            object_data = {
                "image_id": image_id,
                "viewpoint_id": viewpoint_id,
                "bounding_box_pixels": obj['bbox']
            }
            
            if object_id not in self.memory['object_instances']:
                # Add new object instance to memory
                self.add_object_instance(
                    object_id=object_id,
                    label=obj['label'],
                    confidence=obj['confidence'],
                    map_coordinates=vp_coords or self.get_object_coordinates(obj['label']),
                    timestamp=datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
                    detected_in_images=[object_data]
                )
            else:
                # Update existing object with new detection
                self.memory['object_instances'][object_id]['detected_in_images'].append(object_data)
                
                # Update confidence if higher
                if obj['confidence'] > self.memory['object_instances'][object_id]['confidence']:
                    self.memory['object_instances'][object_id]['confidence'] = obj['confidence']
                    
            # Add object ID to viewpoint's image capture
            if viewpoint_id in self.memory['viewpoints']:
                for capture in self.memory['viewpoints'][viewpoint_id]['image_captures']:
                    if capture['image_id'] == image_id and object_id not in capture['detected_object_instance_ids']:
                        capture['detected_object_instance_ids'].append(object_id)

    def get_object_coordinates(self, label):
        """
        Placeholder for object coordinate determination.
        In a real system, this would use depth information or other techniques.
        """
        return {"x": 0.0, "y": 0.0, "theta": 0.0}

# Example usage:
if __name__ == "__main__":
    memory_saver = SemanticMemorySaver()

    # Example of adding a viewpoint
    viewpoint_id = "vp_001"
    robot_pose = {"x": 4.5, "y": 2.8, "theta": 1.57}
    timestamp = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
    image_captures = [
        {
            "image_id": "img_vp001_a0",
            "relative_angle_deg": 0,
            "timestamp": timestamp,
            "image_path": "/path/to/robot/memory/images/img_vp001_a0.jpg",
            "detected_object_instance_ids": ["obj_mug_001", "obj_microwave_001"]
        },
        {
            "image_id": "img_vp001_a45",
            "relative_angle_deg": 45,
            "timestamp": timestamp,
            "image_path": "/path/to/robot/memory/images/img_vp001_a45.jpg",
            "detected_object_instance_ids": ["obj_chair_001"]
        }
    ]
    semantic_summary = "View shows a counter area with a microwave, mug, and a chair in the background."

    memory_saver.add_viewpoint(viewpoint_id, robot_pose, timestamp, image_captures, semantic_summary)

    # Example of adding object detection results
    detected_objects = [
        {"label": "mug", "confidence": 0.92, "bbox": [150, 200, 180, 240]},
        {"label": "microwave", "confidence": 0.95, "bbox": [250, 180, 350, 280]}
    ]
    image_id = "img_vp001_a0"
    viewpoint_id = "vp_001"
    image_path = "/path/to/robot/memory/images/img_vp001_a0.jpg"

    memory_saver.update_object_detection(image_id, viewpoint_id, detected_objects, image_path)

    # Save the entire memory to JSON file
    memory_saver.save_memory()
