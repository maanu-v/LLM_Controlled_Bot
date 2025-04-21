import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import math
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json


class ViewpointPlanner(Node):
    def __init__(self):
        super().__init__('viewpoint_planner')

        self.declare_parameter('num_viewpoints', 2)
        self.num_viewpoints = self.get_parameter('num_viewpoints').value

        # Publisher to signal when viewpoint is reached
        self.viewpoint_reached_pub = self.create_publisher(String, '/viewpoint_reached', 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to get notification when image capture is complete
        self.capture_complete_sub = self.create_subscription(
            String,
            '/capture_complete',
            self.capture_complete_callback,
            10
        )

        self.map_data = None
        self.resolution = None
        self.origin = None

        self.viewpoints = []
        self.current_viewpoint_index = 0
        self.waiting_for_capture = False

        # Timer to check and generate viewpoints periodically
        self.timer = self.create_timer(5.0, self.check_and_generate_viewpoints)

    def map_callback(self, msg):
        """Callback for receiving the map."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.get_logger().info('Map received for viewpoint planning.')

    def capture_complete_callback(self, msg):
        """Callback for when image capture is complete."""
        if self.waiting_for_capture:
            self.get_logger().info(f'Capture complete at viewpoint {self.current_viewpoint_index}')
            self.waiting_for_capture = False
            self.current_viewpoint_index += 1
            self.process_next_viewpoint()

    def check_and_generate_viewpoints(self):
        """Check if map is available and generate viewpoints."""
        if self.map_data is None:
            self.get_logger().info('Waiting for map...')
            return

        self.get_logger().info('Generating viewpoints...')
        self.viewpoints = self.generate_viewpoints()

        for i, vp in enumerate(self.viewpoints):
            self.get_logger().info(f'Viewpoint {i+1}: x={vp[0]:.2f}, y={vp[1]:.2f}, theta={vp[2]:.2f}')

        self.current_viewpoint_index = 0
        self.process_next_viewpoint()

        # Stop the timer after generating viewpoints
        self.timer.cancel()

    def generate_viewpoints(self):
        """Generate a list of viewpoints."""
        viewpoints = []
        for i in range(self.num_viewpoints):
            # Sample random positions (ensure they are free from obstacles)
            x = np.random.randint(1, self.map_data.shape[1] - 1)
            y = np.random.randint(1, self.map_data.shape[0] - 1)

            # Check if the position is not occupied (i.e., not an obstacle)
            if self.map_data[y, x] == 0:  # Assuming 0 is free space
                # Calculate a random orientation (theta) for the viewpoint
                theta = np.random.uniform(0, 2 * np.pi)
                viewpoints.append((x * self.resolution + self.origin.position.x,
                                   y * self.resolution + self.origin.position.y,
                                   theta))
        return viewpoints

    def process_next_viewpoint(self):
        """Process the next viewpoint."""
        if self.current_viewpoint_index >= len(self.viewpoints):
            self.get_logger().info('All viewpoints processed. Mission complete!')
            return

        viewpoint = self.viewpoints[self.current_viewpoint_index]
        self.navigate_to_viewpoint(viewpoint)

    def navigate_to_viewpoint(self, viewpoint):
        """Navigate to a specific viewpoint."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = viewpoint[0]
        goal_msg.pose.pose.position.y = viewpoint[1]
        goal_msg.pose.pose.orientation.z = math.sin(viewpoint[2] / 2)
        goal_msg.pose.pose.orientation.w = math.cos(viewpoint[2] / 2)

        self.get_logger().info(f"Navigating to viewpoint: x={viewpoint[0]}, y={viewpoint[1]}, theta={viewpoint[2]}")

        # Send the navigation goal
        self.navigator.send_goal_async(goal_msg)

        # After navigation is complete, signal the capture
        self.waiting_for_capture = True

    def publish_viewpoint_reached(self):
        """Publish message indicating the viewpoint is reached."""
        msg = String()
        msg.data = f"Viewpoint {self.current_viewpoint_index} reached"
        self.viewpoint_reached_pub.publish(msg)
        self.get_logger().info(f"Viewpoint {self.current_viewpoint_index} reached and captured.")


def main(args=None):
    rclpy.init(args=args)
    viewpoint_planner = ViewpointPlanner()
    rclpy.spin(viewpoint_planner)
    viewpoint_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
