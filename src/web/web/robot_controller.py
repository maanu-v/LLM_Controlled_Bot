import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import json

class CrobotController(Node):
    def __init__(self):
        super().__init__('crobot_controller')
        
        # Create a publisher for the cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        
        # Subscribe to the command topic from the web interface
        self.command_subscription = self.create_subscription(
            String, 'web_commands', self.command_callback, 10
        )
        
        # Subscribe to robot state topics
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        # Publishers for web visualization
        self.odom_publisher = self.create_publisher(
            String, 'web_odom', 10
        )
        self.map_publisher = self.create_publisher(
            String, 'web_map', 10
        )
        self.scan_publisher = self.create_publisher(
            String, 'web_scan', 10
        )
        
        # Add periodic publishers for debugging in case we're not getting callbacks
        self.create_timer(1.0, self.publish_debug_data)
        
        self.get_logger().info('Crobot controller started')
        
        # Default speeds
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Store latest data
        self.latest_odom = None
        self.latest_map = None
        self.latest_scan = None
        
    def publish_debug_data(self):
        """Periodically publish debug data to verify connections"""
        self.get_logger().debug('Publishing debug data')
        
        # If we have real data, republish it
        if self.latest_scan:
            self.scan_publisher.publish(self.latest_scan)
            self.get_logger().debug('Republished latest scan data')
        else:
            # Create dummy scan data for testing
            dummy_scan = {
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.01,
                'range_min': 0.1,
                'range_max': 10.0,
                'ranges': [5.0 for _ in range(628)]  # Simple circular pattern
            }
            web_msg = String()
            web_msg.data = json.dumps(dummy_scan)
            self.scan_publisher.publish(web_msg)
            self.get_logger().debug('Published dummy scan data')
        
        # If we have real odom data, republish it
        if self.latest_odom:
            self.odom_publisher.publish(self.latest_odom)
            self.get_logger().debug('Republished latest odom data')
        else:
            # Create dummy odom data for testing
            dummy_odom = {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            }
            web_msg = String()
            web_msg.data = json.dumps(dummy_odom)
            self.odom_publisher.publish(web_msg)
            self.get_logger().debug('Published dummy odom data')
        
    def odom_callback(self, msg):
        """Process odometry data and publish for web visualization"""
        try:
            pose = msg.pose.pose
            twist = msg.twist.twist
            
            # Create a simplified message for the web interface
            odom_data = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                },
                'linear_velocity': {
                    'x': twist.linear.x,
                    'y': twist.linear.y,
                    'z': twist.linear.z
                },
                'angular_velocity': {
                    'x': twist.angular.x,
                    'y': twist.angular.y,
                    'z': twist.angular.z
                }
            }
            
            web_msg = String()
            web_msg.data = json.dumps(odom_data)
            self.odom_publisher.publish(web_msg)
            self.latest_odom = web_msg
            self.get_logger().debug('Published odom data')
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}')
            
    def map_callback(self, msg):
        """Process map data and publish for web visualization"""
        try:
            # Create a simplified map message for the web interface
            map_data = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'theta': 2 * (msg.info.origin.orientation.z or 0)  # Approximation for small angles
                },
                'data': list(msg.data)  # Convert to list for JSON serialization
            }
            
            web_msg = String()
            web_msg.data = json.dumps(map_data)
            self.map_publisher.publish(web_msg)
            self.latest_map = web_msg
            self.get_logger().debug('Published map data')
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')
            
    def scan_callback(self, msg):
        """Process LIDAR scan data and publish for web visualization"""
        try:
            # Create a simplified scan message for the web interface
            scan_data = {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': list(msg.ranges)  # Convert to list for JSON serialization
            }
            
            web_msg = String()
            web_msg.data = json.dumps(scan_data)
            self.scan_publisher.publish(web_msg)
            self.latest_scan = web_msg
            self.get_logger().debug('Published scan data')
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}')
    
    def command_callback(self, msg):
        """Process incoming commands from the web interface"""
        try:
            # Try to parse as JSON first (from WebSocket)
            data = json.loads(msg.data)
            cmd = data.get('command', '').strip().lower()
            params = data.get('params', {})
            
            # Handle speed command
            if cmd == 'speed':
                if 'linear' in params:
                    self.linear_speed = float(params['linear'])
                if 'angular' in params:
                    self.angular_speed = float(params['angular'])
                self.get_logger().info(f'Speed updated (JSON): linear={self.linear_speed}, angular={self.angular_speed}')
                return
            
        except (json.JSONDecodeError, AttributeError):
            # If not JSON, process as before (string format)
            cmd = msg.data.strip().lower()
            
            if ':' in cmd:
                # Format: "command:param1=value1,param2=value2"
                main_cmd, params_str = cmd.split(':', 1)
                params = {}
                if params_str:
                    for param in params_str.split(','):
                        if '=' in param:
                            key, value = param.split('=', 1)
                            try:
                                params[key.strip()] = float(value.strip())
                            except ValueError:
                                params[key.strip()] = value.strip()
                
                if main_cmd == 'speed':
                    if 'linear' in params:
                        self.linear_speed = params['linear']
                    if 'angular' in params:
                        self.angular_speed = params['angular']
                    self.get_logger().info(f'Speed updated: linear={self.linear_speed}, angular={self.angular_speed}')
                    return
                
                cmd = main_cmd
        
        # Process movement commands
        twist = Twist()
        
        if cmd == 'forward':
            twist.linear.x = self.linear_speed
        elif cmd == 'backward':
            twist.linear.x = -self.linear_speed
        elif cmd == 'left':
            twist.angular.z = self.angular_speed
        elif cmd == 'right':
            twist.angular.z = -self.angular_speed
        elif cmd == 'stop':
            # All values default to 0
            pass
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')
            return
            
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published command: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    controller = CrobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
