import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
        
        self.get_logger().info('Crobot controller started')
        
        # Default speeds
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
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
