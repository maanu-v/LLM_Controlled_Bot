import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import tf2_ros
import numpy as np
import threading
import socket
import json
import time
import base64
import hashlib
import struct
import re
import cv2
from cv_bridge import CvBridge

class WebBridge(Node):
    def __init__(self, port=8765):
        super().__init__('web_bridge')
        
        # Publisher for sending commands from the web to ROS
        self.command_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Subscriber for camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe directly to actual robot state topics
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.actual_odom_callback, 10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.actual_map_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.actual_scan_callback, 10
        )
        
        # Motor RPM subscriptions for telemetry
        self.left_motor_rpm_subscription = self.create_subscription(
            Float32, '/left_motor_rpm', self.left_motor_rpm_callback, 10
        )
        self.right_motor_rpm_subscription = self.create_subscription(
            Float32, '/right_motor_rpm', self.right_motor_rpm_callback, 10
        )
        
        # TF buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Store latest data
        self.latest_odom = None
        self.latest_map = None
        self.latest_scan = None
        self.left_motor_rpm = 0
        self.right_motor_rpm = 0
        
        # OpenCV bridge for converting ROS images to OpenCV format
        self.cv_bridge = CvBridge()
        
        # Store connected WebSocket clients to stream images
        self.websocket_clients = set()
        self.websocket_clients_lock = threading.Lock()
        
        # Create a TCP server to communicate with the web interface
        self.port = port
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.running = True
        
        # Set up a timer to broadcast telemetry data
        self.telemetry_timer = self.create_timer(0.5, self.broadcast_telemetry)
        
        self.get_logger().info(f'Web bridge starting on port {port}')
        self.server_thread.start()
    
    def broadcast_telemetry(self):
        """Broadcasts overall telemetry data to clients"""
        if not self.websocket_clients:
            return  # Skip if no clients connected
        
        try:
            # Calculate battery level - this is a placeholder
            # In a real robot, you would read battery voltage or state of charge
            battery_level = 85  # Placeholder value in percentage
            
            # Create telemetry message
            telemetry_msg = {
                'type': 'telemetry_data',
                'data': {
                    'battery_level': battery_level,
                    'left_motor_rpm': self.left_motor_rpm,
                    'right_motor_rpm': self.right_motor_rpm,
                    'connection': 'connected'
                }
            }
            
            ws_data = json.dumps(telemetry_msg).encode('utf-8')
            self._send_to_all_websocket_clients(ws_data)
        except Exception as e:
            self.get_logger().error(f'Error sending telemetry data: {e}')
    
    def left_motor_rpm_callback(self, msg):
        """Store left motor RPM data"""
        self.left_motor_rpm = msg.data
        
    def right_motor_rpm_callback(self, msg):
        """Store right motor RPM data"""
        self.right_motor_rpm = msg.data
    
    def actual_odom_callback(self, msg):
        """Process actual odometry data from ROS topic"""
        if not self.websocket_clients:
            return  # Skip if no clients connected
            
        try:
            # Store latest odom data
            self.latest_odom = msg
            
            # Convert ROS odometry message to JSON-serializable format
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            linear = msg.twist.twist.linear
            angular = msg.twist.twist.angular
            
            odom_data = {
                'position': {
                    'x': position.x,
                    'y': position.y,
                    'z': position.z
                },
                'orientation': {
                    'x': orientation.x,
                    'y': orientation.y,
                    'z': orientation.z,
                    'w': orientation.w
                },
                'twist': {
                    'linear': {
                        'x': linear.x,
                        'y': linear.y,
                        'z': linear.z
                    },
                    'angular': {
                        'x': angular.x,
                        'y': angular.y,
                        'z': angular.z
                    }
                },
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            
            # Create JSON message
            odom_msg = {
                'type': 'odom_data',
                'data': odom_data
            }
            ws_data = json.dumps(odom_msg).encode('utf-8')
            
            # Send to all connected WebSocket clients
            self._send_to_all_websocket_clients(ws_data)
        except Exception as e:
            self.get_logger().error(f'Error processing odometry data: {e}')
            
    def actual_map_callback(self, msg):
        """Process actual map data from ROS topic"""
        if not self.websocket_clients:
            return  # Skip if no clients connected
            
        try:
            # Store latest map data
            self.latest_map = msg
            
            # Convert ROS occupancy grid to JSON-serializable format
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            
            # Convert map data array (int8[]) to regular list
            data = [int(cell) for cell in msg.data]
            
            map_data = {
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin': {
                    'x': origin_x,
                    'y': origin_y
                },
                'data': data
            }
            
            # Create JSON message
            map_msg = {
                'type': 'map_data',
                'data': map_data
            }
            ws_data = json.dumps(map_msg).encode('utf-8')
            
            # Send to all connected WebSocket clients
            self._send_to_all_websocket_clients(ws_data)
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {e}')
            
    def actual_scan_callback(self, msg):
        """Process actual LIDAR scan data from ROS topic"""
        if not self.websocket_clients:
            return  # Skip if no clients connected
            
        try:
            # Store latest scan data
            self.latest_scan = msg
            
            # Convert LaserScan message to JSON-serializable format
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            range_min = msg.range_min
            range_max = msg.range_max
            scan_time = msg.scan_time
            
            # Convert ranges to regular list (filter out infinities)
            ranges = []
            for r in msg.ranges:
                if r == float('inf'):
                    ranges.append(range_max)
                elif r == float('-inf'):
                    ranges.append(range_min)
                else:
                    ranges.append(float(r))
            
            scan_data = {
                'angle_min': angle_min,
                'angle_max': angle_max,
                'angle_increment': angle_increment,
                'range_min': range_min,
                'range_max': range_max,
                'scan_time': scan_time,
                'ranges': ranges
            }
            
            # Create JSON message
            scan_msg = {
                'type': 'scan_data',
                'data': scan_data
            }
            ws_data = json.dumps(scan_msg).encode('utf-8')
            
            # Send to all connected WebSocket clients
            self._send_to_all_websocket_clients(ws_data)
        except Exception as e:
            self.get_logger().error(f'Error processing LIDAR scan data: {e}')
    
    def _send_to_all_websocket_clients(self, data):
        """Helper method to send data to all connected WebSocket clients"""
        with self.websocket_clients_lock:
            disconnected_clients = set()
            for client in self.websocket_clients:
                try:
                    self._send_websocket_message(client, data)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    disconnected_clients.add(client)
            
            # Remove any disconnected clients
            for client in disconnected_clients:
                self.websocket_clients.remove(client)
    
    def image_callback(self, msg):
        """Process incoming camera images and send to WebSocket clients"""
        if not self.websocket_clients:
            return  # Skip processing if no clients are connected
            
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize image for web streaming (reduce bandwidth)
            height, width = cv_image.shape[:2]
            max_width = 640  # Maximum width for web display
            
            if width > max_width:
                # Calculate new height to maintain aspect ratio
                scale_factor = max_width / float(width)
                new_height = int(height * scale_factor)
                cv_image = cv2.resize(cv_image, (max_width, new_height))
            
            # Compress the image to JPEG (quality 70%)
            _, jpeg_image = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            
            # Encode to base64 for WebSocket transmission
            encoded_image = base64.b64encode(jpeg_image).decode('utf-8')
            
            # Create a JSON message with the image data
            image_msg = {
                'type': 'camera_feed',
                'data': encoded_image,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'width': cv_image.shape[1],
                'height': cv_image.shape[0]
            }
            image_data = json.dumps(image_msg).encode('utf-8')
            
            # Send to all connected WebSocket clients
            self._send_to_all_websocket_clients(image_data)
                    
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def _run_server(self):
        """Run a simple TCP server to receive commands from the web interface"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_socket.bind(('0.0.0.0', self.port))
            server_socket.listen(5)
            server_socket.settimeout(1.0)  # Allow checking self.running
            
            self.get_logger().info(f'Server listening on port {self.port}')
            
            while self.running:
                try:
                    client_socket, address = server_socket.accept()
                    self.get_logger().info(f'Connection from {address}')
                    
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_socket, address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().error(f'Server error: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'Failed to start server: {e}')
        finally:
            server_socket.close()
            
    def _handle_client(self, client_socket, address):
        """Handle communication with a connected client"""
        try:
            # First, check if this is a WebSocket connection
            data = client_socket.recv(1024)
            if not data:
                return
                
            # Check if this is a WebSocket handshake
            if b'GET ' in data and b'Sec-WebSocket-Key:' in data:
                self.get_logger().info(f'WebSocket handshake detected from {address}')
                self._handle_websocket(client_socket, data, address)
            else:
                # Handle as regular TCP connection
                buffer = data
                while self.running:
                    if not buffer:
                        data = client_socket.recv(1024)
                        if not data:
                            break
                        buffer = data
                        
                    if b'\n' in buffer:
                        # Process complete messages that end with newline
                        messages = buffer.split(b'\n')
                        buffer = messages.pop()  # Keep incomplete message in buffer
                        
                        for msg_bytes in messages:
                            if msg_bytes:
                                self._process_json_message(client_socket, msg_bytes)
        except Exception as e:
            self.get_logger().error(f'Client handler error: {e}')
        finally:
            client_socket.close()
            self.get_logger().info(f'Connection closed from {address}')
    
    def _handle_websocket(self, client_socket, handshake_data, address):
        """Handle WebSocket protocol communication"""
        try:
            # Complete WebSocket handshake
            key_match = re.search(b'Sec-WebSocket-Key: (.+)\r\n', handshake_data)
            if not key_match:
                self.get_logger().error('WebSocket handshake failed: no Sec-WebSocket-Key found')
                return
                
            ws_key = key_match.group(1).decode('utf-8')
            response_key = self._generate_websocket_accept(ws_key)
            
            handshake_response = (
                b'HTTP/1.1 101 Switching Protocols\r\n'
                b'Upgrade: websocket\r\n'
                b'Connection: Upgrade\r\n'
                b'Sec-WebSocket-Accept: ' + response_key.encode('utf-8') + b'\r\n\r\n'
            )
            
            client_socket.sendall(handshake_response)
            self.get_logger().info(f'WebSocket handshake completed with {address}')
            
            # Add client to WebSocket clients for streaming
            with self.websocket_clients_lock:
                self.websocket_clients.add(client_socket)
            
            # Now handle WebSocket frames
            while self.running:
                try:
                    message = self._read_websocket_message(client_socket)
                    if message is None:
                        break
                    
                    # Process the JSON message from the WebSocket
                    self._process_json_message(client_socket, message, is_websocket=True)
                        
                except Exception as e:
                    self.get_logger().error(f'WebSocket error: {e}')
                    break
            
            # Remove client when connection ends
            with self.websocket_clients_lock:
                if client_socket in self.websocket_clients:
                    self.websocket_clients.remove(client_socket)
                    
        except Exception as e:
            self.get_logger().error(f'WebSocket handler error: {e}')
            
            # Ensure client is removed from the list
            with self.websocket_clients_lock:
                if client_socket in self.websocket_clients:
                    self.websocket_clients.remove(client_socket)
    
    def _generate_websocket_accept(self, key):
        """Generate the Sec-WebSocket-Accept value for WebSocket handshake"""
        GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
        hash_key = key + GUID
        sha1 = hashlib.sha1(hash_key.encode('utf-8')).digest()
        return base64.b64encode(sha1).decode('utf-8')
    
    def _read_websocket_message(self, client_socket):
        """Read and decode a WebSocket message frame"""
        try:
            # Read the header
            header = client_socket.recv(2)
            if not header or len(header) < 2:
                return None
                
            # Parse basic frame info
            fin = (header[0] & 0x80) != 0
            opcode = header[0] & 0x0F
            mask = (header[1] & 0x80) != 0
            payload_len = header[1] & 0x7F
            
            # Handle control frames
            if opcode == 0x8:  # Close frame
                self.get_logger().info("WebSocket close frame received")
                return None
            elif opcode == 0x9:  # Ping frame
                # Send pong response
                client_socket.sendall(b'\x8A\x00')
                return b''
            elif opcode == 0xA:  # Pong frame
                return b''
            elif opcode not in [0x1, 0x2]:  # Only handle text and binary frames
                self.get_logger().warn(f"Unsupported WebSocket opcode: {opcode}")
                return b''
                
            # Extended payload length
            if payload_len == 126:
                ext_len = client_socket.recv(2)
                payload_len = struct.unpack('!H', ext_len)[0]
            elif payload_len == 127:
                ext_len = client_socket.recv(8)
                payload_len = struct.unpack('!Q', ext_len)[0]
                
            # Masking key
            if mask:
                masking_key = client_socket.recv(4)
            else:
                masking_key = None
                
            # Payload data
            payload = client_socket.recv(payload_len)
            if mask:
                payload = self._unmask_data(payload, masking_key)
                
            return payload
                
        except Exception as e:
            self.get_logger().error(f"Error reading WebSocket frame: {e}")
            return None
    
    def _unmask_data(self, payload, mask_key):
        """Unmask WebSocket data using XOR with the mask key"""
        unmasked = bytearray(len(payload))
        for i in range(len(payload)):
            unmasked[i] = payload[i] ^ mask_key[i % 4]
        return bytes(unmasked)
    
    def _send_websocket_message(self, client_socket, message):
        """Send a message through WebSocket protocol"""
        # Create a WebSocket frame with text data
        header = bytearray()
        header.append(0x81)  # FIN + Text opcode
        
        length = len(message)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack('!H', length))
        else:
            header.append(127)
            header.extend(struct.pack('!Q', length))
            
        client_socket.sendall(header + message)
    
    def _process_json_message(self, client_socket, msg_bytes, is_websocket=False):
        """Process a received JSON message"""
        try:
            msg_str = msg_bytes.decode('utf-8').strip()
            msg_data = json.loads(msg_str)
            
            if 'command' in msg_data:
                self._process_command(msg_data['command'], msg_data.get('params', {}))
                
                # Send acknowledgment
                response = {'status': 'success', 'message': f'Command {msg_data["command"]} processed'}
                response_data = (json.dumps(response) + '\n').encode('utf-8')
                
                if is_websocket:
                    self._send_websocket_message(client_socket, response_data)
                else:
                    client_socket.sendall(response_data)
            else:
                response = {'status': 'error', 'message': 'No command specified'}
                response_data = (json.dumps(response) + '\n').encode('utf-8')
                
                if is_websocket:
                    self._send_websocket_message(client_socket, response_data)
                else:
                    client_socket.sendall(response_data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON received: {msg_str}')
            response = {'status': 'error', 'message': 'Invalid JSON format'}
            response_data = (json.dumps(response) + '\n').encode('utf-8')
            
            if is_websocket:
                self._send_websocket_message(client_socket, response_data)
            else:
                client_socket.sendall(response_data)
    
    def _process_command(self, command, params=None):
        """Process a command from the web interface and publish to ROS"""
        # Create a Twist message for movement commands
        twist_msg = Twist()
        
        if command == 'forward':
            twist_msg.linear.x = params.get('speed', 0.5)  # Default to 0.5 m/s if not specified
        elif command == 'backward':
            twist_msg.linear.x = -params.get('speed', 0.5)  # Negative for backward
        elif command == 'left':
            twist_msg.angular.z = params.get('speed', 0.5)  # Positive for counterclockwise
        elif command == 'right':
            twist_msg.angular.z = -params.get('speed', 0.5)  # Negative for clockwise
        elif command == 'stop' or command == 'emergency_stop':
            # Stop all motion
            pass  # Twist defaults to all zeros
        elif command == 'move' and params:
            # Direct control of linear and angular velocity
            linear = params.get('linear', 0.0)
            angular = params.get('angular', 0.0)
            
            twist_msg.linear.x = float(linear)
            twist_msg.angular.z = float(angular)
            
        # Publish the twist message to control the robot
        self.command_publisher.publish(twist_msg)
        self.get_logger().info(f'Published command: {command} with twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    bridge = WebBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.running = False
        time.sleep(1.5)  # Give server thread time to shutdown
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
