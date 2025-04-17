import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading
import socket
import json
import time
import base64
import hashlib
import struct
import re
import cv2
import numpy as np
from cv_bridge import CvBridge

class WebBridge(Node):
    def __init__(self, port=8765):
        super().__init__('web_bridge')
        
        # Publisher for sending commands from the web to ROS
        self.command_publisher = self.create_publisher(
            String, 'web_commands', 10
        )
        
        # Subscriber for camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
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
        
        self.get_logger().info(f'Web bridge starting on port {port}')
        self.server_thread.start()
        
    def image_callback(self, msg):
        """Process incoming camera images and send to WebSocket clients"""
        if not self.websocket_clients:
            return  # Skip processing if no clients are connected
            
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Compress the image to JPEG
            _, jpeg_image = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            
            # Encode to base64 for WebSocket transmission
            encoded_image = base64.b64encode(jpeg_image).decode('utf-8')
            
            # Create a JSON message with the image data
            image_msg = {
                'type': 'camera_feed',
                'data': encoded_image
            }
            image_data = json.dumps(image_msg).encode('utf-8')
            
            # Send to all connected WebSocket clients
            with self.websocket_clients_lock:
                disconnected_clients = set()
                for client in self.websocket_clients:
                    try:
                        self._send_websocket_message(client, image_data)
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                        disconnected_clients.add(client)
                
                # Remove any disconnected clients
                for client in disconnected_clients:
                    self.websocket_clients.remove(client)
                    
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
        msg = String()
        
        if command == 'move' and params:
            # Format a more specific movement command with direction and speeds
            direction = params.get('direction', 'stop')
            linear = params.get('linear', 0.0)
            angular = params.get('angular', 0.0)
            
            msg.data = f"{direction}:linear={linear},angular={angular}"
        elif params:
            # Format other commands with parameters
            param_str = ','.join([f"{k}={v}" for k, v in params.items()])
            msg.data = f"{command}:{param_str}"
        else:
            msg.data = command
            
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published command: {msg.data}')

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
