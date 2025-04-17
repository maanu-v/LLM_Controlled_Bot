#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import http.server
import socketserver
from ament_index_python.packages import get_package_share_directory

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value
        
        # Get the path to the web resources
        self.web_dir = os.path.join(get_package_share_directory('web'), 'static')
        self.get_logger().info(f'Starting web server on port {self.port}')
        self.get_logger().info(f'Serving files from {self.web_dir}')
        
        # Start the HTTP server in a separate thread
        self.start_server()
    
    def start_server(self):
        # Check if directory exists, if not create it with a basic index.html
        if not os.path.exists(self.web_dir):
            self.get_logger().warn(f"Static directory not found: {self.web_dir}")
            self.get_logger().info("Creating basic static directory")
            os.makedirs(self.web_dir, exist_ok=True)
            with open(os.path.join(self.web_dir, 'index.html'), 'w') as f:
                f.write("<html><body><h1>Mark Robot Web Interface</h1><p>Default page - static directory was created automatically.</p></body></html>")
            
        os.chdir(self.web_dir)
        handler = http.server.SimpleHTTPRequestHandler
        self.httpd = socketserver.TCPServer(("", self.port), handler)
        
        import threading
        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        self.get_logger().info(f'Web server started successfully on port {self.port}')
    
    def destroy_node(self):
        self.get_logger().info('Shutting down web server')
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    web_server = WebServer()
    
    try:
        rclpy.spin(web_server)
    except KeyboardInterrupt:
        pass
    finally:
        web_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# import os
# import http.server
# import socketserver
# from ament_index_python.packages import get_package_share_directory

# class WebServer(Node):
#     def __init__(self):
#         super().__init__('web_server')
#         self.declare_parameter('port', 8080)
#         self.port = self.get_parameter('port').value
        
#         # Get the path to the web resources
#         self.web_dir = os.path.join(get_package_share_directory('web'), 'static')
#         self.get_logger().info(f'Starting web server on port {self.port}')
#         self.get_logger().info(f'Serving files from {self.web_dir}')
        
#         # Start the HTTP server in a separate thread
#         self.start_server()
    
#     def start_server(self):
#         os.chdir(self.web_dir)
#         handler = http.server.SimpleHTTPRequestHandler
#         self.httpd = socketserver.TCPServer(("", self.port), handler)
        
#         import threading
#         self.server_thread = threading.Thread(target=self.httpd.serve_forever)
#         self.server_thread.daemon = True
#         self.server_thread.start()
#         self.get_logger().info(f'Web server started successfully on port {self.port}')
    
#     def destroy_node(self):
#         self.get_logger().info('Shutting down web server')
#         if hasattr(self, 'httpd'):
#             self.httpd.shutdown()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     web_server = WebServer()
    
#     try:
#         rclpy.spin(web_server)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         web_server.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
