#!/usr/bin/env python3

"""
Crobot Keyboard Publisher Node

This script provides a ROS2 node that publishes keyboard input commands for controlling a Crobot robot's motion.
It captures keyboard input using built-in Python libraries and publishes the commands to the 'keyboard_input' topic.

Author:

Date: 1.8.2023

ROS2 Node Name: keyboard_publisher

Subscribed Topics:
  None

Published Topics:
  - /keyboard_input (std_msgs/String): Topic to publish keyboard input commands for motor control.

Parameters:
  linear_speed (int): Default linear speed for the robot (0 to 30).
  angular_speed (int): Default angular speed for the robot (0 to 30).
  speed_increase_percentage (int): Speed increment percentage when adjusting linear and angular speeds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import termios
import tty

# Function to get a single character from the terminal without pressing Enter
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        # Default linear and angular speeds and speed increase percentage
        self.linear_speed = 0
        self.angular_speed = 0
        self.speed_increase_percentage = 5

        # Create a publisher for keyboard input
        self.pub = self.create_publisher(String, 'keyboard_input', 10)

        # Create a timer to publish commands at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        key = getch()

        if key == '\x1b':  # Arrow key
            key = getch()  # Read the next character representing the arrow direction
            if key == '[':
                key = getch()  # Read the specific arrow key
                if key == 'A':  # Up arrow
                    self.pub.publish(String(data="up"))
                elif key == 'B':  # Down arrow
                    self.pub.publish(String(data="down"))
                elif key == 'C':  # Right arrow
                    self.pub.publish(String(data="right"))
                elif key == 'D':  # Left arrow
                    self.pub.publish(String(data="left"))

        if key == 'w' or key == 'W':
            self.linear_speed += self.speed_increase_percentage
            self.linear_speed = min(self.linear_speed, 30)
        elif key == 's' or key == 'S':
            self.linear_speed -= self.speed_increase_percentage
            self.linear_speed = max(self.linear_speed, 0)
        elif key == 'a':
            self.angular_speed += self.speed_increase_percentage
            self.angular_speed = min(self.angular_speed, 10)
        elif key == 'd':
            self.angular_speed -= self.speed_increase_percentage
            self.angular_speed = max(self.angular_speed, 0)
        elif key == 'k':
            self.pub.publish(String(data="stop"))
        elif key == ' ':
            self.pub.publish(String(data="stop"))
        elif key == '\x03':  # Ctrl+C
            self.pub.publish(String(data="stop"))
            self.get_logger().info("Thanks For Choosing Crobot")
            self.destroy_node()
            rclpy.shutdown()

        command = f"linear:{self.linear_speed}:angular:{self.angular_speed}"
        self.get_logger().info(command)
        self.pub.publish(String(data=command))

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    print("Keyboard control node started")
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
