#!/usr/bin/env python3

"""
Crobot Motor Control Node (ROS 2)

This script provides ROS 2 integration for controlling the motors of a Crobot robot using GPIO pins
and PWM for speed control. It subscribes to the 'keyboard_input' topic to receive commands for motor control,
and it also checks for motor load alarm triggers and emergency switch signals.

Author:

Date: 01.08.2023

ROS Node Name: crobot_motor_node

Subscribed Topics:
  - keyboard_input (std_msgs/String): Topic to receive keyboard input commands for motor control.

Published Topics:
  None

Parameters:
  speed_left_motor_pin (int): GPIO pin number for the left motor speed control.
  speed_right_motor_pin (int): GPIO pin number for the right motor speed control.
  direction_left_motor_pin (int): GPIO pin number for the left motor direction control.
  direction_right_motor_pin (int): GPIO pin number for the right motor direction control.
  stop_motor_pin (int): GPIO pin number to stop the motor.
  break_motor_pin (int): GPIO pin number to brake the motor.
  alarm_reset_pin (int): GPIO pin number to reset the motor load alarm.
  emergency_switch_pin (int): GPIO pin number for the emergency switch.
  alarm_trigger_pin (int): GPIO pin number for the motor load alarm trigger.
  pwm_frequency (int): Frequency (in Hz) for PWM control of the motors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as gpio
import time

gpio.setwarnings(False)

class CrobotMotorNode(Node):
    """Class for controlling motors of a Crobot robot via ROS 2 messages."""
    
    def __init__(self):
        """Initialize the CrobotMotorNode class."""
        super().__init__('crobot_motor_node')
        
        gpio.setmode(gpio.BCM)

        self.speed_left_motor_pin = 12
        self.speed_right_motor_pin = 13
        self.direction_left_motor_pin = 17
        self.direction_right_motor_pin = 27

        self.stop_motor_pin = 16
        self.break_motor_pin = 26
        self.alarm_reset_pin = 25

        self.emergency_switch_pin = 5
        self.alarm_trigger_pin = 22
        self.pwm_frequency = 1000

        self.linear_speed = 0
        self.angular_speed = 0

        # Set up GPIO pins
        gpio.setup(self.emergency_switch_pin, gpio.IN)
        gpio.setup(self.alarm_trigger_pin, gpio.IN)
        gpio.setup(self.stop_motor_pin, gpio.OUT)
        gpio.setup(self.break_motor_pin, gpio.OUT)
        gpio.setup(self.alarm_reset_pin, gpio.OUT)
        gpio.setup(self.speed_left_motor_pin, gpio.OUT)
        gpio.setup(self.speed_right_motor_pin, gpio.OUT)
        gpio.setup(self.direction_left_motor_pin, gpio.OUT)
        gpio.setup(self.direction_right_motor_pin, gpio.OUT)

        # Engage break and stop pins before running
        gpio.output(self.break_motor_pin, True)
        gpio.output(self.stop_motor_pin, True)

        # Set up PWM for motor speed control
        self.speed_left_motor = gpio.PWM(self.speed_left_motor_pin, self.pwm_frequency)
        self.speed_right_motor = gpio.PWM(self.speed_right_motor_pin, self.pwm_frequency)
        self.speed_left_motor.start(0)
        self.speed_right_motor.start(0)

        # Initialize ROS 2 subscription
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        
    def engage_motor(self):
        gpio.output(self.break_motor_pin, False)
        gpio.output(self.stop_motor_pin, False)

    def forward(self, speed):
        """Move the robot forward at a given speed."""
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def reverse(self, speed):
        """Move the robot backward at a given speed."""
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def left(self, speed):
        """Turn the robot left at a given angular speed."""
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def right(self, speed):
        """Turn the robot right at a given angular speed."""
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def stop(self, speed):
        """Stop the robot by setting motor speed to 0 and turning off motor directions."""
        kp = 0.5
        speed = max(speed - 10, 0)
        for i in range(speed, 0, -5):
            self.speed_left_motor.ChangeDutyCycle(i * kp)
            self.speed_right_motor.ChangeDutyCycle(i * kp)
            time.sleep(0.2)
        gpio.output(self.stop_motor_pin, True)
        time.sleep(0.2)
        gpio.output(self.break_motor_pin, True)

    def keyboard_callback(self, msg):
        """Callback function for processing keyboard input commands."""
        alarm_val = gpio.input(self.alarm_trigger_pin)
        emergency_val = gpio.input(self.emergency_switch_pin)
        
        command = msg.data
        if "linear:" in command and "angular:" in command:
            _, linear_speed_str, _, angular_speed_str = command.split(":")
            self.linear_speed = int(linear_speed_str)
            self.angular_speed = int(angular_speed_str)
        elif command == "up":
            self.get_logger().info("Forward")
            self.forward(self.linear_speed)
        elif command == "down":
            self.get_logger().info("Reverse")
            self.reverse(self.linear_speed)
        elif command == "left":
            self.get_logger().info("Left")
            self.left(self.angular_speed)
        elif command == "right":
            self.get_logger().info("Right")
            self.right(self.angular_speed)
        elif command == "stop":
            self.get_logger().info("Stop")
            self.stop(max(self.linear_speed, self.angular_speed))

        self.get_logger().info(f"Linear Speed {self.linear_speed}\tAngular Speed {self.angular_speed}")

    def run(self):
        """Run the CrobotMotorNode class."""
        try:
            gpio.output(self.alarm_reset_pin, False)
            time.sleep(2)
            gpio.output(self.alarm_reset_pin, True)
            time.sleep(2)
            gpio.output(self.alarm_reset_pin, False)
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

        self.stop(self.linear_speed)
        gpio.cleanup()


def main(args=None):
    rclpy.init(args=args)
    crobot_motor_node = CrobotMotorNode()
    print("Motor control node started")
    crobot_motor_node.run()
    crobot_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

