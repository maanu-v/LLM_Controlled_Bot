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
from geometry_msgs.msg import Twist
import Jetson.GPIO as gpio
import time

gpio.setwarnings(False)

class CrobotMotorNode(Node):
    def __init__(self):
        super().__init__('crobot_motor_node')

        # Motor pins setup
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

        # Manual override state
        self.manual_override = False
        self.override_timeout = 2.0  # seconds
        self.last_manual_time = self.get_clock().now()

        # GPIO setup
        gpio.setup(self.emergency_switch_pin, gpio.IN)
        gpio.setup(self.alarm_trigger_pin, gpio.IN)
        gpio.setup(self.stop_motor_pin, gpio.OUT)
        gpio.setup(self.break_motor_pin, gpio.OUT)
        gpio.setup(self.alarm_reset_pin, gpio.OUT)
        gpio.setup(self.speed_left_motor_pin, gpio.OUT)
        gpio.setup(self.speed_right_motor_pin, gpio.OUT)
        gpio.setup(self.direction_left_motor_pin, gpio.OUT)
        gpio.setup(self.direction_right_motor_pin, gpio.OUT)

        gpio.output(self.break_motor_pin, True)
        gpio.output(self.stop_motor_pin, True)

        # PWM setup
        self.speed_left_motor = gpio.PWM(self.speed_left_motor_pin, self.pwm_frequency)
        self.speed_right_motor = gpio.PWM(self.speed_right_motor_pin, self.pwm_frequency)
        self.speed_left_motor.start(0)
        self.speed_right_motor.start(0)

        # Subscribers
        self.create_subscription(String, 'keyboard_input', self.keyboard_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def is_manual_override_active(self):
        time_diff = self.get_clock().now() - self.last_manual_time
        return time_diff.nanoseconds / 1e9 < self.override_timeout

    def engage_motor(self):
        gpio.output(self.break_motor_pin, False)
        gpio.output(self.stop_motor_pin, False)

    def forward(self, speed):
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def reverse(self, speed):
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def left(self, speed):
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def right(self, speed):
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def stop(self, speed):
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
        command = msg.data
        self.last_manual_time = self.get_clock().now()

        if "linear:" in command and "angular:" in command:
            _, linear_speed_str, _, angular_speed_str = comand.split(":")
            self.linear_speed = int(linear_speed_str)
            self.angular_speed = int(angular_speed_str)
            return

        if command == "up":
            self.get_logger().info("[MANUAL] Forward")
            self.forward(self.linear_speed)
        elif command == "down":
            self.get_logger().info("[MANUAL] Reverse")
            self.reverse(self.linear_speed)
        elif command == "left":
            self.get_logger().info("[MANUAL] Left")
            self.left(self.angular_speed)
        elif command == "right":
            self.get_logger().info("[MANUAL] Right")
            self.right(self.angular_speed)
        elif command == "stop":
            self.get_logger().info("[MANUAL] Stop")
            self.stop(max(self.linear_speed, self.angular_speed))

    def cmd_vel_callback(self, msg):
        if self.is_manual_override_active():
            return  # Skip autonomous input while manual override is active

        linear = msg.linear.x
        angular = msg.angular.z

        pwm_linear = int(min(abs(linear) * 30, 30))
        pwm_angular = int(min(abs(angular) * 30, 30))

        if linear > 0:
            self.get_logger().info("[AUTO] Forward")
            self.forward(pwm_linear)
        elif linear < 0:
            self.get_logger().info("[AUTO] Reverse")
            self.reverse(pwm_linear)
        elif angular > 0:
            self.get_logger().info("[AUTO] Left")
            self.left(pwm_angular)
        elif angular < 0:
            self.get_logger().info("[AUTO] Right")
            self.right(pwm_angular)
        else:
            self.get_logger().info("[AUTO] Stop")
            self.stop(max(pwm_linear, pwm_angular))

    def run(self):
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

