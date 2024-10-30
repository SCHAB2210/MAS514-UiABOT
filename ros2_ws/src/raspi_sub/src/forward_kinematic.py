#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState  # Import JointState
from std_msgs.msg import Float32
import numpy as np
import serial
from math import cos, sin

# Initialize serial connection
#ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)  # Use a common baud rate

class FwdKinematics(Node):

    def __init__(self):
        super().__init__("kinematics")

        # Publishers for individual wheel speeds
        self.pub = self.create_publisher(Float32, "right_wheel", 10)
        self.pub1 = self.create_publisher(Float32, "left_wheel", 10)
        
        # Joint state publisher
        self.joint_publisher = self.create_publisher(JointState, "joint_states", 10)

        # Timers for publishing wheel speeds and joint states
        self.timer = self.create_timer(0.005, self.speed_callback)
        self.timer1 = self.create_timer(0.005, self.speed1_callback)
        self.joint_timer = self.create_timer(0.05, self.publish_joint_states)  # Joint states update at 20 Hz

        # Variables
        self.x = 0.0
        self.theta = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.r = 0.068 / 2  # Wheel radius
        self.d = 0.2        # Distance between wheels

        # Subscribe to cmd_vel
        self.velocity_subscriber = self.create_subscription(Twist, "cmd_vel", self.kin_callback, 10)

    def kin_callback(self, msg):
        """Callback to convert cmd_vel to wheel speeds."""
        self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
        self.x = msg.linear.x
        self.theta = msg.angular.z * -1

        # Compute wheel speeds
        mat1 = np.matrix([[1 / self.r, 0, self.d / self.r], [1 / self.r, 0, -self.d / self.r]])
        mat2 = np.matrix([[self.x], [0], [self.theta]])

        M = np.matmul(mat1, mat2)
        self.right_speed = M[0].item()  # Convert to scalar
        self.left_speed = M[1].item()   # Convert to scalar

    def speed_callback(self):
        """Publish right wheel speed."""
        vel_msg = Float32()
        vel_msg.data = self.right_speed
        self.get_logger().info(f"Publishing right wheel speed: {self.right_speed}")
        self.pub.publish(vel_msg)

    def speed1_callback(self):
        """Publish left wheel speed."""
        vel_msg1 = Float32()
        vel_msg1.data = self.left_speed
        self.get_logger().info(f"Publishing left wheel speed: {self.left_speed}")
        self.pub1.publish(vel_msg1)

    def publish_joint_states(self):
        """Publish joint states for visualization."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Update wheel positions based on speed and time interval
        delta_time = 0.05  # Same as joint_timer period
        self.left_position += self.left_speed * delta_time
        self.right_position += self.right_speed * delta_time

        # Fill joint state message with positions and velocities
        joint_state_msg.position = [self.left_position, self.right_position]
        joint_state_msg.velocity = [self.left_speed, self.right_speed]
        
        self.get_logger().info(f"Publishing joint states: left position={self.left_position}, right position={self.right_position}")
        # Publish joint states
        self.joint_publisher.publish(joint_state_msg)

def main():
    rclpy.init()
    node = FwdKinematics()  # Initialize the node   
    rclpy.spin(node)        # Spin it
    rclpy.shutdown()

if __name__ == '__main__':    
    main()

