#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import struct
import serial

# Initialize serial connection to the encoder (example on /dev/ttyUSB0)
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)

class FwdKinematics(Node):

    def __init__(self):
        super().__init__("kinematics")

        # Publishers for individual wheel speeds
        self.pub = self.create_publisher(Float32, "right_wheel", 10)
        self.pub1 = self.create_publisher(Float32, "left_wheel", 10)
        
        # Joint state publisher
        self.joint_publisher = self.create_publisher(JointState, "joint_states", 10)

        # Initialize variables
        self.left_position = 0.0
        self.right_position = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.wheel_radius = 0.034  # Adjust to your wheel radius
        self.wheel_base = 0.2  # Distance between wheels

        # Timer to update joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # Subscription to receive cmd_vel messages
        self.velocity_subscriber = self.create_subscription(Twist, "cmd_vel", self.kin_callback, 10)

    def kin_callback(self, msg):
        """Callback to convert cmd_vel messages to individual wheel speeds."""
        self.x = msg.linear.x
        self.theta = msg.angular.z * -1

        mat1 = np.matrix([[1/self.wheel_radius, 0, self.wheel_base/self.wheel_radius], 
                          [1/self.wheel_radius, 0, -self.wheel_base/self.wheel_radius]])
        mat2 = np.matrix([[self.x], [0], [self.theta]])

        M = np.matmul(mat1, mat2)
        self.right_speed = M[0].item()  # Convert to scalar
        self.left_speed = M[1].item()  # Convert to scalar

    def read_encoder_data(self):
    """Reads and parses encoder data with a header from the serial port."""
    pass  # Placeholder for testing indentation

    try:
        if ser.in_waiting > 0:
            # Read enough bytes to include header and message
            data = ser.read(40)

            # Log the raw data to debug
            self.get_logger().info(f"Raw data: {data}")

            # Check if sufficient data was read
            if len(data) < 40:
                self.get_logger().warning("Incomplete data received")
                return

            # Find the header (36, 36 or \x24\x24)
            header_pos = data.find(b'\x24\x24')
            if header_pos == -1:
                self.get_logger().warning("Header not found in received data")
                return

            # Extract the data after the header, check length
            in_data = data[(header_pos + 2):(header_pos + 10)]  # Get 8 bytes for two floats

            # Verify we have the expected 8 bytes
            if len(in_data) != 8:
                self.get_logger().warning("Incomplete data after header; expected 8 bytes")
                return

            # Unpack as two floats for left and right encoder ticks
            left_ticks, right_ticks = struct.unpack('ff', in_data)
            self.left_position = left_ticks * self.wheel_radius
            self.right_position = right_ticks * self.wheel_radius
            self.get_logger().info(f"Encoder data received: left={self.left_position}, right={self.right_position}")

    except struct.error as e:
        self.get_logger().error(f"Error unpacking data: {e}")
    except serial.SerialException as e:
        self.get_logger().error(f"Serial communication error: {e}")




    def publish_joint_states(self):
        """Publishes joint states based on encoder data."""
        self.read_encoder_data()  # Read encoder data

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [self.left_position, self.right_position]
        joint_state_msg.velocity = [self.left_speed, self.right_speed]  # Update based on encoder speed if available

        self.get_logger().info(f"Publishing joint states: left position={self.left_position}, right position={self.right_position}")
        self.joint_publisher.publish(joint_state_msg)

def main():
    rclpy.init()
    node = FwdKinematics()  # Initialize the node   
    rclpy.spin(node)        # Spin it
    rclpy.shutdown()

if __name__ == '__main__':    
    main()

