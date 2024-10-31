from sensor_msgs.msg import JointState
from rclpy.node import Node

class FwdKinematics(Node):
    def __init__(self):
        super().__init__("kinematics")
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Example placeholders for positions
        self.left_position = 0.0
        self.right_position = 0.0

    def read_encoder_data(self):
        """Reads and updates self.left_position and self.right_position from encoders."""
        # Example placeholder to simulate encoder reading
        self.left_position += 0.01  # Replace with actual encoder read logic
        self.right_position += 0.01

    def publish_joint_states(self):
        # Read and update encoder data
        self.read_encoder_data()

        # Prepare and publish JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_state_msg.position = [self.left_position, self.right_position]
        
        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info(f"Publishing joint states: left={self.left_position}, right={self.right_position}")


