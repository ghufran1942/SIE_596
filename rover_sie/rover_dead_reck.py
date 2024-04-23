import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState

from math import cos, sin

class DeadReckoningNode(Node):
    """
    Node class for dead reckoning calculations of a rover's position and orientation.
    """

    def __init__(self):
        super().__init__('dead_reckoning_node')

        self.subscription = self.create_subscription(JointState, '/rover/wheels/wheels_read', self.wheels_read_callback, 10)

        self.dr_publisher = self.create_publisher(Pose2D, '/rover/motion/dead_reckoning', 10)
        self.timer_ = self.create_timer(0.1, self.update_position)

        self.wheel_radius = 0.4 #meters
        self.wheel_base = 0.4 #meters
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.orientation = 0.0  # Initial orientation in radians
        
        self.current_left_wheel_pos = 0.0 # Rear left wheel position
        self.current_right_wheel_pos = 0.0  # Rear right wheel position

        self.last_rear_left_wheel_pos = 0.0
        self.last_rear_right_wheel_pos = 0.0


    def wheels_read_callback(self, msg):
        self.current_left_wheel_pos = msg.position[0] # Rear left wheel position
        self.current_right_wheel_pos = msg.position[1]  # Rear right wheel position

    def update_position(self):
        """
        Updates the position of the rover based on the wheel measurements.

        Args:
            current_left_wheel_pos: The current position of the left wheel.
            current_right_wheel_pos: The current position of the right wheel.
        """
        # Calculate the distance each wheel has traveled since the last update
        left_distance = (self.current_left_wheel_pos - self.last_rear_left_wheel_pos) * self.wheel_radius
        right_distance = (self.current_right_wheel_pos - self.last_rear_right_wheel_pos) * self.wheel_radius

        # Update the last wheel positions
        self.last_rear_left_wheel_pos = self.current_left_wheel_pos
        self.last_rear_right_wheel_pos = self.current_right_wheel_pos
        
        # Dead reckoning calculations
        delta_distance = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Update the global position and orientation
        self.orientation += delta_theta
        self.x_pos += delta_distance * cos(self.orientation)
        self.y_pos += delta_distance * sin(self.orientation)
        
        # Publish the updated position and orientation
        pose_msg = Pose2D()
        pose_msg.x = self.x_pos
        pose_msg.y = self.y_pos
        pose_msg.theta = self.orientation
        self.dr_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()