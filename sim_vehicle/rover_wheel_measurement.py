import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D

from math import cos, sin

class WheelMeasurementsSubscriber(Node):
    """
    A class that subscribes to the '/rover/wheel/wheels_read' topic and publishes data to the '/rover/wheel/wheel_data' topic.

    Attributes:
        subscription: The subscription object for receiving wheel measurements.
        publisher: The publisher object for publishing wheel data.
        publishrate: The rate at which wheel data is published.
        timer: The timer object for triggering the wheel data publishing.
        time: The current time.
        data_wheel: A list to store the wheel data.
        wheel_radius: The radius of the wheels.
        wheel_base: The distance between the two wheels.
        x_pos: The current x position of the rover.
        y_pos: The current y position of the rover.
        orientation: The current orientation of the rover in radians.
        rear_left_wheel_pos: The position of the rear left wheel.
        rear_right_wheel_pos: The position of the rear right wheel.

    Methods:
        __init__: Initializes the WheelMeasurementsSubscriber object.
        wheels_read_callback: Callback function for processing the received wheel measurements.
        update_position: Updates the position of the rover based on the wheel measurements.
        wheel_data_publish: Publishes the wheel data to the '/rover/wheel/wheel_data' topic.
    """
    def __init__(self):
        super().__init__('wheel_measurements_subscriber')
        self.subscription = self.create_subscription(JointState, '/rover/wheels/wheels_read', self.wheels_read_callback, 10)

        self.publisher = self.create_publisher( Float64MultiArray, '/rover/wheels/wheel_data', 10)
        self.dr_publisher = self.create_publisher(Pose2D, '/rover/motion/dead_reckoning', 10)
        
        self.publishrate = 1.0 #s
        self.timer = self.create_timer(self.publishrate, self.wheel_data_publish)
        self.time = 0.0
        self.data_wheel = [0.0] * 13 #initialize the list with 13 zeros:time stamp + 4 wheels with 3 data each.
        
        self.wheel_radius = 0.4 #meters
        self.wheel_base = 0.4 #meters
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.orientation = 0.0  # Initial orientation in radians
        
        self.last_rear_left_wheel_pos = 0.0
        self.last_rear_right_wheel_pos = 0.0

    def wheels_read_callback(self, msg):
        """
        Callback function for processing the received wheel measurements.

        Args:
            msg: The JointState message containing the wheel measurements.
        """
        wheel_data = [0.0] * 13 #initialize the list with 12 zeros: 4 wheels with 3 data each.
        #time
        wheel_data[0] = self.time
        #wheel 1
        wheel_data[1] = msg.position[0] 
        wheel_data[2] = msg.velocity[0]
        wheel_data[3] = msg.effort[0]
        #wheel 2
        wheel_data[4] = msg.position[1]
        wheel_data[5] = msg.velocity[1]
        wheel_data[6] = msg.effort[1]
        #wheel 3
        wheel_data[7] = msg.position[2]
        wheel_data[8] = msg.velocity[2]
        wheel_data[9] = msg.effort[2]
        #wheel 4
        wheel_data[10] = msg.position[3]
        wheel_data[11] = msg.velocity[3]
        wheel_data[12] = msg.effort[3]

        self.data_wheel = wheel_data

        current_left_wheel_pos = wheel_data[1] # Rear left wheel position
        current_right_wheel_pos = wheel_data[4]  # Rear right wheel position
        
        # Call the dead reckoning function to update position
        self.update_position(current_left_wheel_pos, current_right_wheel_pos)
        
        # Update the last positions for the next callback
        self.last_rear_left_wheel_pos = current_left_wheel_pos
        self.last_rear_right_wheel_pos = current_right_wheel_pos

    def update_position(self, current_left_wheel_pos, current_right_wheel_pos):
        """
        Updates the position of the rover based on the wheel measurements.

        Args:
            current_left_wheel_pos: The current position of the left wheel.
            current_right_wheel_pos: The current position of the right wheel.
        """
        # Calculate the distance each wheel has traveled since the last update
        left_distance = (current_left_wheel_pos - self.last_rear_left_wheel_pos) * self.wheel_radius
        right_distance = (current_right_wheel_pos - self.last_rear_right_wheel_pos) * self.wheel_radius
        
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
    
    def wheel_data_publish(self):
        """
        Publishes the wheel data to the '/rover/wheel/wheel_data' topic.
        """
        wheel_data_msg = Float64MultiArray()
        wheel_data_msg.data = self.data_wheel
        self.publisher.publish(wheel_data_msg)
        self.time += self.publishrate
        # self.get_logger().info(f'Publishing wheel data: {wheel_data_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelMeasurementsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# |0.4|
# |   |
# |   |