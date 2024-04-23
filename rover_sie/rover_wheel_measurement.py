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
        
        self.publishrate = 1.0 #s
        self.timer = self.create_timer(self.publishrate, self.wheel_data_publish)
        self.time = 0.0
        self.data_wheel = [0.0] * 13 #initialize the list with 13 zeros:time stamp + 4 wheels with 3 data each.
        
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