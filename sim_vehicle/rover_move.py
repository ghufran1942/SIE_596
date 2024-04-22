import rclpy 
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64, Float32MultiArray 
import numpy as np

from math import pi 

class MotionNode(Node):
    """
    A class representing a ROS2 node for controlling motion of a rover.

    This node subscribes to lidar scan data and publishes motion commands to control the rover's movement.

    Attributes:
        subscription: A subscription object for receiving lidar scan data.
        delta_T_publishing: The time interval between publishing motion commands.
        vforward: The desired forward velocity of the rover.
        vturn: The desired angular velocity of the rover.
        time: The current time.
        publisher_motion: A publisher object for publishing motion commands.
        publisher_deltaT: A publisher object for publishing the time interval between motion commands.
        timer_input: A timer object for triggering the callback function at regular intervals.
    """

    def __init__(self):
        super().__init__('motion_command')

        self.subscription = self.create_subscription(Float32MultiArray, '/rover/sensor/lidar/lidar_ranges', self.lidar_scan_callback, 10)

        self.delta_T_publishing = 1.0 #seconds 
        self.vforward = 0.1 #[m/s]
        self.vturn = 0.0 #[rad/s]
        self.time = 0.0 #seconds
        self.radius = 5 #meters

        #motion command -
        self.publisher_motion = self.create_publisher(Twist, '/rover/motion/cmd_vel', 10) 
        publish_every = self.delta_T_publishing # seconds
        self.publisher_deltaT = self.create_publisher(Float64, '/rover/motion/delta', 10) 
        self.timer_input = self.create_timer(publish_every, self.timer_in_callback) 
        self.time += self.delta_T_publishing

    def lidar_scan_callback(self, msg):
        """
        Callback function for processing lidar scan data.

        This function processes the received lidar scan data and sets the desired motion command values based on the detected obstacles.

        Args:
            msg: A Float32MultiArray message containing lidar scan data.
        """
        avoid_range = 9 #meters 
        avoid_range_critical = 2*np.sqrt(2) #meters 
        range_center_index = len(msg.data)//2 
        range_center_value = msg.data[range_center_index]
        range_right_value = msg.data[0] 
        range_left_value = msg.data[-1] 

        if range_center_value < avoid_range:
            self.vforward = -0.5
            self.vturn = 0.0
        elif range_center_value > avoid_range and range_right_value >= avoid_range and range_left_value >= avoid_range:
            self.vturn = 1.0
            self.vforward = 2*self.vturn
        else:
            self.vforward = 0.0

        # if range_center_value != float('inf') or range_right_value != float('inf') or range_left_value != float('inf'):
        #     self.get_logger().info('Obstacles in sight')
        #     # Set the desired motion command values
        #     if range_center_value > avoid_range and range_right_value > avoid_range and range_left_value > avoid_range:
        #         self.vforward = 0.3
        #         self.vturn = 0.0
        #     elif range_center_value <= avoid_range_critical and range_right_value <= avoid_range_critical and range_left_value <= avoid_range_critical:
        #         self.vforward = 0.0
        #         self.vturn = 0.5
        #     else:
        #         self.vforward = 0.1
        #         self.vturn = 0.5
        # else:
        #     self.get_logger().info('No obstacles in sight')
        #     self.vforward = 0.5
        #     self.vturn = 0.0
        
    def timer_in_callback(self):
        """
        Callback function for the timer.

        This function is triggered at regular intervals and publishes the motion command and time interval between motion commands.
        """
        motion_msg = Twist() 
        motion_msg.linear.x = self.vforward 
        motion_msg.angular.z = self.vturn  

        self.publisher_motion.publish(motion_msg) 

        #publish the deltaT for the thepretical trajectory computation 
        deltaT = Float64()
        deltaT.data = self.delta_T_publishing
        self.publisher_deltaT.publish(deltaT)
        self.get_logger().info(f'Forward velocity: {round(self.vforward,4)} m/s, Angular velocity: {round(self.vturn,4)} rad/s')
    
def main(args=None):
    rclpy.init(args=args)

    node = MotionNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        