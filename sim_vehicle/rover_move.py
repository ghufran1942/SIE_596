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
        self.dead_reckoning_subscription = self.create_subscription(Twist, '/rover/motion/dead_reckoning', self.dead_reckoning_callback, 10)
        self.wheel_data_subscription = self.create_subscription(Float32MultiArray, '/rover/sensor/wheel_data', self.wheel_data_callback, 10)

        self.delta_T_publishing = 0.1 #seconds 
        self.vforward = 0.1 #[m/s]
        self.vturn = 0.0 #[rad/s]
        self.time = 0.0 #seconds

        #motion command -
        self.publisher_motion = self.create_publisher(Twist, '/rover/motion/cmd_vel', 10) 
        publish_every = self.delta_T_publishing # seconds
        self.publisher_deltaT = self.create_publisher(Float64, '/rover/motion/delta', 10) 
        self.timer_input = self.create_timer(publish_every, self.timer_in_callback) 
        self.time += self.delta_T_publishing

        self.dead_reckoning_ = None
        self.wheel_data_ = None
    
    def wheel_data_callback(self, msg):
        """
        Callback function for processing wheel data.

        This function processes the received wheel data and performs actions based on the data.

        Args:
            msg: A Float32MultiArray message containing wheel data.
        """
        self.wheel_data_ = msg
        # self.get_logger().info(f'Wheel data: {msg.data}')

    def dead_reckoning_callback(self, msg):
        """
        Callback function for processing dead reckoning data.

        This function processes the received dead reckoning data and performs actions based on the data.

        Args:
            msg: A Twist message containing dead reckoning data.
        """
        self.dead_reckoning_ = msg
        # self.get_logger().info(f'Dead Reckoning: Linear velocity: {round(msg.linear.x,4)} m/s, Angular velocity: {round(msg.angular.z,4)} rad/s')
            
            
    def lidar_scan_callback(self, msg):
        """
        Callback function for processing lidar scan data.

        This function processes the received lidar scan data and sets the desired motion command values based on the detected obstacles.

        Args:
            msg: A Float32MultiArray message containing lidar scan data.
        """
        avoid_range = 9 #meters 
        avoid_range_critical = 2*np.sqrt(3) #meters 
        range_center_index = len(msg.data)//2 
        range_center_value = msg.data[range_center_index]
        range_right_value = msg.data[0] 
        range_left_value = msg.data[-1]

        if range_center_value != float('inf') or range_right_value != float('inf') or range_left_value != float('inf'):
            self.get_logger().info('Obstacles in sight')
            # Set the desired motion command values
            if range_center_value <= avoid_range or range_right_value <= avoid_range or range_left_value <= avoid_range:
                self.vforward = -0.3
                self.vturn = 0.0

        else:
            self.get_logger().info('No obstacles in sight')
            if range_center_value > avoid_range and range_right_value > avoid_range and range_left_value > avoid_range:
                self.vturn = 0.5
                self.vforward = 2*self.vturn
                self.get_logger().info("Moving in Circle")
                while self.vforward > 0.5:
                    self.vforward -= 0.05
                    self.vturn = self.vforward/2
            else:
                self.vforward = 0.1
                self.vturn = 0.5
        
    def timer_in_callback(self):
        """
        Callback function for the timer.

        This function is triggered at regular intervals and publishes the motion command and time interval between motion commands.
        """
        motion_msg = Twist() 
        motion_msg.linear.x = self.vforward 
        motion_msg.angular.z = self.vturn  

        self.publisher_motion.publish(motion_msg) 

        # publish the deltaT for the thepretical trajectory computation 
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
        