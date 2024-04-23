import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

class SimulatorPose(Node):
    """
    Class representing the simulator pose.

    This class subscribes to the '/rover/model/pose' topic to receive the pose information of the rover model.
    It then publishes the simulated pose information to the '/rover/model/simulated_pose' topic.
    """

    def __init__(self):
        super().__init__('simulator_pose')

        # Initialization of the trajectory quantities

        self.subscription = self.create_subscription(Pose, '/rover/model/pose', self.pose_listener_callback, 10)
        # self.subscription  # prevent unused variable warning
        
        self.time = 0.0  # Time initialization in seconds
        self.deltaT = 1.0  # Time step in seconds
        self.publisher = self.create_publisher(Float64MultiArray, '/rover/model/simulated_pose', 10)
        self.timer = self.create_timer(self.deltaT, self.datafile_callback)
        
        # Pose Initialization
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0
        
    def pose_listener_callback(self, msg):
        """
        Callback function for the '/rover/model/pose' subscription.

        This function is called whenever a new pose message is received.
        It extracts the position and orientation information from the message and updates the pose variables.
        """
        position = msg.position
        orientation = msg.orientation
    
        self.x = position.x
        self.y = position.y
        self.z = position.z
        self.qx = orientation.x
        self.qy = orientation.y
        self.qz = orientation.z
        self.qw = orientation.w
        # self.get_logger().info(f"Pose Received: \nPosition: [x: {position.x}, y: {position.y}, z: {position.z}]\nOrientation: [x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}]")

    def datafile_callback(self):
        """
        Callback function for the timer.

        This function is called at regular intervals specified by the time step.
        It updates the time variable and publishes the simulated pose information.
        """
        self.time += self.deltaT
        msg = Float64MultiArray()
        msg.data = [self.time, self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]
        self.publisher.publish(msg)
        # self.get_logger().info(f'Acquired Pose: X {self.x} m, Y {self.y} m, Z {self.z} m')

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()