import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

class SimulatorPose(Node):
    """
    Class representing the simulator pose.

    This class subscribes to the '/rover/model/pose' topic to receive Pose messages and publishes data to the
    '/rover/model/simulated_pose' topic.

    Attributes:
        subscription: A subscription to receive Pose messages from the '/rover/model/pose' topic.
        time: The current time in seconds.
        deltaT: The frequency at which the data is published.
        publisher: A publisher to publish data to the '/rover/model/simulated_pose' topic.
        timer: A timer to trigger the publish_datafile_callback() function at the specified frequency.
        theoretical_x_t: The theoretical x-coordinate.
        theoretical_y_t: The theoretical y-coordinate.
        theoretical_z_t: The theoretical z-coordinate.
        x: The real x-coordinate.
        y: The real y-coordinate.
        z: The real z-coordinate.
        qx: The x-component of the orientation quaternion.
        qy: The y-component of the orientation quaternion.
        qz: The z-component of the orientation quaternion.
    """

    def __init__(self):
        super().__init__('simulator_pose')

        # Initialization of the trajectory quantities

        # Create a subscription to receive Pose messages from the '/rover/model/pose' topic
        self.subscription = self.create_subscription(Pose, '/rover/model/pose', self.listener_pose_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.time = 0.0  # seconds, initialization of the time
        self.deltaT = 1.0  # seconds, it is the frequency at which the data are published
        self.publisher = self.create_publisher(Float64MultiArray, '/rover/model/simulated_pose', 10)
        self.timer = self.create_timer(self.deltaT, self.publish_datafile_callback)

        self.theoretical_x_t = 0.0
        self.theoretical_y_t = 0.0
        self.theoretical_z_t = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        
    def listener_pose_callback(self, msg):
        # Callback function to handle incoming Pose messages
        position = msg.position
        orientation = msg.orientation
    
        self.x = position.x
        self.y = position.y
        self.z = position.z
        self.qx = orientation.x
        self.qy = orientation.y
        self.qz = orientation.z
        self.time += 1.0  # seconds, increment of the time
        
    def publish_datafile_callback(self):
        # Callback function to publish data to '/rover/model/simulated_pose' topic
        # The data will be a Float64MultiArray of 7 elements in the following order: 
        # time, theoretical_x_t, theoretical_y_t, theoretical_z_t, real_x, real_y, real_z, deltaX, deltaY, deltaZ
        msg = Float64MultiArray()
        # Uncomment the line below to include additional data in the message
        # msg.data = [self.time, self.theoretical_x_t, self.theoretical_y_t, self.theoretical_z_t, self.x, self.y, self.z, self.qx, self.qy, self.qz]
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SimulatorPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
