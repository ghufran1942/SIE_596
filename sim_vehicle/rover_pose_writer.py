import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PoseWriter(Node):
    """
    A class that writes rover pose data to a text file.

    This class subscribes to the '/rover/model/simulated_pose' topic and writes the received pose data to a text file.

    Attributes:
        subscription: A subscription object for receiving pose messages.
        file: A file object for writing the pose data.
    """

    def __init__(self):
        super().__init__('rover_pose_writer')

        self.subscription = self.create_subscription(Float64MultiArray, '/rover/model/simulated_pose', self.pose_callback, 10)

        self.file = open('rover_pose_data.txt', 'a')

    def pose_callback(self, msg):
        """
        Process the received message and append the data to the text file.

        Args:
            msg: A Float64MultiArray message containing the rover pose data.
        """
        self.file.write(str(msg.data) + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = PoseWriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
