import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class WheelDeadReckoningWriter(Node):
    def __init__(self):
        """
        Initializes the WheelDeadReckoningWriter class.

        This class is responsible for subscribing to the '/rover/motion/dead_reckoning' topic,
        processing the received messages, and writing the data to a text file.
        """
        super().__init__('wheel_dead_reckoning_writer')

        # Subscriber
        self.subscription_ = self.create_subscription(Pose2D, '/rover/motion/dead_reckoning', self.callback, 10)

        self.file = open('lidardatafile.txt', 'w') 

    def callback(self, msg):
        """
        Callback function for the '/rover/motion/dead_reckoning' topic.

        This function is called whenever a new message is received on the topic.
        It processes the received message and appends the data to the text file.
        """
        self.file.write(f'x: {msg.x}, y: {msg.y}, theta: {msg.theta} \n') # Writing the received message data to the file

def main(args=None):
    """
    Main function of the script.

    This function initializes the ROS 2 node, creates an instance of the WheelDeadReckoningWriter class,
    and spins the node to start processing messages.
    """
    rclpy.init(args=args) 
    node = WheelDeadReckoningWriter()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__':
    main() 
