import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

class WheelDeadReckoningWriter(Node):
    def __init__(self):
        super().__init__('wheel_dead_reckoning_writer')

        # Subscriber
        self.subscription_ = self.create_subscription(Pose2D, '/rover/motion/dead_reckoning', self.callback, 10)

        self.file = open('lidardatafile.txt', 'w') 

    def callback(self, msg):
        # Process the received message and append the data to the text file
        # self.get_logger().info(f'Wheel Dead Reckoning Data: {msg.x} {msg.y} {msg.theta}')
        self.file.write(f'x: {msg.x}, y: {msg.y}, theta: {msg.theta} \n') # Writing the received message data to the file

def main(args=None):
    rclpy.init(args=args) 
    node = WheelDeadReckoningWriter()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__':
    main() 


if __name__ == '__main__':
    main()