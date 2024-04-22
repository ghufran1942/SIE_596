import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray #Input messages

class WheelDataFromSimulation(Node):
    """
    Class to handle wheel data received from simulation and write it to a text file.
    """

    def __init__(self):
        super().__init__('rover_wheel_writer')
        self.subscription = self.create_subscription(Float64MultiArray, '/rover/wheels/wheel_data', self.callback, 10)
        
        self.file = open('rover_wheel_data.txt', 'a')

    def callback(self, msg):
        """
        Callback function to process the received message and append the data to the text file.

        Args:
            msg (Float64MultiArray): The received message containing wheel data.
        """
        self.file.write(str(msg.data) + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = WheelDataFromSimulation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()