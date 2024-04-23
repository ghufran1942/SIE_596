import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray # Importing the Float32MultiArray message type

class LidarDataFromSimulation(Node):
    """
    Class for subscribing to lidar data from simulation and writing it to a file.
    """

    def __init__(self):
        super().__init__('rover_lidar_writer') # Initializing the Node with the name 'rover_lidar_writer'
        self.subscription = self.create_subscription(Float32MultiArray, '/rover/sensor/lidar/lidar_ranges', self.callback, 10) # Creating a subscription to the '/rover/sensor/lidar/lidar_ranges' topic and specifying the callback function to be called when a message is received
        
        self.file = open('lidardatafile.txt', 'w') 

    def callback(self, msg):
        # Process the received message and append the data to the text file
        self.file.write(str(msg.data) + '\n') # Writing the received message data to the file

def main(args=None):
    rclpy.init(args=args) 
    node = LidarDataFromSimulation()
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == '__main__':
    main() 
