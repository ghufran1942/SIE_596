import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32MultiArray #LaserScan messages contain Float32 numbers

class LidarMeasurments(Node):
    """
    Class representing a Lidar sensor for measuring range data.

    This class subscribes to a LaserScan topic, converts the received data to a Float32MultiArray,
    and publishes the range data to another topic.

    Attributes:
        range_data (list): List to store the range data received from the Lidar sensor.

    Subscribers:
        - '/rover/sensor/lidar/lidar_topic' (LaserScan): Subscription to the LaserScan topic.

    Publishers:
        - '/rover/sensor/lidar/lidar_ranges' (Float32MultiArray): Publication of the range data.

    """

    def __init__(self):
        super().__init__('lidar_measurements')
        
        self.range_data = [] #initialize the range data

        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/rover/sensor/lidar/lidar_topic', self.laser_scan_callback, 10)
        # self.subscription # prevent unused variable warning
        # Publishers
        self.publisher = self.create_publisher(Float32MultiArray, '/rover/sensor/lidar/lidar_ranges', 10)
        publish_rate = 0.2 # seconds
        self.timer = self.create_timer(publish_rate, self.publisher_callback)

    def laser_scan_callback(self, msg):
        """
        Callback function for the LaserScan subscriber.

        Converts the received LaserScan message to a Float32MultiArray and stores the range data.

        Args:
            msg (LaserScan): The received LaserScan message.

        """
        measured_ranges = np.array(msg.ranges, dtype=np.float32) #measured ranges
        measured_ranges = measured_ranges.tolist()
        self.range_data = measured_ranges

    def publisher_callback(self):
        """
        Callback function for the publisher timer.

        Publishes the range data as a Float32MultiArray message.

        """
        msg = Float32MultiArray()
        msg.data = self.range_data
        self.publisher.publish(msg)

        finite_distances = [d for d in msg.data if d != float('inf')]
        if not finite_distances:
            max_distance = 'max'
        else:
            max_distance = round(max(finite_distances),2)
        self.get_logger().info(f'Publishing: {max_distance} meters')

def main(args=None):
    rclpy.init(args=args)
    node = LidarMeasurments()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
