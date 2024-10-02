import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
    

class RandomLaserScanPublisher(Node):

    def __init__(self):
        super().__init__('PUB_SUB')
        self.publisher = self.create_publisher(LaserScan, 'laserscan', 10)
        self.timer = self.create_timer(1.0, self.publish_laserscan)
        
        self.subscription = self.create_subscription(Twist,'cmdvel',self.listener_callback,10)
        print('Sist. Nav. --> Creating publishers and subscribers...')

    def publish_laserscan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        # In radiant, aprox 180º
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 3.14
        # Increment between measures
        scan_msg.angle_increment = 0.01
        # Time between measures
        # ((angle_max - angle_min) / angle_increment) + 1 = points = 315
        # time_increment = scan_time / points
        scan_msg.time_increment = 0.1 / 315
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.23
        scan_msg.range_max = 4.02
        scan_msg.ranges = [random.uniform(0.25, 4.0) for _ in range(360)]
        scan_msg.ranges[179] = 0.23
        scan_msg.ranges[300] = 4.02

        self.publisher.publish(scan_msg)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard a /cmd_vel msg! WELL DONE!')

def main(args=None):
    rclpy.init(args=args)
    node = RandomLaserScanPublisher()
    
    rclpy.spin(node)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
