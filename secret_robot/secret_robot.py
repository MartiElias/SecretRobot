import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import random
    

class RandomLaserScanPublisher(Node):

    def __init__(self):
        super().__init__('PUB_SUB')
        self.publisher = self.create_publisher(LaserScan, 'laserscan', 10)
        self.timer = self.create_timer(1.0, self.publish_laserscan)
        
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)
        print('Sist. Nav. --> Creating publishers and subscribers...')

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_static_transform()

    def create_static_transform(self):
        # Define a static transform from 'world' (or 'map') to 'base_link'
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'world'  # Parent frame
        transform_stamped.child_frame_id = 'base_link'  # Child frame (used in LaserScan message)

        # Set the translation (position) and rotation (orientation) of 'base_link' relative to 'world'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0  # Identity quaternion

        # Publish the static transform
        self.static_broadcaster.sendTransform(transform_stamped)
        self.get_logger().info('Static transform from world to base_link published.')

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
        # Total time for a full measure
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.23
        scan_msg.range_max = 4.02
        scan_msg.ranges = [random.uniform(0.25, 4.0) for _ in range(315)]
        #scan_msg.ranges[179] = 0.25
        scan_msg.ranges[300] = 4.02

        self.publisher.publish(scan_msg)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard a /cmd_vel msg! WELL DONE!')
        

def main(args=None):
    rclpy.init(args=args)
    node = RandomLaserScanPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
