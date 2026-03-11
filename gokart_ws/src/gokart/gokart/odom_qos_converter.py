import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class OdomQosConverter(Node):
    def __init__(self):
        super().__init__('odom_qos_converter')
        qos_profile_pub = QoSProfile(depth=10)
        qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile_sub = QoSProfile(depth=10)
        qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Odometry, 
            '/camera/pose/sample', 
            self.listener_callback, 
            qos_profile_sub)

        self.publisher = self.create_publisher(
            Odometry, 
            '/odom_qos_converted', 
            qos_profile_pub)

    def listener_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info('Forwarding scan data to /odom_qos_converted')

def main(args=None):
    rclpy.init(args=args)
    odom_qos_converter = OdomQosConverter()
    rclpy.spin(odom_qos_converter)
    odom_qos_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
