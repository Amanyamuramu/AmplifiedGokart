import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import tf_transformations
import math

class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')
        self.subscriber = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, 'actual_cmd_vel', 10)
        self.last_pose = None
        self.last_time = None

    def odom_callback(self, msg):
        if self.last_pose is None:
            self.last_pose = msg.pose.pose
            self.last_time = msg.header.stamp
            return

        current_time = msg.header.stamp
        dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) / 1e9
        if dt <= 0:
            return

        # 線形速度の計算
        dx = msg.pose.pose.position.x - self.last_pose.position.x
        dy = msg.pose.pose.position.y - self.last_pose.position.y
        dz = msg.pose.pose.position.z - self.last_pose.position.z
        vx = dx / dt
        vy = dy / dt
        vz = dz / dt

        # 角速度の計算（クォータニオンからの変換）
        last_orientation = [self.last_pose.orientation.x, self.last_pose.orientation.y,
                            self.last_pose.orientation.z, self.last_pose.orientation.w]
        current_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        d_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_inverse(last_orientation),
            current_orientation)
        
        # Normalize the quaternion to avoid flip issues at 180 degrees
        if np.dot(last_orientation, current_orientation) < 0:
            current_orientation = [-x for x in current_orientation]

        d_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_inverse(last_orientation),
            current_orientation)

        # Calculate angle and axis
        angle = 2 * math.atan2(np.linalg.norm(d_orientation[:3]), d_orientation[3])
        axis = np.array(d_orientation[:3])
        if np.linalg.norm(axis) != 0:
            axis /= np.linalg.norm(axis)
        angular_velocity = axis * (angle / dt)

        # Update the last pose and time
        self.last_pose = msg.pose.pose
        self.last_time = current_time

        # Publish the velocity message
        velocity_msg = Twist()
        velocity_msg.linear.x = vx
        velocity_msg.linear.y = vy
        velocity_msg.linear.z = vz
        velocity_msg.angular.x = angular_velocity[0]
        velocity_msg.angular.y = angular_velocity[1]
        velocity_msg.angular.z = angular_velocity[2]
        self.publisher.publish(velocity_msg)
        self.get_logger().info(f'Publishing Twist: Linear Velocity: ({vx}, {vy}, {vz}), Angular Velocity: {angular_velocity}')
        # self.get_logger().info(f'Angular Velocity: {angular_velocity[2]}')
        # if(angle > 0.5):
        #     self.get_logger().info(f'Angle: {angle}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
