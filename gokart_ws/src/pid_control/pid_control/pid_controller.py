import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simple_pid import PID
import csv
import os
from datetime import datetime

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_filtered_value = 0

    def filter(self, value):
        filtered_value = self.last_filtered_value + self.alpha * (value - self.last_filtered_value)
        self.last_filtered_value = filtered_value
        return filtered_value

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter('use_filtering', True)
        self.use_filtering = self.get_parameter('use_filtering').value
        
        self.subscription = self.create_subscription(
            Odometry,
            'odom_qos_converted',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'ideal_cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.linear_cmd_vel = None
        self.angular_cmd_vel = None
        self.linear_pid = PID(1.853240118028355, 1.6308510766377924, 0.18924134914358529, setpoint=1)
        self.angular_pid = PID(0.7754753262053543, 1.661765859293789, -0.06986925076771763, setpoint=1)
        self.linear_filter = LowPassFilter(alpha=0.05)
        self.angular_filter = LowPassFilter(alpha=0.05)

        self.linear_cmd_vel_filter = LowPassFilter(alpha=0.5)
        self.angular_cmd_vel_filter = LowPassFilter(alpha=0.5)

        self.csv_file = f"pid_data_{datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
        self.init_csv()

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp", "target_linear_velocity", "actual_linear_velocity", "corrected_linear_velocity",
                                 "target_angular_velocity", "actual_angular_velocity", "corrected_angular_velocity"])

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received target velocities: linear {msg.linear.x}, angular {msg.angular.z}')
        #self.linear_cmd_vel = msg.linear.x
        #self.angular_cmd_vel = msg.angular.z

        #ここでLOWPASSの処理
        self.linear_cmd_vel = self.linear_cmd_vel_filter.filter(msg.linear.x)
        self.angular_cmd_vel = self.angular_cmd_vel_filter.filter(msg.angular.z)
        

        self.linear_pid.setpoint = self.linear_cmd_vel
        self.angular_pid.setpoint = self.angular_cmd_vel


    def odom_callback(self, msg):
        raw_linear_speed = msg.twist.twist.linear.x
        raw_angular_speed = msg.twist.twist.angular.z
        
        if self.use_filtering:
            filtered_linear_speed = self.linear_filter.filter(raw_linear_speed)
            filtered_angular_speed = self.angular_filter.filter(raw_angular_speed)
        else:
            filtered_linear_speed = raw_linear_speed
            filtered_angular_speed = raw_angular_speed
        
        self.get_logger().info(f'Velocities used for PID: linear {filtered_linear_speed}, angular {filtered_angular_speed}')
        
        if self.linear_cmd_vel is not None and self.angular_cmd_vel is not None:
            corrected_linear_speed = self.linear_pid(filtered_linear_speed)
            corrected_angular_speed = self.angular_pid(filtered_angular_speed)
            self.write_to_csv(datetime.now(), self.linear_cmd_vel, filtered_linear_speed, corrected_linear_speed,
                              self.angular_cmd_vel, filtered_angular_speed, corrected_angular_speed)
            velocity_msg = Twist()
            # velocity_msg.linear.x = corrected_linear_speed
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = corrected_angular_speed
            self.publisher.publish(velocity_msg)
            self.get_logger().info(f'Publishing corrected velocities: linear {corrected_linear_speed}, angular {corrected_angular_speed}')

    def write_to_csv(self, timestamp, target_linear, actual_linear, corrected_linear,
                     target_angular, actual_angular, corrected_angular):
        timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp_str, target_linear, actual_linear, corrected_linear,
                             target_angular, actual_angular, corrected_angular])

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
