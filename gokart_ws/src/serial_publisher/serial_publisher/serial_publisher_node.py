import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import serial
import threading
import time

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.counter_publisher = self.create_publisher(Int32, 'encoder_value', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
        self.last_counter = 0
        self.last_time = time.time()
        self.pulses_per_revolution = 720  # エンコーダの1回転あたりのパルス数（例）
        self.filtered_speed = 0.0
        self.alpha = 0.1  # ローパスフィルタの係数（0 < alpha <= 1）

    def read_serial(self):
        while True:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                try:
                    counter_value = int(line)
                    current_time = time.time()
                    time_diff = current_time - self.last_time
                    count_diff = counter_value - self.last_counter

                    # 回転速度の計算 (count_diff / time_diff で得られる値は時間当たりのカウント数)
                    speed_rpm = (count_diff / self.pulses_per_revolution) / time_diff * 60
                    speed_rps = speed_rpm / 60  # 回転毎分から回転毎秒に変換
                    speed_rad_per_sec = speed_rps * 2 * 3.141592653589793  # rad/sに変換

                    # ローパスフィルタの適用
                    self.filtered_speed = self.alpha * speed_rad_per_sec + (1 - self.alpha) * self.filtered_speed

                    self.last_counter = counter_value
                    self.last_time = current_time

                    counter_msg = Int32()
                    counter_msg.data = counter_value
                    self.counter_publisher.publish(counter_msg)
                    self.get_logger().info(f'Publishing counter: {counter_msg.data}')

                    twist_msg = Twist()
                    twist_msg.angular.z = self.filtered_speed
                    self.cmd_vel_publisher.publish(twist_msg)
                    self.get_logger().info(f'Publishing filtered speed (rad/s): {twist_msg.angular.z}')
                except ValueError:
                    self.get_logger().warn(f'Invalid data received: {line}')

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)

    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
