import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        # GPIOピン設定
        self.PinEncA = 14  # A相（BCM方式で GPIO 14）
        self.PinEncB = 15  # B相（BCM方式で GPIO 15）
        self.counter = 0
        self.publisher_ = self.create_publisher(Int32, 'encoder_value', 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PinEncA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.PinEncB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.PinEncA, GPIO.FALLING, callback=self.handle_encoder, bouncetime=10)
        self.timer = self.create_timer(0.5, self.publish_encoder_value)

    def handle_encoder(self, channel):
        if GPIO.input(self.PinEncA) == GPIO.LOW:
            if GPIO.input(self.PinEncB) == GPIO.LOW:
                self.counter += 1
            else:
                self.counter -= 1
        self.get_logger().info(f"Encoder value: {self.counter}")

    def publish_encoder_value(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)

    def cleanup(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)  # ノードを実行
    except KeyboardInterrupt:
        node.get_logger().info('終了します')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
