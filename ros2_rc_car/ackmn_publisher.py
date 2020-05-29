import rclpy
from rclpy.node import Node

from my_ackermann_interfaces.msg import AckermannDrive


class AckermannPublisher(Node):

    def __init__(self):
        super().__init__('ackermann_publisher')
        self.publisher_ = self.create_publisher(AckermannDrive, 'ackermann_drive', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AckermannDrive()

        msg.speed = 3.0
        msg.steering_angle = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.speed)


def main(args=None):
    rclpy.init(args=args)

    ackermann_publisher = AckermannPublisher()

    rclpy.spin(ackermann_publisher)

    ackermann_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
