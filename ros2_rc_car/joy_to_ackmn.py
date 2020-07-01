import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy


class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')

        self.publisher_ = self.create_publisher(
                AckermannDrive, 'ackermann_drive', 10)

        self.subscription = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('Received joy message: %s' % msg.axes)

        ackmn_msg = AckermannDrive()
        ackmn_msg.steering_angle = msg.axes[0]
        ackmn_msg.speed = msg.axes[4]
        
        self.get_logger().info('steer: %s, speed: %s' % (ackmn_msg.steering_angle, ackmn_msg.speed))

        self.publisher_.publish(ackmn_msg)
        self.get_logger().info('published Ackermann msg')


def main(args=None):
    rclpy.init(args=args)

    joy_subscriber = JoySubscriber()

    rclpy.spin(joy_subscriber)
    
    joy_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
