import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive


class AckermannPublisher(Node):

    def __init__(self):
        super().__init__('ackermann_publisher')
        self.publisher_ = self.create_publisher(AckermannDrive, 'ackermann_drive', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.lr = 'L'
        self.speed = 'STOP'

    def timer_callback(self):
        msg = AckermannDrive()
        
        if self.speed == 'STOP':
            msg.speed = 0.2
            self.speed = 'RUN'
        else:
            msg.speed = 0.0
            self.speed = 'STOP'

        if self.lr == 'L':
            msg.steering_angle = 1.0
            self.lr = 'R'
        else: 
            msg.steering_angle = -1.0
            self.lr = 'L'

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: speed=%s, steer=%s' % (msg.speed, msg.steering_angle))
        self.get_logger().info('spd=%s' % self.speed)

def main(args=None):
    rclpy.init(args=args)

    ackermann_publisher = AckermannPublisher()

    rclpy.spin(ackermann_publisher)

    ackermann_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
