import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from my_ackermann_interfaces.msg import AckermannDrive
import Adafruit_PCA9685


CHANNEL_STEER = 0
CHANNEL_DRIVE = 1
# Servo pulse lengths (out of 4096)
STEER_MIN = 230
STEER_MAX = 520
STEER_RANGE = STEER_MAX - STEER_MIN
STEER_NEUTRAL = STEER_RANGE // 2 + STEER_MIN
DRIVE_MIN = 250
DRIVE_MAX = 500
DRIVE_RANGE = DRIVE_MAX - DRIVE_MIN
DRIVE_NEUTRAL = DRIVE_RANGE // 2 + DRIVE_MIN


class AckermannSubscriber(Node):

    def __init__(self):
        super().__init__('ackman_subscriber')

        self.pwm = Adafruit_PCA9685.PCA9685()  # default i2c address: 0x40
        self.pwm.set_pwm_freq(60)  # Hz

        self.subscription = self.create_subscription(
            AckermannDrive,
            'ackermann_drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: speed=%s' % msg.speed)

        drive_pwm, steer_pwm = to_pwm(msg.speed, msg.steering_angle)

        self.pwm.set_pwm(CHANNEL_DRIVE, 0, drive_pwm)
        self.pwm.set_pwm(CHANNEL_STEER, 0, steer_pwm)

    def to_pwm(self, drive, steer):
        self.get_logger().info('drive=', drive, 'steer=', steer)

        if drive > 0.1:
            drive = 0.1

        steer_pwm = STEER_NEUTRAL - int(steer * STEER_RANGE)//2
        drive_pwm = DRIVE_NEUTRAL + int(drive * DRIVE_RANGE)//2
 
        return drive_pwm, steer_pwm

    def reset_pwm(self):
        self.pwm.set_pwm(CHANNEL_DRIVE, 0, DRIVE_NEUTRAL)


def main(args=None):
    rclpy.init(args=args)

    ackman_subscriber = AckermannSubscriber()

    rclpy.spin(ackman_subscriber)
    
    ackman_subscriber.reset_pwm()
    
    ackman_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
