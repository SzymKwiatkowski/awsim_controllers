import rclpy
from rclpy.node import Node

from autoware_auto_control_msgs.msg import AckermannControlCommand


class Validator(Node):

    def __init__(self):
        super().__init__('validate_topic_sent')
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.longitudinal.speed)


def main(args=None):
    rclpy.init(args=args)

    validator = Validator()

    rclpy.spin(validator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    validator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()