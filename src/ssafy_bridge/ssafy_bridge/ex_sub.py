import rclpy
from rclpy.node import Node

from ssafy_msgs.msg import TurtlebotStatus #타입 이름 가져오기


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            TurtlebotStatus, #타입 이름
            'turtlebot_status', #토픽 이름
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Battert_Percentage: "%s"' % msg.battery_percentage)
        self.get_logger().info('Power_Supply_Status: "%s"' % msg.power_supply_status)
        self.get_logger().info('Angular: "%s"' % msg.twist.angular.z)
        self.get_logger().info('Linear: "%s"' % msg.twist.linear.x)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()