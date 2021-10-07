import rclpy
from rclpy.node import Node

from squaternion import Quaternion
from nav_msgs.msg import Odometry


class odom(Node):

    def __init__(self):
        super().__init__('odom')

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='map'
        self.time=0.0

    def timer_callback(self):
        x=self.time
        q=Quaternion.from_euler(0,0,0)
        self.odom_msg.pose.pose.position.x=x
        self.odom_msg.pose.pose.position.y=0.0
        self.odom_msg.pose.pose.orientation.x=q.x
        self.odom_msg.pose.pose.orientation.y=q.y
        self.odom_msg.pose.pose.orientation.z=q.z
        self.odom_msg.pose.pose.orientation.w=q.w
        self.odom_publisher.publish(self.odom_msg)
        self.time+=0.1


def main(args=None):
    rclpy.init(args=args)

    odom_node = odom()

    rclpy.spin(odom_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()