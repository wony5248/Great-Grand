
import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Int8MultiArray
from ssafy_bridge.ssafy_udp_parser import erp_udp_sender,handControlSender,app_control_sender
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ssafy_msgs.msg import HandControl

dst_ip='127.0.0.1'

class ssafy_bridge(Node):

    def __init__(self):
        super().__init__('ssafy_bridge_sub')

        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)
        self.hand_sub = self.create_subscription(HandControl,'hand_control',self.hand_callback,10)
        self.app_control_subscriber= self.create_subscription(Int8MultiArray,'app_control',self.app_control_callback,10)



        self.ctrl_cmd=erp_udp_sender(dst_ip,7601)
        self.app_control=app_control_sender(dst_ip,7901)

        self.hand_control=handControlSender(dst_ip,8101)
      

    def hand_callback(self,msg):
        self.hand_control.send_data(msg.control_mode,msg.put_distance,msg.put_height)
        # print(msg.control_mode,msg.put_distance,msg.put_height)

    def listener_callback(self, msg):
        print(msg.linear.x,msg.angular.z)
        self.ctrl_cmd.send_data(msg.linear.x,msg.angular.z)

    def app_control_callback(self, msg):
        control_data=msg.data
        self.app_control.send_data(control_data)


def main(args=None):
    rclpy.init(args=args)
    bridge = ssafy_bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()