
import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Int8MultiArray
from ssafy_bridge.ssafy_udp_parser import erp_udp_parser  

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from ssafy_msgs.msg import EnviromentStatus,TurtlebotStatus,CustomObjectInfo

user_ip='127.0.0.1'

class ssafy_bridge(Node):

    def __init__(self):
        super().__init__('ssafy_bridge')
        
        #envir
        self.envir_publisher = self.create_publisher(EnviromentStatus, 'envir_status', 10)
        self.envir_udp=erp_udp_parser(self.envir_publisher,user_ip, 7802,'envir_status')

        #application
        self.app_status_publisher = self.create_publisher(Int8MultiArray, 'app_status', 10)
        self.app_udp=erp_udp_parser(self.app_status_publisher,user_ip, 8002,'app_status')

        #turtlebot status
        self.ego_status_publisher = self.create_publisher(TurtlebotStatus, 'turtlebot_status', 10)
        self.ego_status_udp=erp_udp_parser(self.ego_status_publisher,user_ip, 8202,'turtlebot_status')

        # imu
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.imu_udp=erp_udp_parser(self.imu_publisher,user_ip, 9092,'imu') 
        
        # custom object info
        self.obj_publisher = self.create_publisher(CustomObjectInfo, 'custom_object_info', 10)
        self.obj_udp=erp_udp_parser(self.obj_publisher,user_ip, 8302,'custom_object') 


def main(args=None):
    rclpy.init(args=args)
    bridge = ssafy_bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()