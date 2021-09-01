import numpy as np
import cv2
import socket

import rclpy
from rclpy.node import Node
from ssafy_bridge.utils import UDP_LIDAR_Parser

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32


params_lidar = {
    "CHANNEL" : int(1),
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206)
}


class PCPublisher(Node):

    def __init__(self):
        super().__init__(node_name='PCPublisher')
        self.publisher_laser = self.create_publisher(LaserScan, '/scan', 5)
        self.udp_parser = UDP_LIDAR_Parser(self.publisher_laser,ip=params_lidar["localIP"], port=params_lidar["localPort"], params_lidar=params_lidar)        

        self.timer_period = 1/60  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        
        ranges, intens, aux_data = self.udp_parser.recv_udp_data()

        if len(ranges)==0:
            pass

        else:
            self.pc_msg = LaserScan()
            self.pc_msg.header.frame_id = "laser"
            self.pc_msg.angle_min = 0.0
            self.pc_msg.angle_max = 360.0
            self.pc_msg.angle_increment = np.pi/180           
            self.pc_msg.range_max = 10.0

            self.pc_msg.ranges = ranges.astype(np.float32).tolist()
            self.pc_msg.ranges[-1]=0.0
            self.pc_msg.intensities = intens.astype(np.float32).tolist()
            
            self.pose_msg = Float32MultiArray()                
            self.pose_msg.data = aux_data.astype(np.float32).tolist()       
            self.pc_msg.range_min = self.pose_msg.data[0] #x
            self.pc_msg.scan_time=self.pose_msg.data[1]   #y
            self.pc_msg.time_increment=self.pose_msg.data[2] #heading

            self.publisher_laser.publish(self.pc_msg)







        

def main(args=None):

    rclpy.init(args=args)
    pc_parser = PCPublisher()
    rclpy.spin(pc_parser)
    pc_parser.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':

    main()