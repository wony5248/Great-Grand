import numpy as np
import cv2
import socket

import rclpy
from rclpy.node import Node
from ssafy_bridge.utils import UDP_CAM_Parser
from sensor_msgs.msg import CompressedImage


params_cam_0 = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65535),
    "UnitBlock_HEIGHT": int(30),
    "X": 1.7, # meter
    "Y": 0,
    "Z": 1.2,
    "YAW": 0, # deg
    "PITCH": -5,
    "ROLL": 0
}


class IMGPublisher(Node):

    def __init__(self):
        super().__init__(node_name='IMGPublisher')

        self.publisher_ = self.create_publisher(CompressedImage, '/image_jpeg/compressed', 10)
        self.udp_parser = UDP_CAM_Parser(self.publisher_, ip=params_cam_0["localIP"], port=params_cam_0["localPort"], params_cam=params_cam_0)        


def main(args=None):

    rclpy.init(args=args)
    image_parser = IMGPublisher()
    rclpy.spin(image_parser)    
    image_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()