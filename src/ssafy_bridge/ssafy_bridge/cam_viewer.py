#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_parser')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_parser = IMGParser()

    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()