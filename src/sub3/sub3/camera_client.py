#!/ C:\Python37\python.exe

import numpy as np
import cv2
import os
import rclpy
import socketio
import base64
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

sio = socketio.Client()

@sio.event
def connect():
    print('connection established')

@sio.event
def disconnect():
    print('disconnected from server')


class HumanDetectorToServer(Node):

    def __init__(self):
        print("init started")
        super().__init__('camera_client')
    
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        self.byte_data = None

        self.img_bgr = None
        
        self.timer_period = 0.03

        # self.img_saved = False

        self.human_detected = False

        self.dir_img = os.path.join("C:\\Users\\user\\Desktop\\catkin_ws\\src\\security_service\\web\\client", "detect.png")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.pedes_detector = cv2.HOGDescriptor()
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # sio.connect('http://127.0.0.1:12001')
        sio.connect('https://j5a103.p.ssafy.io/io')

        cv2.imwrite(self.dir_img, np.zeros((240, 320, 3)).astype(np.uint8))


    def img_callback(self, msg):
    
        self.byte_data = msg.data
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("img_test", self.img_bgr) 
        
        self.timer_callback()
 

        
    def timer_callback(self):

        if self.img_bgr is not None:

            print("subscribed")

            # self.detect_human()

            b64data = base64.b64encode(self.byte_data)

            # if self.human_detected:

            self.byte_data = cv2.imencode('.jpg', self.img_bgr)[1].tobytes()

            sio.emit('streaming', b64data.decode( 'utf-8' ) )
            cv2.waitKey(1)

def main(args=None):
    print("main 1")
    rclpy.init(args=args)
    print("main 2")
    human_detector = HumanDetectorToServer()
    print("main 3")
    rclpy.spin(human_detector)
    print("main 4")
    human_detector.destroy_node()
    cv2.destroyAllWindows()
    print("main 5")
    rclpy.shutdown()
    print("main 6")
    sio.disconnect()



if __name__ == '__main__':
    print("main started")
    main()

