#!/ C:\Python37\python.exe

import numpy as np
import cv2
import os
import rclpy
import socketio
import base64

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

sio = socketio.Client()

@sio.event
def connect():
    print('connection established')

@sio.event
def disconnect():
    print('disconnected from server')

def non_maximum_supression(bboxes, threshold=0.5):
    
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    new_bboxes=[]
    
    new_bboxes.append(bboxes[0])
    
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
            
            x_overlap = max(0, min(x1_br, x2_br)-max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br)-max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap
            
            area_1 = bbox[2] * new_bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2 - overlap_area

            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:
                
                new_bboxes.append(bbox)

    return new_bboxes



class HumanDetectorToServer(Node):

    def __init__(self):
        super().__init__('detect_to_server')

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
        
        sio.connect('http://127.0.0.1:12001')

        cv2.imwrite(self.dir_img, np.zeros((240, 320, 3)).astype(np.uint8))


    def img_callback(self, msg):
    
        self.byte_data = msg.data

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  

    def detect_human(self):
    
        img_pre = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_pre, winStride=(2, 2), padding=(8, 8), scale=2)

        if len(rects_temp) != 0:

            rects = non_maximum_supression(rects_temp)

            self.human_detected = True

            for (x,y,w,h) in rects:

                cv2.rectangle(self.img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)

        cv2.imshow("detection result", self.img_bgr)

        cv2.waitKey(1)

        
    def timer_callback(self):

        if self.img_bgr is not None:

            print("subscribed")

            self.detect_human()

            b64data = base64.b64encode(self.byte_data)

            if self.human_detected:

                self.byte_data = cv2.imencode('.jpg', self.img_bgr)[1].tobytes()

            sio.emit('streaming', b64data.decode( 'utf-8' ) )

            if self.human_detected:

                str_to_web = "house intruder detected"

            else:

                str_to_web = "safe"

            sio.emit('safety_status', str_to_web)

        else:

            pass
        

def main(args=None):
    
    rclpy.init(args=args)

    human_detector = HumanDetectorToServer()

    rclpy.spin(human_detector)

    human_detector.destroy_node()
    
    rclpy.shutdown()

    sio.disconnect()


if __name__ == '__main__':
    main()

