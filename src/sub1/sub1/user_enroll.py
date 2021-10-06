#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from pathlib import Path
# face
from os import makedirs
from os.path import isdir

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')
        print(cv2.__version__)
        self.face_dirs = 'faces/'
        source = str(Path.cwd())
        source += "\haarcascade_frontalface_default.xml"
        print(source)
        self.face_classifier = cv2.CascadeClassifier(source)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        self.img_bgr = None
        self.count = 0
        print("init ok")

    def face_extractor(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_classifier.detectMultiScale(gray, 1.3, 5)
        # 얼굴이 없으면 패스!
        if faces is ():
            return None
        # 얼굴이 있으면 얼굴 부위만 이미지로 만들고
        for (x, y, w, h) in faces:
            cropped_face = img[y:y + h, x:x + w]
        # 리턴!
        return cropped_face

    # 얼굴만 저장하는 함수
    def take_pictures(self, name):
        # 해당 이름의 폴더가 없다면 생성
        if not isdir(self.face_dirs + name):
            makedirs(self.face_dirs + name)

        # 카메라 ON
        # cap = cv2.VideoCapture(1)
        # cap.open(1)
        # count = 0
        # while True:
            # 카메라로 부터 사진 한장 읽어 오기
            # ret, frame = cap.read()
            # 사진에서 얼굴 검출 , 얼굴이 검출되었다면
        if self.face_extractor(self.img_bgr) is not None:

            self.count += 1
            # 200 x 200 사이즈로 줄이거나 늘린다음
            face = cv2.resize(self.face_extractor(self.img_bgr), (200, 200))
            # 흑백으로 바꿈
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)

            # 200x200 흑백 사진을 faces/얼굴 이름/userxx.jpg 로 저장
            file_name_path = self.face_dirs + name + '/user' + str(self.count) + '.jpg'
            cv2.imwrite(file_name_path, face)

            cv2.putText(face, str(self.count), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Face Cropper', face)
        else:
            print("Face not Found")
            pass

        # 얼굴 사진 100장을 다 얻었거나 enter키 누르면 종료
        # if cv2.waitKey(1) == 13 or count == 100:
        #     break
        cv2.waitKey(1)
        # cap.release()
        # cv2.destroyAllWindows()
        print('Colleting Samples Complete!!!')


    def img_callback(self, msg):
        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.img_bgr is not None:

            self.take_pictures('user1')
            # # 로직 8 : bbox msg 송신s
            # self.bbox_pub_.publish(self.bbox_msg)

        else:
            pass


def main(args=None):
    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    if image_parser.count < 100:
        rclpy.spin(image_parser)
    else:
        image_parser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
