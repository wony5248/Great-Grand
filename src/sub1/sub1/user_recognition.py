#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from os import listdir
from os.path import isdir, isfile, join
# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow
# 여러 사용자 학습
f_dir = 'C:\Ddrive/2021\ssafy2\Iot\git\S05P21A103\install\sub1\Lib\site-packages\sub1/'
face_classifier = cv2.CascadeClassifier(f_dir + 'haarcascade_frontalface_default.xml')


# 사용자 얼굴 학습
def train(name):
    data_path = f_dir + 'faces/' + name + '/'
    # 파일만 리스트로 만듬
    face_pics = [f for f in listdir(data_path) if isfile(join(data_path, f))]

    Training_Data, Labels = [], []

    for i, files in enumerate(face_pics):
        image_path = data_path + face_pics[i]
        images = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        # 이미지가 아니면 패스
        if images is None:
            continue
        Training_Data.append(np.asarray(images, dtype=np.uint8))
        Labels.append(i)
    if len(Labels) == 0:
        print("There is no data to train.")
        return None
    Labels = np.asarray(Labels, dtype=np.int32)
    # 모델 생성
    model = cv2.face.LBPHFaceRecognizer_create()
    # 학습
    model.train(np.asarray(Training_Data), np.asarray(Labels))
    print(name + " : Model Training Complete!!!!!")

    # 학습 모델 리턴
    return model

def trains():
    # faces 폴더의 하위 폴더를 학습
    data_path = f_dir + 'faces/'
    # 폴더만 색출
    model_dirs = [f for f in listdir(data_path) if isdir(join(data_path, f))]

    # 학습 모델 저장할 딕셔너리
    models = {}
    # 각 폴더에 있는 얼굴들 학습
    for model in model_dirs:
        print('model :' + model)
        # 학습 시작
        result = train(model)
        # 학습이 안되었다면 패스!
        if result is None:
            continue
        # 학습되었으면 저장
        print('model2 :' + model)
        models[model] = result

    # 학습된 모델 딕셔너리 리턴
    return models


# 얼굴 검출
def face_detector(img, size=0.5):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_classifier.detectMultiScale(gray, 1.3, 5)
    if faces is ():
        return img, []
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        roi = img[y:y + h, x:x + w]
        roi = cv2.resize(roi, (200, 200))
    return img, roi  # 검출된 좌표에 사각 박스 그리고(img), 검출된 부위를 잘라(roi) 전달


# 인식 시작
def run(models):
    # 카메라 열기
    cap = cv2.VideoCapture(1)

    while True:
        # 카메라로 부터 사진 한장 읽기
        ret, frame = cap.read()
        # 얼굴 검출 시도
        image, face = face_detector(frame)
        try:
            min_score = 999  # 가장 낮은 점수로 예측된 사람의 점수
            min_score_name = ""  # 가장 높은 점수로 예측된 사람의 이름

            # 검출된 사진을 흑백으로 변환
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)

            # 위에서 학습한 모델로 예측시도
            for key, model in models.items():
                result = model.predict(face)
                if min_score > result[1]:
                    min_score = result[1]
                    min_score_name = key

            # min_score 신뢰도이고 0에 가까울수록 자신과 같다는 뜻이다.
            if min_score < 500:
                # ????? 어쨋든 0~100표시하려고 한듯
                confidence = int(100 * (1 - (min_score) / 300))
                # 유사도 화면에 표시
                display_string = str(confidence) + '% Confidence it is ' + min_score_name
            cv2.putText(image, display_string, (100, 120), cv2.FONT_HERSHEY_COMPLEX, 1, (250, 120, 255), 2)
            # 75 보다 크면 동일 인물로 간주해 UnLocked!
            if confidence > 70:
                cv2.putText(image, "Name : " + min_score_name, (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Face Cropper', image)
            else:
                # 75 이하면 타인.. Locked!!!
                cv2.putText(image, "Locked", (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Face Cropper', image)
        except:
            # 얼굴 검출 안됨
            cv2.putText(image, "Face Not Found", (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('Face Cropper', image)
            pass
        if cv2.waitKey(1) == 13 or (cv2.waitKey(1) & 0xFF == ord('q')):
            break
    cap.release()
    cv2.destroyAllWindows()

class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        self.img_bgr = None
        self.models = trains()

    def img_callback(self, msg):
        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.img_bgr is not None:

            self.run(self.models)

        # cv2.imshow("img_bgr", self.img_bgr)
        #
        # cv2.waitKey(1)

    def run(self, models):

        # 얼굴 검출 시도
        image, face = face_detector(self.img_bgr)
        try:
            min_score = 999  # 가장 낮은 점수로 예측된 사람의 점수
            min_score_name = ""  # 가장 높은 점수로 예측된 사람의 이름

            # 검출된 사진을 흑백으로 변환
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)

            # 위에서 학습한 모델로 예측시도
            for key, model in models.items():
                result = model.predict(face)
                if min_score > result[1]:
                    min_score = result[1]
                    min_score_name = key

            # min_score 신뢰도이고 0에 가까울수록 자신과 같다는 뜻이다.
            if min_score < 500:
                # ????? 어쨋든 0~100표시하려고 한듯
                confidence = int(100 * (1 - (min_score) / 300))
                # 유사도 화면에 표시
                display_string = str(confidence) + '% Confidence it is ' + min_score_name
            cv2.putText(image, display_string, (100, 120), cv2.FONT_HERSHEY_COMPLEX, 1, (250, 120, 255), 2)
            # 75 보다 크면 동일 인물로 간주해 UnLocked!
            if confidence > 70:
                cv2.putText(image, "Name : " + min_score_name, (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),
                            2)
                cv2.imshow('Face Cropper', image)
            else:
                # 75 이하면 타인.. Locked!!!
                cv2.putText(image, "Locked", (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Face Cropper', image)
        except:
            # 얼굴 검출 안됨
            cv2.putText(image, "Face Not Found", (250, 450), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('Face Cropper', image)
            pass

        cv2.waitKey(1)
        # if cv2.waitKey(1) == 13 or (cv2.waitKey(1) & 0xFF == ord('q')):
        #     break


def main(args=None):
    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':
    main()
