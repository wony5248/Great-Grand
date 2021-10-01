#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# Detection lib
import torch
import time
from .FallDownDetection.DetectorLoader import TinyYOLOv3_onecls
from .FallDownDetection.Track.Tracker import Detection, Tracker
from .FallDownDetection.ActionsEstLoader import TSSTG
from .FallDownDetection.PoseEstimateLoader import SPPE_FastPose
from .FallDownDetection.Detection.Utils import ResizePadding
from .FallDownDetection.fn import draw_single

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

        # DETECTION MODEL.
        self.inp_dets = 384
        self.detect_model = TinyYOLOv3_onecls(self.inp_dets, device='cuda')

        # POSE MODEL.
        print("POSE MODEL")
        self.inp_pose = ['224', '160']
        self.inp_pose = (int(self.inp_pose[0]), int(self.inp_pose[1]))
        self.pose_model = SPPE_FastPose('resnet50', self.inp_pose[0], self.inp_pose[1], device='cuda')

        # Tracker.
        self.max_age = 30
        self.tracker = Tracker(max_age=self.max_age, n_init=3)

        # Actions Estimate.
        self.action_model = TSSTG()
        self.resize_fn = ResizePadding(self.inp_dets, self.inp_dets)

        self.show_detected = False
        self.show_skeleton = True

        self.fps_time = 0
        self.f = 0
        self.resize_fn = ResizePadding(self.inp_dets, self.inp_dets)

        print("Subscription")
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        self.img_bgr = None

        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def preproc(self, image):
        """preprocess function for CameraLoader.
        """
        image = self.resize_fn(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def img_callback(self, msg):

        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def detect_falldown(self, img_bgr):
        # Detection
        frame = self.preproc(img_bgr)

        # # Detect humans bbox in the frame with detector model.\
        # 사람인식 모델 : detect_model --> 인식 후 bbox 출력
        # detected의 리턴값: [top, left, bottom, right, bbox_score, class_score, class]
        detected = self.detect_model.detect(frame, need_resize=False, expand_bb=10)

        # Predict each tracks bbox of current frame from previous frames information with Kalman filter.
        # 이전 프레임의 정보를 토대로 kalman filter를 사용하여 현재 프레임의 bbox를 예측
        # kalman filter란 물체의 시작부터 현재까지 센서 입력값과 제어 입력값을 이용하여 현재 상태를 확률적으로 추정하는 것
        # 특히 kalman filter는 일부 동적시스템에 대해서 정보가 확실하지 않은 곳에서 사용할 수 있고 노이즈를 제거하는데 좋은 역활을 한다.
        # 이 이상 설명을 하려면 수식으로 깊게 들어가야하기때문에...이정도만...!더 자료 찾아보셔도 좋습니다...!
        self.tracker.predict()
        # Merge two source of predicted bbox together.
        # tracker에서 예측한 bbox와 실제 현재 프레임의 bbox를 합침
        for track in self.tracker.tracks:
            det = torch.tensor([track.to_tlbr().tolist() + [0.5, 1.0, 0.0]], dtype=torch.float32)
            # 만일 detected가 None이 아니면 detected의 배열과 det을 합치고 아니라면 det만 detected에 넣기
            detected = torch.cat([detected, det], dim=0) if detected is not None else det

        detections = []  # List of Detections object for tracking.
        if detected is not None:
            # detected = non_max_suppression(detected[None, :], 0.45, 0.2)[0]
            # Predict skeleton pose of each bboxs.
            # 위의 담긴 bbox를 토대로 skeleton을 디텍션
            '''
            AlphaPose사용 - Real-Time 환경에서 Multi-Person Pose Estimation 및 Tracking 이 가능한 오픈 시스템
            
            return값
            'bbox'
            'bbox_score'
            'keypoints'
            'kp_score'
            'proposal_score'
            '''
            poses = self.pose_model.predict(frame, detected[:, 0:4], detected[:, 4])

            # Create Detections object.
            # detection은 poses에서 받아온 값을 한번 정리 후에 bbox의 center와 범위등을 알기 위해 사용된다.
            detections = [Detection(self.kpt2bbox(ps['keypoints'].numpy()),
                                    np.concatenate((ps['keypoints'].numpy(),
                                                    ps['kp_score'].numpy()), axis=1),
                                    ps['kp_score'].mean().numpy()) for ps in poses]

            # VISUALIZE.
            # cv에 bbox기준으로 사각형 치는 코드(현재는 밑에서 한꺼번에 해주기때문에 쓰이지 않는다.)
            if self.show_detected:
                for bb in detected[:, 0:5]:
                    frame = cv2.rectangle(frame, (bb[0], bb[1]), (bb[2], bb[3]), (0, 0, 255), 1)

        # Update tracks by matching each track information of current and previous frame or
        # create a new track if no matched.
        # detections에 저장된 값으로 tracker의 값들을 update한다 --> 이 후 이전 값을 토대로 현재를 예측하는 kalman filter에 적용하기 위해
        self.tracker.update(detections)

        # Predict Actions of each track.
        # 여기부터 Actions(넘어졌는지 판단) 이건 아까 설명과 비슷합니다.
        # 필요하시면 최대한 빨리 드릴게요...!
        for i, track in enumerate(self.tracker.tracks):
            if not track.is_confirmed():
                continue

            track_id = track.track_id
            bbox = track.to_tlbr().astype(int)
            center = track.get_center().astype(int)

            action = 'pending..'
            clr = (0, 255, 0)
            # Use 30 frames time-steps to prediction.
            if len(track.keypoints_list) == 30:
                pts = np.array(track.keypoints_list, dtype=np.float32)
                out = self.action_model.predict(pts, frame.shape[:2])
                action_name = self.action_model.class_names[out[0].argmax()]
                action = '{}: {:.2f}%'.format(action_name, out[0].max() * 100)
                if action_name == 'Fall Down':
                    clr = (255, 0, 0)
                elif action_name == 'Lying Down':
                    clr = (255, 200, 0)

            # VISUALIZE.
            # 이제까지 찾은 bbox와 스켈레톤을 시각화 및 텍스트 삽입
            if track.time_since_update == 0:
                if self.show_skeleton:
                    frame = draw_single(frame, track.keypoints_list[-1])
                frame = cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 1)
                frame = cv2.putText(frame, str(track_id), (center[0], center[1]), cv2.FONT_HERSHEY_COMPLEX,
                                    0.4, (255, 0, 0), 2)
                frame = cv2.putText(frame, action, (bbox[0] + 5, bbox[1] + 15), cv2.FONT_HERSHEY_COMPLEX,
                                    0.4, clr, 1)

        '''
        로직 4. 이미지 resizing
        cv2를 사용해서 이미지를 원하는 크기로 바꿔보십시오.
        '''
        frame = cv2.resize(frame,(640, 480), interpolation=cv2.INTER_CUBIC)
        frame = cv2.putText(frame, '%d, FPS: %f' % (self.f, 1.0 / (time.time() - self.fps_time)),
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        frame = frame[:, :, ::-1]
        self.fps_time = time.time()


        # 로직 5. 이미지 출력 (cv2.imshow)       
        
        #cv2.imshow("img_bgr", img_bgr)
        #cv2.imshow("img_gray", img_gray)
        cv2.imshow("resize and gray", frame)
        
        cv2.waitKey(1)

    def kpt2bbox(self, kpt, ex=20):
        """Get bbox that hold on all of the keypoints (x,y)
        kpt: array of shape `(N, 2)`,
        ex: (int) expand bounding box,
        """
        return np.array((kpt[:, 0].min() - ex, kpt[:, 1].min() - ex,
                         kpt[:, 0].max() + ex, kpt[:, 1].max() + ex))

    def timer_callback(self):

        if self.img_bgr is not None:

            self.detect_falldown(self.img_bgr)
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
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()