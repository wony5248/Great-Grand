
import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped,PoseWithCovarianceStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import numpy as np
import cv2
import time
import sub3.utils as utils

# localization node의 전체 로직 순서
# 1 : 노드에 필요한 publisher, subscriber, transform broadcaster 생성
# 2 : localization 클래스 생성
# 3 : parameter 입력 받고 클래스 정의
# 4 : 저장된 맵 load
# 5 : 초기 위치 수신
# 6 : odometry 수신 및 업데이트
# 7 : lidar scan 결과 수신
# 8 : localization 클래스 실행
# 9 : particles 초기화
# 10 : prediction
# 11 : weighting
# 12 : resampling
# 13 : particle 시각화하기
# 14 : 위치 추정 결과를 publish


params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-8.0, -4.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAPVIS_RESIZE_SCALE": 2.0
}


params_mcl = {
    "NUM_PARTICLE": 500,
    "MIN_ODOM_DISTANCE": 0.1,
    "MIN_ODOM_ANGLE": 1,
    "REPROPAGETE_COUNT": 1,
    "ODOM_TRANSLATION_COVARIANCE": 0.025,
    "ODOM_HEADING_COVARIANCE": 0.001   # 0.0005
}



def compute_relative_pose(pose_i, pose_j):

    # 로봇의 상대 좌표에 대한 pose를 내놓는 역할을 합니다
    
    # 로직 순서
    # 1. 두 pose를 기준으로 하는 좌표변환 행렬 정의
    # 2. Rot 행렬, trans 백터 추출
    # 3. pose i 기준 pose j에 대한 Rot 행렬, trans 백터 계산
    # 4. 위의 Rot 행렬, trans 백터로 pose_ij를 계산    
        
    """
    로직 1 : 두 pose를 기준으로 하는 좌표변환 행렬 정의
    T_i = xyh2mat2D(pose_i)
    T_j = xyh2mat2D(pose_j)

    """
    
    """
    로직 2 : Rot 행렬, trans 백터 추출
    R_i = 
    t_i = 

    """

    """
    로직 3 : pose i 기준 pose j에 대한 Rot 행렬, trans 백터 계산
    R_ij = 
    t_ij = 
    
    """

    """
    로직 4 : 위의 Rot 행렬, trans 백터로 pose_ij를 계산
    
    pose_ij = np.array([0.0, 0.0, 0.0])    
    pose_ij[:2] = 
    pose_ij[2] = 

    """

    """
    테스트

    pose_i = np.array([2, 3, 30])
    pose_j = np.array([3, 5, 60]) 이면
    
    T_i = [[ 0.8660254 -0.5        2.       ]
            [ 0.5        0.8660254  3.       ]
            [ 0.         0.         1.       ]]
    
    T_j = [[ 0.5       -0.8660254  3.       ]
            [ 0.8660254  0.5        5.       ]
            [ 0.         0.         1.       ]]
    
    R_i = [[ 0.8660254  0.5      ]
            [-0.5        0.8660254]]
    
    t_i = [-3.23205081 -1.59807621]

    R_ij = [[ 0.8660254 -0.5      ]
            [ 0.5        0.8660254]]

    t_ij = [1.8660254  1.23205081]

    pose_ij = [ 1.8660254, 1.23205081, 30. ]

    가 나와야한다

    """    

    return pose_ij


class Localization:

    def __init__(self, params_map, params_mcl):

        # 로직 3. parameter 입력 받고 클래스 정의
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.num_particle = params_mcl["NUM_PARTICLE"]
        self.min_odom_dist = params_mcl["MIN_ODOM_DISTANCE"]
        self.min_odom_angle = params_mcl["MIN_ODOM_ANGLE"]
        self.repropagate_count = params_mcl["REPROPAGETE_COUNT"]
        self.odom_translation_cov = params_mcl["ODOM_TRANSLATION_COVARIANCE"]
        self.odom_heading_cov = params_mcl["ODOM_HEADING_COVARIANCE"]

        self.is_init = False
        self.odom_before = np.zeros(3)
        self.odom_cov = np.array([self.odom_translation_cov, self.odom_translation_cov, self.odom_heading_cov])


        self.load_map()

        self.map_bgr = np.empty((300,300,3))
        self.current_pose = np.array([0.0, 0.0, 0.0]) # Current pose [x, y, theta]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        self.prediction_cnt = 0
        self.stable_score=0.0

    def load_map(self):

        # 로직 4. 저장된 맵을 불러옵니다.
        pkg_path =os.getcwd()
        back_folder='..'
        folder_name='map'
        file_name='map.txt'
        full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
        f=open(full_path,'r')

        line=f.readlines()
        line_data=line[0].split()
        f.close()
       
        map_data = [0 for i in range(int(self.map_size[0]*self.map_size[1]))]

        for num,data in enumerate(line_data) :
            map_data[num]=int(data)
   
        
        np_map_data=np.array(map_data)
        self.map=np.reshape(np_map_data,(350, 350))
        self.map=((100-self.map)*255.0/100.0).astype(np.uint8)

        
        self.map_bgr = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)
        self.map_bgr = self.map_bgr.copy()
        self.map = self.map*1.0/256


    def initialize_particles(self, pose=None, factor=None):

        # 로직 9. particles 초기화
        # 파티클을 x, y, theta, score로 이루어진 array로 랜덤하게 생성시켜 초기화 시키는 단계를 말합니다.
        # rviz2 에서 지정한 수신 받은 초기 위치 기준으로 초기 파티클을 랜덤 생성합니다.
                
        rows, cols = self.map.shape
        self.max_prob_particle = np.zeros((4, 1))
        self.particles = np.zeros((4, self.num_particle)) # [x, y, theta, score]

        if factor is None:
            sfactor = 1.0
        else:
            sfactor = 10.0

        for i in range(self.num_particle):
            if pose is None:
                init_center_x = self.map_center[0]
                init_center_y = self.map_center[1]
            else:
                init_center_x = pose[0]
                init_center_y = pose[1]

            rand_x = np.random.uniform(init_center_x-cols*self.map_resolution/sfactor,
                                       init_center_x+cols*self.map_resolution/sfactor,
                                       1)
            rand_y = np.random.uniform(init_center_y-rows*self.map_resolution/sfactor,
                                       init_center_y+rows*self.map_resolution/sfactor,
                                       1)
            rand_h = np.random.uniform(-np.pi, np.pi, 1)*180.0/np.pi

            self.particles[:3, i]=[rand_x, rand_y, rand_h]

        self.particles[3, :] = 1/self.num_particle # Initialize score
        self.show_particle(self.particles)

        tmp_pose = self.particles[:3, i]
        T = utils.xyh2mat2D(tmp_pose)
        (rand_x, rand_y, rand_h) = self.particles[:3, i]


    def show_particle(self, particles, laser=None, gt_pose=None):

        # 로직 13. 시각화하기
        # 맵 상에서 파티클로 정의된 pose들을 보여주기 위한 용도입니다. 
        # 이 스켈레톤은 단순히 맵만 cv2.imshow로 보여주도록 되어 있으며, 
        # 이 위에 파티클과 라이다 포인트들을 비주얼라이즈하도록 수정해볼 수 있습니다        
        rows, cols = self.map.shape
        tmp_map = self.map*256
        tmp_map = tmp_map.astype(np.uint8)
        self.map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        map_bgr_with_particle = self.map_bgr.copy()

        for i in range(self.num_particle):
            (rand_x, rand_y, rand_h) = particles[:3, i]
            # print(rand_x)
            pos_x = (rand_x - self.map_center[0] + (rows * self.map_resolution)/2)/self.map_resolution
            pos_y = (rand_y - self.map_center[1] + (cols * self.map_resolution)/2)/self.map_resolution
            # print(pos_x)
            # TODO : check rows and cols
            if pos_y.astype(np.int) > 0 and pos_x.astype(np.int) > 0 and pos_y.astype(np.int) < rows and pos_x.astype(np.int) < cols:
                map_bgr_with_particle.itemset(pos_y.astype(np.int), pos_x.astype(np.int),0, 0)
                map_bgr_with_particle.itemset(pos_y.astype(np.int), pos_x.astype(np.int),1, 255)
                map_bgr_with_particle.itemset(pos_y.astype(np.int), pos_x.astype(np.int),2, 0)
            # print(rand_x)

        pose = self.max_prob_particle[:3]
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        center = (int(pose_x),int(pose_y))
        
        cv2.circle(map_bgr_with_particle, center, 2, (0,0,255), -1)

        if laser is not None:
            pose_mat = utils.xyh2mat2D(pose)
            T_h_bar = np.matmul(pose_mat, self.T_r_l)
            
            laser_mat = np.ones((3, laser.shape[1]))
            laser_mat[:2, :] = laser

            laser_global = np.matmul(T_h_bar, laser_mat)
            laser_global_x = (laser_global[0, :] - self.map_center[0] + (
                        self.map_size[0] * self.map_resolution) / 2) / self.map_resolution
            laser_global_y = (laser_global[1, :] - self.map_center[1] + (
                        self.map_size[1] * self.map_resolution) / 2) / self.map_resolution

            for i in range(laser_global.shape[1]):
                (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
                center = (l_x, l_y)
                cv2.circle(map_bgr_with_particle, center, 1, (255, 0, 0), -1)

        if gt_pose is not None:
            pose_x = (gt_pose[0] - self.map_center[0] + (self.map_size[0] * self.map_resolution) / 2) / self.map_resolution
            pose_y = (gt_pose[1] - self.map_center[1] + (self.map_size[1] * self.map_resolution) / 2) / self.map_resolution
           
            center = (int(pose_x),int(pose_y))
            cv2.circle(map_bgr_with_particle, center, 2, (255,0,255), -1)

        map_bgr_with_particle = cv2.resize(map_bgr_with_particle, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        cv2.imshow('Sample Map', map_bgr_with_particle)
        cv2.waitKey(1)

    def _prediction(self, diff_pose):
        """

        # 로직 10. prediction
        # 파티클 필터의 예측단계를 수행하는 파트이며,
        # 사전 지식 학습단계에 언급된 Thrun의 motion prediction model을 수식대로 구현하여 채우시면 되겠습니다

        # 테스트
        # diff_pose = [0.1, 0.2, 20] 이면
        # delta_dist = 0.22361
        # delta_angle1 = 1.1071
        # delta_angle1 = -0.7581

        delta_dist = 

        delta_angle1 = 
        delta_angle2 = 
        
        odom_cov = 

        delta_angle1 = utils.limit_angular_range(delta_angle1)

        dist_noise_coeff = 
        angle1_noise_coeff = 
        angle2_noise_coeff = 

        score_sum = 0

        for i in range(self.num_particle):

            delta_dist = 
            delta_angle1 = 
            delta_angle2 = 

            x = 
            y = 
            h = 

            diff_odom_noise = np.array([x, y, h*180.0/np.pi])
            diff_odom_T = utils.xyh2mat2D(diff_odom_noise)

            xyh_i = self.particles[:3, i]
            T_i = utils.xyh2mat2D(xyh_i)
            T_j = np.matmul(T_i, diff_odom_T)

            score_sum += self.particles[3, i]
            self.particles[:3, i] = utils.mat2D2xyh(T_j)

        """

        self.particles[3, :] = self.particles[3, :] #/ score_sum위 코드를 완성한 후 주석을 해제해 주세요

    def _points_in_map(self, x_list, y_list):

        # x,y좌표를 grid map에 표시하기 위해 행렬 인덱스로 바꿔주는 역할을 합니다
        points_global_x = (x_list - self.map_center[0] + (
                    self.map_size[0] * self.map_resolution) / 2) / self.map_resolution
        points_global_y = (y_list - self.map_center[1] + (
                    self.map_size[1] * self.map_resolution) / 2) / self.map_resolution

        return points_global_x, points_global_y

    def _weighting(self, points):

        ## 필수 사전 학습 단계서도 언급했듯이, 파티클 중 라이다로 측정된 것과 일치하도록 나오는 
        ## 파티클에 대해서 가중치를 주는 _weighting() 메소드를 완성시켜야 합니다.
        ##  일반적으로 라이다에서 측정된 거리들은 벽과의 거리인데, 이 라이다 포인트의 좌표들을
        ##  로봇의 pose와 합쳐서 맵에 매칭시키고, 실제 벽의 좌표와 맞는 포인트 개수가 많을수록
        ##  가중치를 높게 주시면 됩니다.
        ##  그리고 마지막에 가중치의 총합을 1로 바꾸십시오.

        max_score = 0.0
        score_sum = 0.0

        rows, cols = self.map.shape

        n_points = points.shape[1]
        point_mat = np.ones((3, n_points))
        point_mat[:2, :] = points
        weights = np.zeros(self.num_particle)

        for i in range(self.num_particle):
            T_h_bar = np.matmul(utils.xyh2mat2D(self.particles[:3, i]), self.T_r_l)

            particle_pose = self.particles[:3, i]
            particle_pose_x, particle_pose_y = self._points_in_map(particle_pose[0], particle_pose[1])

            # 파티클이 맵 안에 있는 경우
            is_pose_valid = (particle_pose_x>0) * (particle_pose_y>0) * (particle_pose_x<cols) * (particle_pose_y<rows)
            if not is_pose_valid:
                self.particles[3, i] = 0
                continue

            # 파티클이 occupied region (벽) 에 걸친 경우
            occupancy = self.map[particle_pose_y.astype(np.int), particle_pose_x.astype(np.int)]
            if occupancy < 0.75:
                self.particles[3, i] = 0
                continue

            # 좌표변환 후 글로벌좌표계 맵 기준으로 픽셀좌표를 구하십시오 
            transformed_points = np.matmul(T_h_bar, point_mat)
            points_global_x, points_global_y = self._points_in_map(transformed_points[0, :], transformed_points[1, :])

            """
            로직 11. weighting
            # 맵안에 존재하는 유효한 파티클들의 픽셀좌표만 남기고 그 픽셀 좌표 상에서의
            # map의 값만 골라오십시오
            valid_idx = 
            valid_x = 
            valid_y =  
            vals = self.map[valid_y.astype(np.int), valid_x.astype(np.int)]
            
            # 0.25라는 값을 기준으로 이상이면 wall, 이하면 empty로 정의하십시오
            walls = 
            empty = 

            # 라이다의 측정값인 laser scan의 위치가 실제 벽에 매칭이 많이 될수록
            # 그 파티클은 실제 위치에 가까운 것입니다.
            # walls==True인 개수를 weight로 정의하십시오.
            weight = 
            penalty = np.sum(empty)*0.0
            weight = weight - penalty
            weights[i] = weight

            self.particles[3, i] += weight / transformed_points.shape[1]
            score_sum += self.particles[3, i]

            """

            if max_score < self.particles[3, i]:
                self.max_prob_particle = self.particles[:,i]
                max_score = self.particles[3, i]

        ## 가장 높게 나타나는 score를 stable score로 save                
        self.stable_score= np.max(weights)
        print('max matching points', np.max(weights))

        for i in range(self.num_particle):
            self.particles[3,i] = self.particles[3,i]/(score_sum)


    def _resampling(self):

        # 모든 가중치의 cumulative sum list를 구하고 uniform distribution으로 상수를 
        # 파티클 갯수만큼 샘플링하면서, cumulative sum 값에 해당되는 인덱스인 파티클만 
        # 골라내는 작업을 말합니다.
        # 구현이 제대로 되었으면 _weighting() 단계에서 가중치가 높게 나온 파티클들이
        # 주로 재생성 되는 걸 확인 할 수 있을 겁니다

        n_particles = self.num_particle
        score_basline = 0.0
        particle_scores = np.zeros([n_particles])
        selected_particles = np.zeros(self.particles.shape)
        
        for i in range(n_particles):
            score_basline += self.particles[3, i]
            particle_scores[i] = score_basline

   
        """
        로직 12. resampling
        np.random.uniform로 파티클 개수만큼 난수를 뽑은 다음, 그 난수에 
        
        for i in range(n_particles):
            darted_score = 
            darted_idx = 
            
            selected_particles[:, i] = self.particles[:, darted_idx]

        """

        self.particles = selected_particles.copy()

    def update(self, pose, laser):

        # 위의 단계들을 모두 완성시키고, diff_pose를 wheel odometry로 받아서 정의하도록 
        # 수정하면 monte carlo localization 이 완성이 됩니다. 
        # 해당 스켈레톤 코드에는 diff_pose와 diff_dist, diff_angle을 계속 0과 0.1로만 주고 있어서
        # 파티클 필터가 업데이트되고 있지 않습니다

        # 로직 9. particle 초기화하기
        if self.is_init != True:
            self.odom_before = pose
            self.initialize_particles(pose, 10.0)
            
            self.is_init = True
            return


        """diff_pose =""" 
        diff_pose = np.array([[0],[0],[0]])

        """
        diff_dist = 
        diff_angle = 
        """
        diff_dist = 0.1
        diff_angle = 0


        if diff_dist > 1 :
            print('reset')
            self.is_init=False
            return

        if diff_dist > self.min_odom_dist or np.abs(diff_angle) > self.min_odom_angle:
            self._prediction(diff_pose)
            self._weighting(laser)

            self.prediction_cnt += 1
            
            if self.prediction_cnt == self.repropagate_count:
                self._resampling()
                self.prediction_cnt = 0


            
            self.odom_before = pose

            self.show_particle(self.particles, laser, pose)
            prob_pose=self.max_prob_particle[0:3]
            return prob_pose



class Localizer(Node):

    def __init__(self):

        # 로직 3. 노드에 필요한 publisher, subscriber, transform broadcaster 생성
        super().__init__('Localizer')
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,1)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
        '/initialpose',self.init_pose_callback,1)
        self.imu_sub = self.create_subscription(Imu,
        '/imu',self.imu_callback,1)
        self.turtle_sub = self.create_subscription(TurtlebotStatus,
        '/turtlebot_status',self.turtlebot_status_callback,1)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.odom_msg=Odometry()
        self.is_init_pose=False
        self.is_status=False
        self.is_imu=False
        self.x=0.0
        self.y=0.0
        self.heading=0.0
        self.prev_amcl_pose=np.array([[-9.4],[-7.7],[0.0]])

        self.amcl_pose=None

 

        self.odom_msg.header.frame_id='map'
        self.odom_msg.child_frame_id='base_link'


        self.localizer_transform=TransformStamped()
        self.localizer_transform.header.frame_id = "map"
        self.localizer_transform.child_frame_id = "base_link"

        self.laser_transform=TransformStamped()
        self.laser_transform.header.frame_id = "base_link"
        self.laser_transform.child_frame_id = "laser"      
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 0.6
        self.laser_transform.transform.rotation.w = 1.0


        # 로직 2. localization 클래스 생성
        self.localization = Localization(params_map, params_mcl)


        
    def imu_callback(self,msg):

        if self.is_imu ==False :    
            self.is_imu=True
            imu_q= Quaternion(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z)
            self.imu_offset=imu_q.to_euler()
        else :
            imu_q= Quaternion(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z)
            self.odom_theta=imu_q.to_euler()[2]+self.imu_offset[2]


    def init_pose_callback(self,msg):

        # 로직 5. 초기 위치 수신
        # rviz2에서 2d estimate pose 버튼으로 사용자가 원하는 초기 위치를 지정하면 작동합니다.
        # 초기 위치 선택 시 위에 생성된 subscriber 중 '/initialpose' 토픽으로
        # 들어오는 pose 메시지를 init_pose_callback() 안에서 처리합니다. 

        if msg.header.frame_id=='map' :
       
            q=Quaternion(msg.pose.pose.orientation.w,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z)
            _,_,self.odom_theta=q.to_euler()
            self.odom_x=msg.pose.pose.position.x
            self.odom_y=msg.pose.pose.position.y   
            self.is_init_pose=True

    def turtlebot_status_callback(self, msg):

        """
        로직 6. odometry 수신 및 업데이트
        """        
        #turtlebot_status로 odometry를 받는 과정
        if self.is_init_pose ==True and self.is_imu==True:
            if self.is_status == False :
                # 첫번째 스텝에서 is_status는 False로 시작하며 들어가고 prev_time만 save 하는 과정
                self.prev_time=rclpy.clock.Clock().now()

                # prev_time가 기록되면 이 다음에 status msg에서 odometry를 받기 시작한다.

                self.is_status=True
            else :

                # 두번째 스텝부터 odom 정보 파싱
                # 이전 스텝과 현재 스텝간의 시간 차이 계산
                self.current_time=rclpy.clock.Clock().now()
                self.period=(self.current_time-self.prev_time).nanoseconds/(1e+9)


                """
                turtlebot_status의 정보들로 선속도를 얻고
                imu callback에서 처리한 odom_theta로 로봇의 헤딩을 얻어서
                odom_theta, odom_x, odom_y 업데이트

                linear_x =     
                self.odom_x+=
                self.odom_y+=
                """
                self.prev_time=self.current_time



    def scan_callback(self,msg):

        if self.is_status==True :

            # turtlebot status 로부터 업데이트한 odom 정보    
            pose_x=self.odom_x
            pose_y=self.odom_y
            heading=(self.odom_theta-pi/2)*180/pi 
            heading=heading%360
            if heading > 180 :
                heading= heading-360

            pose = np.array([[pose_x],[pose_y],[heading]])

            # 로직 7. lidar scan 결과 수신
            # 라이다 포인트에 대한 LaserScan 메시지 안에 range(거리)를 받습니다.
            # Range는 길이 360의 어레이 형태로 되어 있고 1도만큼 돌아가면서 거리를 측정된 값입니다.
            # 이를 가지고 포인트들을 x, y 좌표계로 변환할 수 있습니다.

            Distance=np.array(msg.ranges)
            x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))
            y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))
            laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

            # 로직 8
            # . localizer 실행
            # odometry 정보와 라이다 정보로 파티클 업데이트 
            amcl_pose=self.localization.update(pose, laser)
            if amcl_pose is None :
                amcl_pose=self.prev_amcl_pose


            # 라이다를 global 좌표로 변환해서 sending
            """            
            # 로직 14. 위치 추정 결과를 publish
            self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
            self.broadcaster.sendTransform(self.laser_transform)

            amcl_q = Quaternion.from_euler(....)
            self.localizer_transform.header.stamp =
            self.localizer_transform.transform.translation.x = 
            self.localizer_transform.transform.translation.y = 
            self.localizer_transform.transform.rotation.x = 
            self.localizer_transform.transform.rotation.y = 
            self.localizer_transform.transform.rotation.z = 
            self.localizer_transform.transform.rotation.w = 
            
            self.broadcaster.sendTransform(self.localizer_transform)

            self.odom_msg.pose.pose.position.x = 
            self.odom_msg.pose.pose.position.y =
            self.odom_msg.pose.pose.orientation.x =
            self.odom_msg.pose.pose.orientation.y =
            self.odom_msg.pose.pose.orientation.z =
            self.odom_msg.pose.pose.orientation.w =
            """

            self.odom_publisher.publish(self.odom_msg)        
            self.prev_amcl_pose=amcl_pose
        
def main(args=None):
    rclpy.init(args=args)


    run_localization = Localizer()

    rclpy.spin(run_localization)

    run_localization.destroy_node()
    rclpy.shutdown()





if __name__ == '__main__':
    main()
