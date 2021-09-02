
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
    T_i = utils.xyh2mat2D(pose_i)
    T_j = utils.xyh2mat2D(pose_j)
    R_i = np.transpose(T_i[:2, :2])
    t_i = -R_i.dot(T_i[:2, 2])

    R_ij = np.matmul(R_i, T_j[:2, :2])
    t_ij = R_i.dot(T_j[:2, 2]) + t_i

    pose_ij = np.array([0.0, 0.0, 0.0])
    pose_ij[:2] = t_ij[:]
    pose_ij[2] = np.arctan2(R_ij[1,0], R_ij[0,0])*180.0/np.pi

    return pose_ij


class Localization:
    """
    Localization Class
    """
    def __init__(self, params_map, params_mcl):
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

    def show_particle(self, particles, laser=None, gt_pose=None):
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

    # def show_pose_and_points(self, pose, laser_global):
    #     tmp_map = self.map.astype(np.float32)
    #     map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

    #     pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
    #     pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

    #     laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
    #     laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

    #     for i in range(laser_global.shape[1]):
    #         (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
    #         center = (l_x, l_y)
    #         cv2.circle(map_bgr, center, 1, (0,255,0), -1)

    #     center = (pose_x.astype(np.int), pose_y.astype(np.int))
    #     cv2.circle(map_bgr, center, 2, (0,0,255), -1)

    #     map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=3.0, fy=3.0)
    #     cv2.imshow('Sample Map', map_bgr)
    #     cv2.waitKey(1)

    def initialize_particles(self, pose=None, factor=None):
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

    def _prediction(self, diff_pose):
        delta_dist = np.sqrt(np.square(diff_pose[:2]).sum())

        delta_angle1 = np.arctan2(diff_pose[1], diff_pose[0])
        delta_angle2 = diff_pose[2]*np.pi/180.0 - delta_angle1
        
        odom_cov = [self.odom_translation_cov, self.odom_translation_cov, self.odom_heading_cov]

        delta_angle1 = utils.limit_angular_range(delta_angle1)

        dist_noise_coeff = odom_cov[0]*np.fabs(delta_dist) + odom_cov[2]*np.fabs(delta_angle1+ delta_angle2)
        angle1_noise_coeff = odom_cov[2]*np.fabs(delta_angle1) + odom_cov[0]*np.fabs(delta_dist)
        angle2_noise_coeff = odom_cov[2]*np.fabs(delta_angle2) + odom_cov[0]*np.fabs(delta_dist)

        score_sum = 0

        for i in range(self.num_particle):

            delta_dist = delta_dist + np.random.normal(0,1)*dist_noise_coeff
            delta_angle1 = delta_angle1 + np.random.normal(0,1)*angle1_noise_coeff
            delta_angle2 = delta_angle2 + np.random.normal(0,1)*angle2_noise_coeff

            x = delta_dist * np.cos(delta_angle1) + np.random.normal(0,1)*odom_cov[0]
            y = delta_dist * np.sin(delta_angle1) + np.random.normal(0,1)*odom_cov[1]
            h = delta_angle1 + delta_angle2 + np.random.normal(0,1)*odom_cov[2]

            diff_odom_noise = np.array([x, y, h*180.0/np.pi])
            diff_odom_T = utils.xyh2mat2D(diff_odom_noise)

            xyh_i = self.particles[:3, i]
            T_i = utils.xyh2mat2D(xyh_i)
            T_j = np.matmul(T_i, diff_odom_T)

            score_sum += self.particles[3, i]
            self.particles[:3, i] = utils.mat2D2xyh(T_j)

        self.particles[3, :] = self.particles[3, :] / score_sum

    def _points_in_map(self, x_list, y_list):
        points_global_x = (x_list - self.map_center[0] + (
                    self.map_size[0] * self.map_resolution) / 2) / self.map_resolution
        points_global_y = (y_list - self.map_center[1] + (
                    self.map_size[1] * self.map_resolution) / 2) / self.map_resolution

        return points_global_x, points_global_y

    def _weighting(self, points):
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

            # if particle is in the map
            is_pose_valid = (particle_pose_x>0) * (particle_pose_y>0) * (particle_pose_x<cols) * (particle_pose_y<rows)
            if not is_pose_valid:
                self.particles[3, i] = 0
                continue

            # if particle is in the occupied region
            occupancy = self.map[particle_pose_y.astype(np.int), particle_pose_x.astype(np.int)]
            if occupancy < 0.75:
                self.particles[3, i] = 0
                continue

            transformed_points = np.matmul(T_h_bar, point_mat)
            points_global_x, points_global_y = self._points_in_map(transformed_points[0, :], transformed_points[1, :])

            valid_idx = (points_global_x>0) * (points_global_y>0) * (points_global_x<cols) * (points_global_y<rows)
            valid_x = points_global_x[valid_idx]
            valid_y = points_global_y[valid_idx]
            vals = self.map[valid_y.astype(np.int), valid_x.astype(np.int)]
            walls = vals < 0.25
            empty = (vals > 0.25)
            weight = np.sum(walls)
            penalty = np.sum(empty)*0.0
            weight = weight - penalty
            weights[i] = weight

            self.particles[3, i] += weight / transformed_points.shape[1]
            score_sum += self.particles[3, i]

            if max_score < self.particles[3, i]:
                self.max_prob_particle = self.particles[:,i]
                max_score = self.particles[3, i]
        self.stable_score= np.max(weights)
        print('max matching points', np.max(weights))

        for i in range(self.num_particle):
            self.particles[3,i] = self.particles[3,i]/(score_sum)


    def _resampling(self):
        n_particles = self.num_particle
        score_basline = 0.0
        particle_scores = np.zeros([n_particles])
        selected_particles = np.zeros(self.particles.shape)
        
        for i in range(n_particles):
            score_basline += self.particles[3, i]
            particle_scores[i] = score_basline

        # print('highest particle score', np.max(self.particles[3,:]))

        # print(score_basline)
    
        for i in range(n_particles):
            darted_score = np.random.uniform(0, score_basline, 1)
            darted_idx = np.abs(particle_scores-darted_score).argmin().astype(np.int)
            
            selected_particles[:, i] = self.particles[:, darted_idx]

        self.particles = selected_particles.copy()

    def update(self, pose, laser):

        if self.is_init != True:
            self.odom_before = pose
            self.initialize_particles(pose, 10.0)
            # self.initialize_particles()
            self.is_init = True
            return

        diff_pose = compute_relative_pose(self.odom_before, pose)

        diff_dist = np.sqrt(np.square(diff_pose[:2]).sum())
        diff_angle = diff_pose[2]

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

        if self.is_init_pose ==True and self.is_imu==True:
            if self.is_status == False :
                
                self.prev_time=rclpy.clock.Clock().now()

                self.is_status=True
            else :
                self.current_time=rclpy.clock.Clock().now()
                self.period=(self.current_time-self.prev_time).nanoseconds/(1e+9)


                linear_x=msg.twist.linear.x
    
    

                self.odom_x+=linear_x*cos(self.odom_theta) * self.period
                self.odom_y+=linear_x*sin(self.odom_theta) * self.period
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


            Distance=np.array(msg.ranges)
            x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))
            y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))
            laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

            # odometry 정보와 라이다 정보로 파티클 업데이트 
            amcl_pose=self.localization.update(pose, laser)
            if amcl_pose is None :
                amcl_pose=self.prev_amcl_pose


            # 라이다를 global 좌표로 변환해서 sending
            self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
            self.broadcaster.sendTransform(self.laser_transform)
        
        
            amcl_q = Quaternion.from_euler(0, 0, (amcl_pose[2]+90)/180*pi)
            self.localizer_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
            self.localizer_transform.transform.translation.x = float(amcl_pose[0])
            self.localizer_transform.transform.translation.y = float(amcl_pose[1])
            self.localizer_transform.transform.rotation.x = amcl_q.x
            self.localizer_transform.transform.rotation.y = amcl_q.y
            self.localizer_transform.transform.rotation.z = amcl_q.z
            self.localizer_transform.transform.rotation.w = amcl_q.w
            self.broadcaster.sendTransform(self.localizer_transform)

            # #odom
            self.odom_msg.pose.pose.position.x=float(amcl_pose[0])
            self.odom_msg.pose.pose.position.y=float(amcl_pose[1])
            self.odom_msg.pose.pose.orientation.x=amcl_q.x
            self.odom_msg.pose.pose.orientation.y=amcl_q.y
            self.odom_msg.pose.pose.orientation.z=amcl_q.z
            self.odom_msg.pose.pose.orientation.w=amcl_q.w
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
