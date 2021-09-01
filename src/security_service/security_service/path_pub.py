# import rclpy
# from rclpy.node import Node
# import ros2pkg
# from geometry_msgs.msg import Twist,PoseStamped
# from ssafy_msgs.msg import TurtlebotStatus
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32
# from squaternion import Quaternion
# from nav_msgs.msg import Odometry,Path

# from math import pi,cos,sin,sqrt
# import tf2_ros
# import os


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('path_pub')
#         self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
#         self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
#         self.is_odom=False

#         pkg_path =os.getcwd()
#         back_folder='..'
#         # back_folder='src\\security_service'
#         folder_name='path'
#         file_name='patrol_ref.txt'
#         full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)

#         self.f=open(full_path,'r')

#         self.global_path_msg=Path()
#         self.global_path_msg.header.frame_id='odom'


#         lines=self.f.readlines()
#         for line in lines :
#             tmp=line.split()
#             read_pose=PoseStamped()
#             read_pose.pose.position.x=float(tmp[0])
#             read_pose.pose.position.y=float(tmp[1])
#             read_pose.pose.orientation.w=1.0
#             self.global_path_msg.poses.append(read_pose)
        
#         self.f.close()


#         time_period=0.02 # 0.1sec 
#         self.timer = self.create_timer(time_period, self.timer_callback)
#         self.local_path_size=30 #3m
#         self.count=0

#     def listener_callback(self,msg):
        
#         self.is_odom=True
#         self.odom_msg=msg

#     def timer_callback(self):
        
#         self.global_path_pub.publish(self.global_path_msg)
        

        
# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt
import tf2_ros
import os
# path_pub 노드는 make_path 노드에서 만든 텍스트 파일을 읽어와 전역 경로(/global_path)로 사용하고, 
# 전역 경로 중 로봇과 가장 가까운 포인트를 시작점으로 실제 로봇이 경로 추종에 사용하는 경로인 지역 경로(/local_path)를 만들어주는 노드입니다.
# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
# 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
# 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 5. global_path 중 로봇과 가장 가까운 포인트 계산
# 6. local_path 예외 처리
# 7. global_path 업데이트 주기 재설정
class pathPub(Node):
    def __init__(self):
        super().__init__('path_pub')
        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.lidar_sub= self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.odom_msg=Odometry()
        self.is_odom=False
        #전역경로 메시지
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='map'
        '''
        로직 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
        '''
        pkg_path =os.getcwd()
        back_folder='..'
        folder_name='path'
        file_name='test.txt'
        full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
        self.f=open(full_path,'r')
        '''
        로직 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
        '''
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1.0
            self.global_path_msg.poses.append(read_pose)
        self.f.close()
        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.02 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=20 
        self.count=0
    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg
    def timer_callback(self):
        if self.is_odom ==True:
            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            print(x,y)
            current_waypoint=-1
            '''
            로직 5. global_path 중 로봇과 가장 가까운 포인트 계산
            '''
            min_dis=float('inf')
            current_waypoint=-1
            for i,waypoint in enumerate(self.global_path_msg.poses) :
                distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                if distance < min_dis :
                    min_dis=distance
                    current_waypoint=i
            '''
            로직 6. local_path 예외 처리
            '''          
            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1.0
                        local_path_msg.poses.append(tmp_pose)
                else :
                    for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1.0
                        local_path_msg.poses.append(tmp_pose)
            self.local_path_pub.publish(local_path_msg)
        # 로직 7. global_path 업데이트 주기 재설정
        if self.count%10==0 :
            self.global_path_pub.publish(self.global_path_msg)
        self.count+=1       
def main(args=None):
    rclpy.init(args=args)
    path_pub = pathPub()
    rclpy.spin(path_pub)
    path_pub.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()