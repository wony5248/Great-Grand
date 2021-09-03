import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt

# a_star_local_path 노드는 a_star 노드에서 나오는 전역경로(/global_path)를 받아서, 로봇이 실제 주행하는 지역경로(/local_path)를 publish 하는 노드입니다.
# path_pub 노드와 하는 역할은 비슷하나, path_pub은 텍스트를 읽어서 global_path를 지역경로를 생성하는 반면, a_star_local_path는 global_path를 다른 노드(a_star)에서 받아서 지역경로를 생성합니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. global_path 데이터 수신 후 저장
# 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 4. global_path 중 로봇과 가장 가까운 포인트 계산
# 5. local_path 예외 처리


class astarLocalpath(Node):

    def __init__(self):
        super().__init__('a_star_local_path')
        # 로직 1. publisher, subscriber 만들기
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Path,'/global_path',self.path_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.odom_msg=Odometry()
        self.is_odom=False
        self.is_path=False
       
        self.global_path_msg=Path()


        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=30 
        self.count=0


    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def path_callback(self,msg):
        pass
        '''
        로직 2. global_path 데이터 수신 후 저장

        self.is_path=
        self.global_path_msg=
        
        '''

        
    def timer_callback(self):
        if self.is_odom and self.is_path ==True:
            
            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            current_waypoint=-1
            
            '''
            로직 4. global_path 중 로봇과 가장 가까운 포인트 계산
            
            min_dis=
            for i,waypoint in enumerate(self.global_path_msg.poses) : 
                distance=
                if distance < min_dis :
                    min_dis=
                    current_waypoint=

            '''           
            
            
            '''
            로직 5. local_path 예외 처리

            if current_waypoint != -1 : 
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    
                    
                
                else :

                    
                              
            '''           

            self.local_path_pub.publish(local_path_msg)
        

        
def main(args=None):
    rclpy.init(args=args)

    a_star_local = astarLocalpath()

    rclpy.spin(a_star_local)

    a_star_local.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()