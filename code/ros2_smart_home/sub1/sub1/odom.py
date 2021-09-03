import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin
import tf2_ros
import geometry_msgs.msg
import time

# odom 노드는 로봇의 속도(/turtlebot_status), Imu센서(/imu) 메시지를 받아서 로봇의 위치를 추정하는 노드입니다.
# sub1_odom은 imu로 부터 받은 Quaternion을 사용하거나 각속도, 가속도 데이터를 이용해서 로봇의 포즈를 추정 할 것입니다.

# 노드 로직 순서
# 1. publisher, subscriber, broadcaster 만들기
# 2. publish, broadcast 할 메시지 설정
# 3. imu 에서 받은 quaternion을 euler angle로 변환해서 사용
# 4. 로봇 위치 추정
# 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast


class odom(Node):

    def __init__(self):
        super().__init__('odom')
        
        # 로직 1. publisher, subscriber, broadcaster 만들기
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)


        # 로봇의 pose를 저장해 publish 할 메시지 변수 입니다.
        self.odom_msg=Odometry()
        # Map -> base_link 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.base_link_transform=geometry_msgs.msg.TransformStamped()
        # base_link -> laser 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.laser_transform=geometry_msgs.msg.TransformStamped()
        self.is_status=False
        self.is_imu=False
        self.is_calc_theta=False
        # x,y,theta는 추정한 로봇의 위치를 저장할 변수 입니다.
        self.x=0.0
        self.y=0.0
                
        self.theta=0.0
        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset=0
        self.prev_time=0

        '''
        로직 2. publish, broadcast 할 메시지 설정

        self.odom_msg.header.frame_id=
        self.odom_msg.child_frame_id=

        self.base_link_transform.header.frame_id = 
        self.base_link_transform.child_frame_id = 

        self.laser_transform.header.frame_id = 
        self.laser_transform.child_frame_id =      
        self.laser_transform.transform.translation.x = 
        self.laser_transform.transform.translation.y = 
        self.laser_transform.transform.translation.z = 
        self.laser_transform.transform.rotation.w = 

        '''
                      

    def imu_callback(self,msg):
        pass
        '''
        로직 3. IMU 에서 받은 quaternion을 euler angle로 변환해서 사용

        if self.is_imu ==False :    
            self.is_imu=True
            imu_q= 
            self.imu_offset=

        else :
            imu_q= 
            self.theta=

        '''


    def listener_callback(self, msg):
        print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))
        if self.is_imu ==True:
            if self.is_status == False :
                self.is_status=True
                self.prev_time=rclpy.clock.Clock().now()
            else :
                
                self.current_time=rclpy.clock.Clock().now()
                # 계산 주기를 저장한 변수 입니다. 단위는 초(s)
                self.period=(self.current_time-self.prev_time).nanoseconds/1000000000
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                linear_x=msg.twist.linear.x
                angular_z=-msg.twist.angular.z
                '''
                로직 4. 로봇 위치 추정
                (테스트) linear_x = 1, self.theta = 1.5707(rad), self.period = 1 일 때
                self.x=0, self.y=1 이 나와야 합니다. 로봇의 헤딩이 90도 돌아가 있는
                상태에서 선속도를 가진다는 것은 x축방향이 아니라 y축방향으로 이동한다는 뜻입니다. 

                self.x+=
                self.y+=
                self.theta+=

                '''

                self.base_link_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                
                '''
                로직 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast

                q =
                
                self.base_link_transform.transform.translation.x = 
                self.base_link_transform.transform.translation.y = 
                self.base_link_transform.transform.rotation.x = 
                self.base_link_transform.transform.rotation.y = 
                self.base_link_transform.transform.rotation.z = 
                self.base_link_transform.transform.rotation.w = 
                
                self.odom_msg.pose.pose.position.x=
                self.odom_msg.pose.pose.position.y=
                self.odom_msg.pose.pose.orientation.x=
                self.odom_msg.pose.pose.orientation.y=
                self.odom_msg.pose.pose.orientation.z=
                self.odom_msg.pose.pose.orientation.w=
                self.odom_msg.twist.twist.linear.x=
                self.odom_msg.twist.twist.angular.z=

                '''

                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)
                self.prev_time=self.current_time

        
def main(args=None):
    rclpy.init(args=args)

    sub1_odom = odom()

    rclpy.spin(sub1_odom)


    sub1_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()