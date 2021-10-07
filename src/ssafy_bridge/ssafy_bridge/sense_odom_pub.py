import rclpy
from rclpy.node import Node

from squaternion import Quaternion
from nav_msgs.msg import Odometry
from ssafy_msgs.msg import TurtlebotStatus
import tf2_ros
import geometry_msgs.msg
from math import pi, cos, sin

class odom(Node):

    def __init__(self):
        super().__init__('odom')

        self.subscription = self.create_subscription(
            TurtlebotStatus, #타입 이름
            'turtlebot_status', #토픽 이름
            self.listener_callback, #메시지들어올때 호출될 함수
            10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.odom_msg = Odometry()
        self.base_link_transform = geometry_msgs.msg.TransformStamped() #broadcast할 좌표계 메시지 생성
        self.laser_transform = geometry_msgs.msg.TransformStamped() #broadcast할 좌표계 메시지 생성
        self.is_status = False
        self.is_calc_theta = False

        self.x = 0
        self.y = 0
        self.theta = 0
        self.prev_time = 0

        self.odom_msg.header.frame_id = 'map' #odometry 메시지에 좌표계 이름을 정해줌
        self.odom_msg.child_frame_id = 'base_link'

        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform.header.frame_id = 'base_link'
        self.laser_transform.child_frame_id = 'laser'
        self.laser_transform.transform.translation.x = 0.0 #바뀌지 않는 값임
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 1.0
        self.laser_transform.transform.rotation.w = 1.0
        #base_link 로부터 laser 좌표계는 회전되어 있지 않다 그렇기때문에 init에 쓰는 것
        

    def listener_callback(self, msg):
        if self.is_status == False:
            self.is_status = True
            self.prev_time = rclpy.clock.Clock().now() #적분 구간을 정해주기 위해 시간을 저장한다.

        else:
            self.current_time = rclpy.clock.Clock().now()
            self.period = (self.current_time - self.prev_time).nanoseconds/1000000000

            linear_x = msg.twist.linear.x
            angular_z = -msg.twist.angular.z

            self.x += linear_x*cos(self.theta)*self.period #적분을 코드로 표현 => 누적으로 표현한다.
            self.y += linear_x*sin(self.theta)*self.period
            self.theta += angular_z*self.period #각속도 누적 => 향하고 있는 방향 표현

            q = Quaternion.from_euler(0,0,self.theta) #오일러에서 쿼터니언으로 변환

            self.base_link_transform.header.stamp = rclpy.clock.Clock().now().to_msg() #broadcast용 시간
            self.laser_transform.header.stamp = rclpy.clock.Clock().now().to_msg() #broadcast용 시간
            self.base_link_transform.transform.translation.x = self.x #계산한 x, y가 translation(이동)값이 됨
            self.base_link_transform.transform.translation.y = self.y
            self.laser_transform.transform.rotation.x = q.x #계산한 q가 roatation(회전)값이 됨
            self.laser_transform.transform.rotation.y = q.y
            self.laser_transform.transform.rotation.z = q.z
            self.laser_transform.transform.rotation.w = q.w

            self.odom_msg.pose.pose.position.x = self.x #odometry 메시지도 이동, 회전, 제어 값을 채움
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.orientation.x = q.x
            self.odom_msg.pose.pose.orientation.y = q.y
            self.odom_msg.pose.pose.orientation.z = q.z
            self.odom_msg.pose.pose.orientation.w = q.w
            self.odom_msg.twist.twist.linear.x = linear_x
            self.odom_msg.twist.twist.angular.x = angular_z

            self.broadcaster.sendTransform(self.base_link_transform) #좌표계 broadcast
            self.broadcaster.sendTransform(self.laser_transform)

            self.odom_publisher.publish(self.odom_msg) #odometry 메시지 publish
            self.prev_time=self.current_time







def main(args=None):
    rclpy.init(args=args)

    odom_node = odom()

    rclpy.spin(odom_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()