import rclpy
from rclpy.node import Node
import socketio
from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray

# controller는 시뮬레이터로 부터를 데이터를 수신해서 확인(출력)하고, 송신해서 제어가 되는지 확인해보는 통신 테스트를 위한 노드입니다.
# 메시지를 받아서 어떤 데이터들이 있는지 확인하고, 어떤 메시지를 보내야 가전 또는 터틀봇이 제어가 되는지 확인해보면서 ros2 통신에 익숙해지세요.
# 수신 데이터 : 터틀봇 상태(/turtlebot_status), 환경정보(/envir_status), 가전정보(/app_status)
# 송신 데이터 : 터틀봇 제어(/ctrl_cmd), 가전제어(/app_control)

# 노드 로직 순서
# 1. 수신 데이터 출력
# 2. 특정 가전제품 ON
# 3. 특정 가전제품 OFF
# 4. 터틀봇 정지
# 5. 터틀봇 시계방향 회전
# 6. 터틀봇 반시계방향 회전

sio = socketio.Client(engineio_logger=True, logger=True, ssl_verify=False)
@sio.event
def connect():
    print('connection established')

@sio.event
def disconnect():
    print('disconnected from server')
class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        ## 메시지 송신을 위한 PUBLISHER 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)

        ## 메시지 수신을 위한 SUBSCRIBER 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        ## 제어 메시지 변수 생성 
        self.cmd_msg=Twist()
        # sio.connect('http://127.0.0.1:12001')
        sio.connect('http://j5a103.p.ssafy.io:3002')
        self.app_control_msg=Int8MultiArray()
        for i in range(17):
            self.app_control_msg.data.append(0)


        self.turtlebot_status_msg=TurtlebotStatus()
        self.envir_status_msg=EnviromentStatus()
        self.app_status_msg=Int8MultiArray()
        self.is_turtlebot_status=False
        self.is_app_status=False
        self.is_envir_status=False
        self.msg_count=0


    def listener_callback(self, msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg

    def envir_callback(self, msg):
        self.is_envir_status=True
        self.envir_status_msg=msg

    def app_callback(self, msg):
        self.is_app_status=True
        self.app_status_msg=msg

    def app_all_on(self):
        print("on")
        for i in range(17):
            self.app_control_msg.data[i]=1
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_all_off(self):
        for i in range(17):
            self.app_control_msg.data[i]=2
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_select_on(self,num):
        '''
        로직 2. 특정 가전 제품 ON
        '''
        if 0 <= num < 17 :
            self.app_control_msg.data[num]=1
        else :
            print("Error::controller.py::app_on_select::invalid Number")
        self.app_control_pub.publish(self.app_control_msg)

    def app_select_off(self,num):
        '''
        로직 3. 특정 가전 제품 OFF
        '''
        if 0 <= num < 17 :
            self.app_control_msg.data[num]=2
        else :
            print("Error::controller.py::app_dff_select::invalid Number")
        self.app_control_pub.publish(self.app_control_msg)

    def turtlebot_go(self) :
        self.cmd_msg.linear.x=0.3
        self.cmd_msg.angular.z=0.0

    def turtlebot_stop(self) :
        '''
        로직 4. 터틀봇 정지
        '''
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.0

    def turtlebot_cw_rot(self) :
        '''
        로직 5. 터틀봇 시계방향 회전
        '''
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.3

    def turtlebot_cww_rot(self) :
        '''
        로직 6. 터틀봇 반시계방향 회전
        '''
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=-0.3


    def timer_callback(self):

        '''
        로직1. 수신 데이터 출력
        터틀봇 상태 : 현재 선솏도, 현재 각속도, 배터리 상태, 충전 상태 출력
        환경 정보 : 날짜, 시간, 온도, 날씨 출력
        가전 제품 : 가전상태 출력        
        '''
        # 약 1초에 한번 상태 데이터 출력
        self.msg_count += 1
        
        if self.msg_count > 30 :
            self.msg_count = 0
            # 터틀봇 상태 데이터가 있으면 상태 데이터 출력
            if self.is_turtlebot_status :
                print('Linear Velocity : {0}, Angular Velocity : {1}, battery percentage : {2}, power supply : {3}'.format(
                        self.turtlebot_status_msg.twist.linear.x,
                        self.turtlebot_status_msg.twist.angular.z,
                        self.turtlebot_status_msg.battery_percentage,
                        self.turtlebot_status_msg.power_supply_status
                ))

            # 환경 정보 데이터가 있으면 환경정보 데이터 출력
            if self.is_envir_status :
                lst = [self.envir_status_msg.month, self.envir_status_msg.day, self.envir_status_msg.hour, self.envir_status_msg.minute,self.envir_status_msg.temperature,self.envir_status_msg.weather]
                sio.emit('sensor', lst)
                print('{0}.{1}-{2}:{3}, temperature : {4}\'C, {5}'.format(
                    self.envir_status_msg.month,
                    self.envir_status_msg.day,
                    self.envir_status_msg.hour,
                    self.envir_status_msg.minute,
                    self.envir_status_msg.temperature,
                    self.envir_status_msg.weather
                ))

            # 가전 정보 출력
            if self.is_app_status :
                print(self.app_status_msg.data)
        
        ## IOT(가전) 제어 함수
        # self.app_all_on()
        # self.app_all_off()
        # self.app_select_on(12)
        # self.app_select_off(12)


        ## 터틀봇 제어 함수
        self.turtlebot_go()
        # self.turtlebot_stop()
        self.turtlebot_cw_rot()
        # self.turtlebot_cww_rot()

        self.cmd_publisher.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()
    sio.disconnect()


if __name__ == '__main__':
    main()