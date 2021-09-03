import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,HandControl,CustomObjectInfo
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pow,sqrt,atan2,pi

import time, random 

# 노드 로직 순서
# 1. publisher, subscriber, msg 생성
# 2. turtlebot의 초기 위치 정보로 맵 영역 설정.
# 3. turtlebot의 목적지(goal_x, goal_y) 설정
# 4. turtlebot의 선속도, 각속도 정하기
# 5. turtlebot이 목적지에 도착했을때 handcontrol 제어

'''
map 좌표 정보 
◎ : 터틀봇
↑ : 초기 위치에서 터틀봇이 바라보고 있는 방향
    (-25,-75)      (-50,-75)       (-75, -75)
          ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
          ┃                           ┃
          ┃        team1_area         ┃
          ┃                           ┃
          ┃          　 ↑             ┃
(-25,-50) ┃. . . . . . ◎ . . . . . . ┃ (-75, -50)
          ┃                           ┃
          ┃                           ┃
          ┃        team2_area         ┃
          ┃                           ┃
          ┃                           ┃
          ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    (-25,-75)       (-50,-25)     (-75, -25)
'''

'''
시작하면 서로의 진영에서 시작.

custom object의 정보를 받는데 자기 진영에 있는 custom obejct 정보를 받아서 가장 가까이 있는 custom object의 x,y 를  goal_x, goal_y로 잡고 주행.
해당위치에 거의 도달했을 때, can_lift가 True 이면, hand_control_pick_up 기능을 이용해서 custom object를 집는다.

그다음 상대방 영역중 랜덤하게 좌표를 생성해서 goal_x, goal_y로 잡고 주행, 해당 좌표랑 turtlebot의 distance가 1미만이 되면, handcontrol_preview, handcontrol_put 기능을 순차적으로실행해서
해당 위치에 custom object를 위치 시키고, 나의 영역에 있는 새로운 custom object를 새로운 goal_x, goal_y로 잡고 해당 시퀀스 반복
'''

    
class game(Node):

    def __init__(self):        
        super().__init__('game_test')
        # 로직 1 : publisher, subscriber, msg 생성
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)                
        
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.obj_sub = self.create_subscription(CustomObjectInfo,'/custom_object_info',self.object_callback,10)

        self.cmd_msg=Twist()
        self.hand_control_msg=HandControl()
        
        self.is_status=False
        self.is_obj=False
        self.is_set_area = False
      
        self.set_on_enemy_area = True

        self.object_msg = CustomObjectInfo()
        time_period=0.1
        self.timer = self.create_timer(time_period, self.timer_callback)   

    def timer_callback(self):        
        if not self.is_set_area : self.set_area() #영역 설정

        if self.is_status and self.is_obj :            
            
            self.set_goal() #목적지 설정
            
            self.set_cmd_vel() #선속도 각속도 정하기

            self.use_hand_control() #handcontrol 사용             

    # 로직 2 : turtlebot의 초기 위치 정보로 맵 영역 설정.
    def set_area(self):
        if self.status_msg:            
            x = self.status_msg.twist.angular.x
            y = self.status_msg.twist.angular.y
            if (x < -25 and x >-75) and (y <-50 and y > -75):
               
                self.friendly_map = {    
                    "MAP_POINT_x1" : -25, 
                    "MAP_POINT_y1" : -75,
                    "MAP_POINT_x2" : -75,
                    "MAP_POINT_y2" : -50    
                }

                self.enemy_map = {    
                    "MAP_POINT_x1" : -25,
                    "MAP_POINT_y1" : -50,
                    "MAP_POINT_x2" : -75,
                    "MAP_POINT_y2" : -25    
                }
                print("team1")
            else:                
                self.enemy_map = {    
                    "MAP_POINT_x1" : -25, 
                    "MAP_POINT_y1" : -75,
                    "MAP_POINT_x2" : -75,
                    "MAP_POINT_y2" : -50    
                }
                self.friendly_map = {    
                    "MAP_POINT_x1" : -25,
                    "MAP_POINT_y1" : -50,
                    "MAP_POINT_x2" : -75,
                    "MAP_POINT_y2" : -25    
                }                                

            self.is_set_area = True
    
    # 로직 3 : turtlebot의 목적지(goal_x, goal_y) 설정
    def set_goal(self):
        #custom object를 들고 있을 때 상대방 진영중 무작위 위치로 목적지 변경        
        if self.status_msg.can_use_hand and self.set_on_enemy_area:     
                self.set_goal_enemy_area()            
                
        #custom object를 들고 있지 않을 때 아군진영의 custom object위치로 목적지 변경
        if not self.status_msg.can_use_hand :
            self.find_nearly_custom_object()

    # 로직 4 : turtlebot의 선속도 각속도 정하기
    def set_cmd_vel(self):
        #atan2를 이용하여 두 점간의 각도 구하기
        #http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
        dx = float(self.goal_x) - self.status_msg.twist.angular.x
        dy = float(self.goal_y) - self.status_msg.twist.angular.y
        K_anular = 0.5
        disired_angle_goal = atan2(dy,dx)
        angular_speed = (self.yaw - disired_angle_goal) * K_anular
        
        self.cmd_msg.angular.z = angular_speed                                    
        if abs(angular_speed) < 0.05:               
            self.cmd_msg.linear.x = 1.0

        dist = sqrt(pow(dx,2)+pow(dy,2))
        #goal_x, goal_y에 근접했을 때 turtlebot 정지
        if dist < 1:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0

        self.cmd_pub.publish(self.cmd_msg)      
        
    # 로직 5 : 목적지에 도착했을때 handcontrol 제어
    def use_hand_control(self):
        dx = float(self.goal_x) - self.status_msg.twist.angular.x
        dy = float(self.goal_y) - self.status_msg.twist.angular.y

        dist = sqrt(pow(dx,2)+pow(dy,2))  
        
        if dist < 1 :                
            #goal_x, goal_y에 근접했을 때 handcontrol 작동(pick_up, put_down)
            if self.status_msg.can_use_hand:             
                self.hand_control_put_down()
            else:
                self.hand_control_pick_up()

    #아군영역에 있는 custom object로 goal_x, goal_y 설정
    def find_nearly_custom_object(self):
        friendly_map_x1 = self.friendly_map["MAP_POINT_x1"]
        friendly_map_y1 = self.friendly_map["MAP_POINT_y1"]
        friendly_map_x2 = self.friendly_map["MAP_POINT_x2"]
        friendly_map_y2 = self.friendly_map["MAP_POINT_y2"]
        
        x1 = min(friendly_map_x1,friendly_map_x2)
        x2 = max(friendly_map_x1,friendly_map_x2)
        y1 = min(friendly_map_y1,friendly_map_y2)
        y2 = max(friendly_map_y1,friendly_map_y2)

        min_dist = float('inf')
        nearly_object_index = -1                

        #모든 object 데이터를 받아서 아군영역에있는 object중 가장 가까운 위치에 있는 object 탐색.
        for index, obj_pose in enumerate(self.object_msg.position):           
            if (obj_pose.x >= x1 and obj_pose.x <= x2) and (obj_pose.y >= y1 and obj_pose.y <= y2):
                dx = self.status_msg.twist.angular.x - obj_pose.x 
                dy = self.status_msg.twist.angular.y - obj_pose.y                
                dist = sqrt(pow(dx,2)+pow(dy,2))

                if dist < min_dist :
                    min_dist = dist
                    nearly_object_index = index
            
        if not nearly_object_index == -1 :
            self.goal_x = self.object_msg.position[nearly_object_index].x
            self.goal_y = self.object_msg.position[nearly_object_index].y            

        else:#아군 진영에 object가 남아있지 않을 때 초기 위치로                       
            self.goal_x = -50
            self.goal_y = -50
        
    #적 영역 중 랜덤 좌표 생성
    def set_goal_enemy_area(self):
        enemy_area_x1 = self.enemy_map["MAP_POINT_x1"]
        enemy_area_y1 = self.enemy_map["MAP_POINT_y1"]
        enemy_area_x2 = self.enemy_map["MAP_POINT_x2"]
        enemy_area_y2 = self.enemy_map["MAP_POINT_y2"]
        
        self.goal_x = random.randint(min(enemy_area_x1,enemy_area_x2),max(enemy_area_x1,enemy_area_x2))
        self.goal_y = random.randint(min(enemy_area_y1,enemy_area_y2),max(enemy_area_y1,enemy_area_y2))

        self.set_on_enemy_area = False

    def hand_control_pick_up(self):

        if self.status_msg.can_lift:            
            print("pick_up")
            self.hand_control_msg.control_mode = 2
            self.hand_control.publish(self.hand_control_msg) 
            self.set_on_enemy_area = True           

    def hand_control_preview(self):

        if self.status_msg.can_use_hand :
            print("preview")
            self.hand_control_msg.control_mode = 1
            self.hand_control_msg.put_distance = 1.0
            self.hand_control_msg.put_height = 0.3            
            self.hand_control.publish(self.hand_control_msg)      

    def hand_control_put_down(self):        
        if self.status_msg.can_put:          
            print("put_down")
            self.hand_control_msg.control_mode = 3
            self.hand_control.publish(self.hand_control_msg)                                  
                
        else:            
            self.hand_control_preview()


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg
        self.yaw = self.status_msg.twist.linear.z*pi/180

    def object_callback(self,msg):
        self.is_obj=True
        self.object_msg=msg
        
def main(args=None):
    rclpy.init(args=args)
    game_test = game()
    rclpy.spin(game_test)
    game_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()