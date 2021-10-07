import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from time import sleep
import socketio

sio = socketio.Client()
is_patrol = False

@sio.event
def connect():
    print('connection established')

@sio.event
def patrol(data):
    global is_patrol
    print(data)
    if data == 1 :
        is_patrol = True
    else :
        is_patrol = False

@sio.event
def disconnect():
    print('disconnected from server')


class PatrolPublisher(Node):

    def __init__(self):
        super().__init__('patrol')

        global is_patrol

        self.next_point_publisher = self.create_publisher(PoseStamped,'next_pose', 10)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        time_period=0.01
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.pose_msg=PoseStamped()
        self.pose_msg.header.frame_id='map'
        self.x = 0.0
        self.y = 0.0

        self.patrol_point_x = [-4.8164, -7.8902, -10.892, -5.8101, -3.606, -4.955]
        self.patrol_point_y = [6.5351, 5.0274, 4.992, 9.3423, 12.628, 15.364]
        self.next = 0

        self.is_publish=False
        self.is_odom=False
        self.prev_mode = is_patrol

        # sio.connect('http://127.0.0.1:12001')
        sio.connect('http://j5a103.p.ssafy.io:3001')


    def pose_to_grid_cell(self,x,y):   
        map_point_x= int(x*20) + 305
        map_point_y= int(y*20) - 15
        
        return map_point_x,map_point_y


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def timer_callback(self):
        global is_patrol

        if is_patrol == True and self.is_odom == True :
            if is_patrol != self.prev_mode :
                self.is_publish = False

            if self.is_publish == False :
                # 다음 목표가 퍼블리시 되지 않았을 경우
                # 다음 목표 지점을 퍼블리시한다.
                self.pose_msg.pose.position.x = float(self.patrol_point_x[self.next])
                self.pose_msg.pose.position.y = float(self.patrol_point_y[self.next])

                self.is_publish = True
                sleep(3)
                print("Publish Point {}".format(self.next))
                self.next_point_publisher.publish(self.pose_msg)
            else :
                # 다음 목표 지점과 현재 있는 지점이 동일할 경우
                # 목표 지점을 다른 지점으로 변경한다.
                self.x=self.odom_msg.pose.pose.position.x
                self.y=self.odom_msg.pose.pose.position.y

                current_cell = self.pose_to_grid_cell(self.x, self.y)
                target_cell = self.pose_to_grid_cell(self.patrol_point_x[self.next], self.patrol_point_y[self.next])
                if abs(current_cell[0] - target_cell[0]) <= 5 and abs(current_cell[1] - target_cell[1]) <= 5 :
                    self.next += 1
                    if self.next == len(self.patrol_point_x) :
                        self.next = 0
                    self.is_publish = False
        
        self.prev_mode = is_patrol


def main(args=None):
    rclpy.init(args=args)
    patrol_publisher = PatrolPublisher()
    rclpy.spin(patrol_publisher)
    patrol_publisher.destroy_node()
    rclpy.shutdown()
    sio.disconnect()


if __name__ == '__main__':
    main()
