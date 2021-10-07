import socket
import sys
import threading
import time
import struct
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
from squaternion import Quaternion
import rclpy
from math import pi
from sensor_msgs.msg import BatteryState,Imu
from std_msgs.msg import String,Int8MultiArray
from ssafy_msgs.msg import EnviromentStatus,TurtlebotStatus,CustomObjectInfo
from geometry_msgs.msg import Twist,Vector3

class erp_udp_parser :
    def __init__(self,publisher,ip,port,data_type):
        self.ip=ip
        self.port=port
        self.publisher=publisher
        self.data_type=data_type
        self.is_sender_port=False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip,self.port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

    def send_data(self,app_control_data):
        if self.is_sender_port ==True :
            header='#Appliances$'.encode()
            data_length=struct.pack('i',17)
            aux_data=struct.pack('iii',0,0,0)
            self.upper=header+data_length+aux_data
            self.tail='\r\n'.encode()
            lower=struct.pack('17B',app_control_data[0],app_control_data[1],app_control_data[2],app_control_data[3],
                                    app_control_data[4],app_control_data[5],app_control_data[6],app_control_data[7],
                                    app_control_data[8],app_control_data[9],app_control_data[10],app_control_data[11],
                                    app_control_data[12],app_control_data[13],app_control_data[14],app_control_data[15],
                                    app_control_data[16])
        
            send_data=self.upper+lower+self.tail
            self.sock.sendto(send_data,(self.ip,self.sender_port))
        else :
            print('cannot sender port')
    
    

    def recv_udp_data(self):
        try :
            while True :
                raw_data, sender = self.sock.recvfrom(self.data_size)
                self.data_parsing(raw_data)
                
                if self.is_sender_port == False :
                    self.is_sender_port= True
                    self.sender_port=sender[1]
        except:
            sys.exit()


    def data_parsing(self,raw_data) :
        
        
        if self.data_type=='imu' :
            data_length=struct.unpack('i',raw_data[9:13])
            imu_data=struct.unpack('10d',raw_data[25:105])
            msg=Imu()
            msg.header.stamp=rclpy.clock.Clock().now().to_msg()
            msg.header.frame_id='imu'
            msg.orientation.w=imu_data[0]
            msg.orientation.x=imu_data[1]
            msg.orientation.y=imu_data[2]
            msg.orientation.z=imu_data[3]
            
            msg.angular_velocity.x=imu_data[4]
            msg.angular_velocity.y=imu_data[5]
            msg.angular_velocity.z=imu_data[6]
            msg.linear_acceleration.x=imu_data[7]
            msg.linear_acceleration.y=imu_data[8]
            msg.linear_acceleration.z=imu_data[9]

            self.publisher.publish(msg)
        
        if self.data_type=='app_status' :
            header=raw_data[0:12].decode()
            data_length=struct.unpack('i',raw_data[12:16])
            if header == '#Appliances$' and data_length[0] ==17:
                app_status_data=struct.unpack('17B',raw_data[16+12:33+12])
                app_data=list(app_status_data)
                msg=Int8MultiArray()
                msg.data=app_data
                self.publisher.publish(msg)

            
        if self.data_type=='envir_status':
            header=raw_data[0:12].decode()
            data_length=struct.unpack('i',raw_data[12:16])
            
            if header == '#Enviroment$' and data_length[0] ==6:
                envir_status_data=struct.unpack('6B',raw_data[16+12:22+12])
                msg=EnviromentStatus()
                
                if envir_status_data[0] == 0 :
                    weather='Sunny'
                elif envir_status_data[0] == 1 :
                    weather='Cloudy'
                elif envir_status_data[0] == 2 :
                    weather='Foggy'
                elif envir_status_data[0] == 3 :
                    weather='Stormy'
                elif envir_status_data[0] == 4 :
                    weather='Rainy'
                elif envir_status_data[0] == 5 :
                    weather='Snowy'
                msg.weather=weather    
                msg.temperature=envir_status_data[1]
                msg.month=envir_status_data[2]
                msg.day=envir_status_data[3]
                msg.hour=envir_status_data[4]
                msg.minute=envir_status_data[5]
                self.publisher.publish(msg)
        
        if self.data_type=='turtlebot_status':
            # print(raw_data)
            header=raw_data[0:11].decode()
            data_length=struct.unpack('i',raw_data[11:15])
            if header == '#Turtlebot$' and data_length[0] ==32:
                msg=TurtlebotStatus()
                ego_status_data=struct.unpack('2f',raw_data[15+12:15+8+12])
                battery_charge_status=struct.unpack('B',raw_data[15+8+12:15+8+12+1])[0]
                battery_percentage=struct.unpack('f',raw_data[15+8+12+1:15+8+12+1+4])[0]
                msg.twist.linear.x=ego_status_data[0]
                msg.twist.angular.z=ego_status_data[1]
                msg.power_supply_status=battery_charge_status
                msg.battery_percentage=battery_percentage

                x,y,z,heading = struct.unpack('4f',raw_data[15+8+12+1+4:15+8+12+1+4+16])
                msg.twist.angular.x = x
                msg.twist.angular.y = y
                msg.twist.linear.z = heading

                hand_status=struct.unpack('???',raw_data[56:59])
                msg.can_use_hand=hand_status[0]
                msg.can_put=hand_status[1]
                msg.can_lift=hand_status[2]
                self.publisher.publish(msg)


        if self.data_type=='custom_object':
            header=raw_data[0:18].decode()
            data_length=struct.unpack('i',raw_data[18:22])

            if header == '#hand_control_pub$' and data_length[0] ==12*20:
                msg=CustomObjectInfo()                   
                offset = 34
                for i in range(20):
                    start_byte = i*12
                    data = struct.unpack('3f',raw_data[offset+start_byte:offset+start_byte+12])
                    temp = Vector3()    
                    temp.x = data[0]
                    temp.y = data[1]
                    temp.z = data[2]
                    if not data[0] == 0 and not data[1] == 0 and not data[2] == 0 :                                                             
                        msg.position.append(temp)
                self.publisher.publish(msg)

    def __del__(self):
        self.sock.close()
        sys.exit()
        print('del')


        
        
class erp_udp_sender :
    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip=ip
        self.port=port

        header='#Turtlebot_cmd$'.encode()
        data_length=struct.pack('i',8)
        aux_data=struct.pack('iii',0,0,0)
        self.upper=header+data_length+aux_data
        self.tail='\r\n'.encode()
        
        

    def send_data(self,linear_vel,angular_vel):
        
        lower=struct.pack('ff',linear_vel,angular_vel)
    
        send_data=self.upper+lower+self.tail
        
        self.sock.sendto(send_data,(self.ip,self.port))
    


      

class handControlSender :
    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip=ip
        self.port=port

        header='#hand_control$'.encode()
        data_length=struct.pack('i',9)
        aux_data=struct.pack('iii',0,0,0)
        self.upper=header+data_length+aux_data
        self.tail='\r\n'.encode()
    
        
        

    def send_data(self,mode,dis,height):
        
        mode_pac=struct.pack('B',mode)
        dis_pac=struct.pack('f',dis)
        height_pac=struct.pack('f',height)
        lower=mode_pac+dis_pac+height_pac

        send_data=self.upper+lower+self.tail
        # print(send_data,len(send_data))
        
        self.sock.sendto(send_data,(self.ip,self.port))
    
class app_control_sender :
    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip=ip
        self.port=port

        header='#Appliances$'.encode()
        data_length=struct.pack('i',17)
        aux_data=struct.pack('iii',0,0,0)
        self.upper=header+data_length+aux_data
        self.tail='\r\n'.encode()
        
        

    def send_data(self,app_control_data):
        
        lower=struct.pack('17B',app_control_data[0],app_control_data[1],app_control_data[2],app_control_data[3],
                        app_control_data[4],app_control_data[5],app_control_data[6],app_control_data[7],
                        app_control_data[8],app_control_data[9],app_control_data[10],app_control_data[11],
                        app_control_data[12],app_control_data[13],app_control_data[14],app_control_data[15],
                        app_control_data[16])
    
        send_data=self.upper+lower+self.tail
        
        self.sock.sendto(send_data,(self.ip,self.port))





        
