import socket
import sys
import threading
import time
import struct
import os
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray


class UDP_CAM_Parser:

    def __init__(self, publisher, ip, port, params_cam=None):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)
        self.publisher = publisher 
        self.img_msg = CompressedImage()

        self.data_size=int(65535)
        
        self.max_len = int(0)
        
        # the steps while checking how many blocks need to complete jpg
        self.ready_step = int(10)

        thread2 = threading.Thread(target=self.check_max_len)
        thread2.daemon = True 
        thread2.start() 

        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

    def check_max_len(self):

        idx_list = b''
        for i in range(self.ready_step):
            print(i)

            UnitBlock, sender = self.sock.recvfrom(self.data_size)
            
            idx_list+=UnitBlock[3:7]

        self.max_len = np.max(np.fromstring(idx_list, dtype = "int"))+1

    def recv_udp_data(self):

        TotalBuffer = b''
        num_block = 0
        while True:
            UnitBlock, sender = self.sock.recvfrom(self.data_size)
            if len(UnitBlock)==0:
                self.img_bytes=[]
            else:

                UnitIdx = np.fromstring(UnitBlock[3:7], dtype = "int")[0]
                UnitSize = np.fromstring(UnitBlock[7:11], dtype = "int")[0]
                UnitTail = UnitBlock[-2:]
                
                if num_block==UnitIdx:
                    TotalBuffer+=UnitBlock[11:(11 + UnitSize)]
                    num_block+=1

                if UnitTail==b'EI' and num_block==self.max_len:

                    self.img_byte = TotalBuffer
                    print(len(self.img_byte))
                    if len(self.img_byte) != 0 :

                        self.img_msg.format = "jpeg"

                        self.img_msg.data = self.img_byte

                        self.publisher.publish(self.img_msg)
                
                    TotalBuffer = b''
                    num_block = 0

                elif UnitTail==b'EI' and num_block!=self.max_len:

                    TotalBuffer = b''
                    num_block = 0


    def __del__(self):
        self.sock.close()
        sys.exit()
        print('del')



class UDP_LIDAR_Parser :
    
    def __init__(self, publisher, ip, port, params_lidar=None):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        print(ip,port)
        self.publisher = publisher
        self.sock.bind(recv_address)

        self.data_size=params_lidar["Block_SIZE"]

        
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

    def recv_udp_data(self):

        Buffer = b''
        
        while True:            

            UnitBlock, sender = self.sock.recvfrom(self.data_size)           
            if len(UnitBlock)==0:
                return [], [], []

            else:
                AuxData = np.frombuffer(UnitBlock[13:25], dtype=np.float32)
                Buffer+=UnitBlock[25:]
                Buffer_dist = b''

                for i in range(360):                
                    Buffer_dist+=Buffer[(3*i):(3*i+2)]
            
                Distance = np.frombuffer(Buffer_dist, dtype=np.uint8)

                Distance = (Distance[0::2].astype(np.float32) + 256*Distance[1::2].astype(np.float32))/1000

                Distance = np.concatenate([Distance[180:],Distance[:180]])

                Intensity = np.frombuffer(Buffer[0::3], dtype=np.ubyte)

                return Distance, Intensity, AuxData

    def __del__(self):
        self.sock.close()
        sys.exit()
        print('del')

