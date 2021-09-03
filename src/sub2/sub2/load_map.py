import rclpy
import numpy as np
from rclpy.node import Node

import os
from geometry_msgs.msg import Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from math import pi

# load_map 노드는 맵 데이터를 읽어서, 맵 상에서 점유영역(장애물) 근처에 로봇이 움직일 수 없는 영역을 설정하고 맵 데이터로 publish 해주는 노드입니다.
# 추 후 a_star 알고리즘에서 맵 데이터를 subscribe 해서 사용합니다.

# 노드 로직 순서
# 1. 맵 파라미터 설정
# 2. 맵 데이터 읽고, 2차원 행렬로 변환
# 3. 점유영역 근처 필터처리

class loadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        time_period=1  
        self.timer = self.create_timer(time_period, self.timer_callback)
       
        # 로직 1. 맵 파라미터 설정
        # 제공한 맵 데이터의 파라미터입니다. size_x,y는 x,y 방향으로 grid의 개수이고, resolution은 grid 하나당 0.05m라는 것을 의미합니다.
        # offset_x,y 의 -8, -4 는 맵 데이터가 기준 좌표계(map)로 부터 떨어진 거리를 의미합니다. 
        # 각 항에 -8.75를 뺀이유는 ros에서 occupancygrid의 offset이라는 데이터는 맵의 중앙에서 기준좌표계까지 거리가 아니라 맵의 우측하단에서 부터 기준좌표계까지의 거리를 의미합니다.
        # 따라서 (350*0.05)/2를 해준 값을 빼줍니다.
        self.map_msg=OccupancyGrid()
        self.map_size_x=350 
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
        self.map_data = [0 for i in range(self.map_size_x*self.map_size_y)]
        grid=np.array(self.map_data)
        grid=np.reshape(grid,(350, 350))

        self.map_msg.header.frame_id="map"

   

        m = MapMetaData()
        m.resolution = self.map_resolution
        m.width = self.map_size_x
        m.height = self.map_size_y
        m.origin = Pose()
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.map_meta_data = m
        self.map_msg.info=self.map_meta_data
        
        '''
        로직 2. 맵 데이터 읽고, 2차원 행렬로 변환

        full_path=
        self.f=
        
        line=
        line_data=
        
        for num,data in enumerate(line_data) :
            self.map_data[num]=
   
        map_to_grid=
        grid=
        '''


        for y in range(350):
            for x in range(350):
                if grid[x][y]==100 :

                    '''
                    로직 3. 점유영역 근처 필터처리

                    채워 넣기

                    '''

        
        np_map_data=grid.reshape(1,350*350) 
        list_map_data=np_map_data.tolist()
   
   
        ## 로직2를 완성하고 주석을 해제 시켜주세요.
        ## self.f.close()
        print('read_complete')
        self.map_msg.data=list_map_data[0]


    def timer_callback(self):
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

       
def main(args=None):
    rclpy.init(args=args)

    load_map = loadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()