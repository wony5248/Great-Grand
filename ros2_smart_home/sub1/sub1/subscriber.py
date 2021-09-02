import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# subscriber 노드는 ROS2에서 가장 기본이되는 예제코드로 메시지 수신을 위한 기능만 가지고 있는 노드입니다.
# String 타입의 /test라는 메시지를 subscribe 합니다. 

# 노드 로직 순서
# 1. subscriber 생성
# 2. callback 함수 생성


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        # 로직 1. subscriber 생성 
        ## 다른 노드에서 publish 되고 있는 메시지를 수신하기 위한 subscriber를 생성합니다. create_subscription 함수를 이용해 생성하고 매개변수로 메시지 타입, 토픽, 콜백함수, 버퍼를 받습니다.
        ## 메시지가 수신될 때 마다 콜백함수를 실행합니다. 메시지 수신속도가 너무 빨라 콜백함수가 처리되기 전에 메시지가 계속 들어 오면 버퍼에 쌓아두고 처리를 합니다. 
        self.subscription = self.create_subscription(String,'/test',self.listener_callback,10)
        
    # 로직 2. callback 함수 생성
    ## 메시지가 수신될 때 마다 호출되는 함수로 메시지의 내용을 출력하고 있습니다.
    def listener_callback(self,msg):
        print('Sub: '+msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











