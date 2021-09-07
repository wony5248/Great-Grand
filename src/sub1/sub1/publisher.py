import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# publisher 노드는 ROS2에서 가장 기본이 되는 예제코드로 메시지 송신을 위한 기능만 가지고 있는 노드입니다.
# String 타입의 /test라는 메시지를 publish 합니다. 

# 노드 로직 순서
# 1. publisher 생성
# 2. 타이머 함수 생성
# 3. 송신할 메시지 변수 생성
# 4. 메시지 publish
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        # 로직 1 publisher 생성
        ## create_publisher라는 함수를 이용해 메시지를 publisher를 생성합니다. 매개 변수로는 메시지 타입, 토픽, 버퍼 개수를 넣어줍니다.
        ## 버퍼 개수는 publish 함수가 실제 메시지를 보내는 것 보다 빠르게 호출되면 버퍼에 쌓이게 됩니다. 

        self.str_publisher = self.create_publisher(String, '/test', 10)

        # 로직 2. 타이머 함수 생성
        ## 메인 루프가 돌아가는 타이머 함수입니다. time_period 값을 지정해서 주기적으로 실행할 수 있습니다. time_period의 단위는 초(second)입니다.
        time_period=0.1 
        self.timer = self.create_timer(time_period, self.timer_callback)

        # 로직 3. 송신할 메시지 변수를 생성
        ## publisher에 String 타입으로 만들었기 때문에 메시지 역시 String 타입으로 생성해줍니다.
        self.str_msg=String()
        self.count=0


    def timer_callback(self):

        self.count+=1
        self.str_msg.data='Hello world count : {}'.format(self.count)
        print(self.str_msg.data)
        # 로직 4. 메시지 publish
        ## publish 함수가 호출되야 메시지가 송신됩니다. 송신된 메시지는 다른 노드에서 subcriber 또는 rqt와 같은 gui 툴에서 확인할 수 있습니다.
        self.str_publisher.publish(self.str_msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    ## spin 함수는 노드가 죽지않게 계속 대기상태를 만들어 놓는 함수입니다. spin을 삭제하면 minimal_publisher가 생성되고 바로 main 함수가 끝나게 됩니다.
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
