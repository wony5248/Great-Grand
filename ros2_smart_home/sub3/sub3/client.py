import socketio


# client 는 socketio의 기본 API로 구성된 노드입니다. 서버와 연결을 시도해서 서버와 통신을 합니다.

# 각자의 서버 주소에 맞게 connect 함수 안을 바꿔주고, server 스켈레톤코드를 이용해 서비스를 하고 있다면, 연결이 됩니다.
# 버튼을 누르면 해당 키값에 맞는 함수들이 호출이 됩니다. 연결이 된 후에는 emit 함수를 이용해 서버로 키값과 데이터를 보냅니다.
# 이 노드는 AWS EC2에 구축한 서버와 통신만 하는 노드이고, ROS2와 연동하여 사용하면 스마트홈에서 얻은 데이터들을 서버로 보내고, 웹서버로부터의 명령을 ROS2로 전달할 수 있습니다.

# 노드 로직 순서
# 1. 클라이언트 소켓 생성
# 2. 데이터 수신 콜백함수
# 3. 서버 연결
# 4. 데이터 송신

# 로직 1. 클라이언트 소켓 생성
sio = socketio.Client()



@sio.event
def connect():
    print('connection established')


# 로직 2. 데이터 수신 콜백함수
@sio.on('sendAirConOn')
def aircon_on(data):
    print('message received with ', data)

@sio.on('sendAirConOff')
def aircon_off(data):
    print('message received with ', data)


@sio.event
def disconnect():
    print('disconnected from server')


# 로직 3. 서버 연결
sio.connect('http://ec2-3-34-134-166.ap-northeast-2.compute.amazonaws.com:12001/')

# 로직 4. 데이터 송신
sio.emit('sendTime','TEST')

sio.wait()