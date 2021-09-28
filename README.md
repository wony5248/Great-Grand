# Object detection

## install 추가파일 추가 위치

```
install/sub1/Lib/site-packages/sub1
```
안에 install추가파일 모두 복붙

## 낙상감지 실행법

빌드, 시뮬레이션 실행, 런처 이후

```
ros2 run sub1 perception
```

* 에러 날 수 있음 이 때 경로를 본인 경로에 맞게 수정이 필요함...

지금 코드의 import문에서 'ctrl + 클릭' 으로 함수에 들어갈 수 있다.<br>
들거가서 절대 경로에 대한 부분을 바꾸어 줄것

(아래 두개 외에 더 있을 수 있음)

```python
# TinyYOLOv3_onecls class 안에서 config_file, weight_file
from .FallDownDetection.DetectorLoader import TinyYOLOv3_onecls

# TSSTG class 안에서 weight_file
from .FallDownDetection.ActionsEstLoader import TSSTG
```

## 사용자 인식(현재 에러남)

빌드, 시뮬레이션 실행, 런처 이후

```
ros2 run sub1 make_user
```
