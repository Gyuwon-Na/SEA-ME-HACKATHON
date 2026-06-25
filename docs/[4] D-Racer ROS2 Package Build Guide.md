# D-Racer ROS2 패키지 빌드 가이드
본 가이드는 D-Racer의 ROS2 패키지 빌드 환경 구성 및 빌드 방법을 담은 문서입니다.
아래 내용을 포함합니다.
- 패키지 빌드 전 사전 환경 셋업
- ROS2 기초
- D-Racer ROS2 Package Diagram
- D-Racer 패키지 빌드 가이드

<br>

## 1 ) 사전 환경 셋업
ROS2 패키지 빌드 전에 필요한 환경 설정을 진행합니다.

### 1.1 필요 유틸리티 설치 - OpenCV
컬러 기반 이미지 영상처리 기능을 사용하기 위해 OpenCV를 사용합니다.
```bash
sudo apt install python3-opencv
```

### 1.2 ROS2 설정 환경 자동 적용
아래 커맨드를 통해서 ROS2 설정 환경을 자동 등록하실 수 있습니다.
해당 명령어는 한 번만 실행하면 되며, 이후 새로 여는 터미널부터 ROS 2 환경이 자동으로 적용됩니다.

```bash
echo "source /opt/ros/humble/local_setup.bash" >> ~/.bashrc # ~/.bashrc에 ROS2 환경설정 등록
tail -n 10 ~/.bashrc  # ~/.bashrc 최하단 10줄 출력
source ~/.bashrc # 환경설정 적용
```

<br>

## 2 ) ROS2 기초
이 섹션에서는 D-Racer 패키지를 빌드하고 실행하기 전에 알아야 할 ROS2의 기본 개념을 설명합니다.
ROS2를 처음 접하는 학생은 모든 내부 구조를 한 번에 외우기보다, `Node`, `Topic`, `Message`, `Package`, `Workspace`, `Launch`의 역할을 먼저 이해하는 것이 중요합니다.

### 2.1 ROS2는 무엇인가
ROS2(Robot Operating System 2)는 로봇 프로그램을 여러 기능 단위로 나누어 실행하고, 각 기능이 서로 데이터를 주고받을 수 있게 해주는 미들웨어입니다.
D-Racer처럼 카메라, 조이스틱, 모터 제어, 배터리 모니터링이 함께 동작하는 시스템에서는 하나의 큰 프로그램으로 모든 기능을 처리하기보다, 기능별 프로그램을 나누어 관리하는 방식이 더 이해하기 쉽고 유지보수하기 좋습니다.

D-Racer에서는 다음 기능들이 동시에 동작할 수 있습니다.
- 카메라 이미지를 읽는 기능
- 조이스틱 입력을 읽는 기능
- 차량의 조향과 속도를 제어하는 기능
- 배터리 상태를 확인하는 기능
- 웹 화면에서 상태를 확인하는 기능

ROS2는 이런 기능들을 각각 분리해서 실행하고, 필요한 데이터만 서로 주고받도록 도와줍니다.(Figure 1)

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/4/figure1-ros2-communication-structure.png" alt="ROS2 Communication Structure">
  <br>
  <b>Figure 1. ROS2 Communication Structure</b>
</p>


### 2.2 Node, Topic, Message
ROS2에서 가장 먼저 이해해야 할 개념은 `Node`, `Topic`, `Message`입니다.

`Node`는 하나의 기능을 담당하는 실행 단위입니다.
예를 들어 카메라 이미지를 읽는 프로그램, 조이스틱 값을 읽는 프로그램, 모터를 제어하는 프로그램이 각각 하나의 Node가 될 수 있습니다.

`Topic`은 Node들이 데이터를 주고받는 통로입니다.
한 Node가 특정 Topic으로 데이터를 보내면, 그 Topic을 구독하는 다른 Node가 데이터를 받아 사용할 수 있습니다.

`Message`는 Topic을 통해 주고받는 데이터의 형식입니다.
예를 들어 조이스틱 입력은 조향값과 속도값을 담은 Message로 전달되고, 카메라 이미지는 압축 이미지 Message로 전달됩니다.

쉽게 생각하면 다음과 같습니다.(Figure 2)
- Node: 일을 하는 담당자
- Topic: 담당자들이 쓰는 연락 채널
- Message: 연락 채널로 주고받는 내용

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/4/figure2-node-topic-message-relationship.png" alt="Node Topic Message Relationship">
  <br>
  <b>Figure 2. Node, Topic, Message Relationship</b>
</p>

### 2.3 D-Racer에서 사용하는 ROS2 예시
D-Racer 패키지에서도 Node와 Topic을 이용해 기능을 나누어 실행합니다. 아래 표는 D-Racer에서 사용되는 대표적인 예시입니다.

| 기능 | 예시 Node | 주요 Topic | 설명 |
| --- | --- | --- | --- |
| 카메라 | `camera_node` | `/camera/image/compressed` | USB 카메라 이미지를 읽어 압축 이미지로 전달합니다. |
| 이미지 처리 | `opencv_node` | `/opencv/image/grayscale`, `/opencv/image/blur`, `/opencv/image/edge` | 카메라 이미지를 받아 OpenCV 처리 결과를 다시 전달합니다. |
| 조이스틱 | `joystick_node` | `/joystick` | 조이스틱 입력을 조향, 속도, 비상정지 상태로 변환합니다. |
| 차량 제어 | `control_node` | `/control`, `/joystick` | 제어 명령을 받아 실제 조향/스로틀 출력을 계산합니다. |
| 배터리 | `battery_node` | `/battery_status` | 배터리 상태를 읽어 모니터링용 데이터로 전달합니다. |
| 모니터 | `monitor_node` | 여러 상태 Topic | 카메라, 배터리, 제어 상태를 웹 화면으로 보여줍니다. |

이 구조에서 중요한 점은 모든 Node가 서로 직접 연결되어 있는 것이 아니라, Topic을 기준으로 필요한 데이터만 주고받는다는 것입니다.
예를 들어 `camera_node`는 카메라 이미지를 `/camera/image/compressed` Topic으로 보내고, `opencv_node`나 `monitor_node`는 이 Topic을 구독해서 이미지를 사용할 수 있습니다.


### 2.4 Package와 Workspace
ROS2에서 `Package`는 관련된 Node, 설정 파일, 실행 파일을 묶어 놓은 단위입니다.
D-Racer 저장소의 `src` 디렉터리 안에는 여러 Package가 들어 있습니다.

예를 들어:
- `camera`: 카메라 입력 처리
- `opencv`: OpenCV 영상 처리
- `joystick`: 조이스틱 입력 처리
- `control`: 차량 제어
- `battery`: 배터리 상태 확인
- `monitor`: 웹 기반 상태 모니터
- `*_msgs`: D-Racer에서 사용하는 Message 정의

`Workspace`는 여러 ROS2 Package를 모아 빌드하고 실행하는 작업 공간입니다.
이 문서에서는 `D-Racer-Kit` 디렉터리를 하나의 ROS2 Workspace로 사용합니다.

```bash
sooyong@TOPST-Build-SVR:~/D-Racer-Kit$ tree -L 2
.
├── bagfile
│   └── bagfile.md
├── docs
│   ├── [1] D-Racer Hardware Assembly Guide.md
│   ├── [2] Development Environment Setup Guide.md
│   ├── [3] Claude Code CLI Guide.md
│   ├── [4] D-Racer ROS2 Package Build Guide.md
│   ├── [5] Monitor Package.md
│   ├── [6] Battery Package.md
│   ├── [7] Camera Package.md
│   ├── [8] Joystick & Control Package.md
│   ├── [9] OpenCV Package.md
│   └── asset
├── LICENSE
├── README.md
└── src
    ├── battery
    ├── battery_msgs
    ├── camera
    ├── config
    ├── control
    ├── control_msgs
    ├── data_acquisition.sh
    ├── image_raw.jpg
    ├── joystick
    ├── joystick_msgs
    ├── monitor
    ├── opencv
    └── topst_utils

15 directories, 13 files
```


### 2.5 ROS2 상태 확인 명령어
ROS2 패키지를 빌드하고 실행한 뒤에는 아래 명령어로 현재 동작 상태를 확인할 수 있습니다.

현재 실행 중인 Node 목록을 확인 & 실행 결과입니다.
```bash
# ros2 node list

topst@TOPST:~/D-Racer-Kit$ ros2 node list
/control_node
/joystick_node
```


현재 사용 중인 Topic 목록을 확인 & 실행 결과입니다.
```bash
# ros2 topic list

topst@TOPST:~/D-Racer-Kit$ ros2 topic  list
/control
/joystick
/parameter_events
/rosout
```

특정 Topic으로 어떤 데이터가 오가는지 확인합니다.
```bash
# ros2 topic echo {토픽명}

topst@TOPST:~/D-Racer-Kit$ ros2 topic echo /battery_status
battery_status: 100.0
---
battery_status: 99.61000061035156
---
```

특정 Topic의 Message 타입을 확인합니다.
```bash
# ros2 topic info {토픽명}

topst@TOPST:~/D-Racer-Kit$ ros2 topic info /camera/image/compressed
Type: sensor_msgs/msg/CompressedImage
Publisher count: 1
Subscription count: 0
```

이 명령어들은 ROS2 시스템이 정상적으로 실행 중인지 확인할 때 가장 자주 사용합니다.
처음에는 모든 명령어를 외우기보다, `node list`는 실행 중인 기능 확인, `topic list`는 데이터 통로 확인, `topic echo`는 실제 데이터 확인이라고 이해하면 됩니다.


### 2.6 Launch와 설정 파일
ROS2에서는 여러 Node를 한 번에 실행하기 위해 `Launch` 파일을 사용합니다.
D-Racer도 수동 주행이나 자동 주행처럼 여러 기능이 함께 필요한 경우 Launch 파일로 필요한 Node를 한 번에 실행할 수 있습니다.

예를 들어 수동 주행 관련 Launch 파일은 다음 위치에 있습니다.
```bash
src/control/launch/manual_driving.launch.py
```

Launch 파일을 사용하면 조이스틱 Node, 제어 Node처럼 함께 실행되어야 하는 기능을 한 번에 시작할 수 있습니다.
또한 D-Racer의 일부 설정은 아래 YAML 파일에서 관리됩니다.
```bash
src/config/vehicle_config.yaml

# Camera configuration
USB_CAM: true
MIPI_CAM: false
USB_CAM_DEVICE: /dev/video1
IMAGE_WIDTH: 320
IMAGE_HEIGHT: 160
ROI_TOP: 50
ROI_LEFT: 0

# Vehicle control parameters
STEER_TRIM: 0.10000000000000003

# Web server configuration
WEB_HOST: # D3-G IP Address 입력
WEB_PORT: 5000
BATTERY_TOPIC: /battery_status
IMAGE_TOPIC: /camera/image/compressed
CONTROL_TOPIC: /control
JOYSTICK_TOPIC: /joystick
STORAGE_PATH: /
IMAGE_DISPLAY_WIDTH: 640
IMAGE_DISPLAY_HEIGHT: 480
OPENCV_DEBUG_MODE : true

```


이 파일에는 카메라 장치, Topic 이름, 웹 모니터 주소 같은 설정이 들어 있습니다.
처음부터 Launch 파일의 내부 문법을 모두 이해할 필요는 없습니다. 먼저 "여러 Node를 한 번에 실행하기 위한 파일"이라고 이해하고, 이후 실습하면서 조금씩 수정해보는 방식이 좋습니다.
아래는 control 패키지 내 manual_driving.launch 파일 일부입니다. 해당 launch 파일을 실행할 때, control 노드와 joystick 노드가 동시에 동작합니다.

```python

def generate_launch_description():
    vehicle_config_path = get_vehicle_config_path()
    allow_reverse = LaunchConfiguration('allow_reverse')

    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_reverse',
            default_value='true',
            description='Allow joystick throttle to go negative for reverse driving.',
        ),
        Node(  ## control 노드
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {
                    'use_joystick_control': True,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
        Node(  ## joystick 노드
            package='joystick',
            executable='joystick_node',
            name='joystick_node',
            output='screen',
            parameters=[
                {
                    'calibration_mode': True,
                    'allow_reverse': allow_reverse,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
    ])

```
<br>

## 3 ) D-Racer ROS2 Package Diagram
D-Racer의 ROS2 패키지 구조와 각 패키지 간의 관계는 다음과 같이 구성되어 있습니다.(Figure 3)

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/4/figure3-D-Racer-ros2-architecture.png" alt="D-Racer Package Diagram">
  <br>
  <b>Figure 3. D-Racer Package Diagram</b>
</p>


### 주요 패키지 설명
- **battery**: 배터리 상태 모니터링 및 전력 관리
- **camera**: 카메라 이미지 처리 및 전송
- **control**: 차량 제어 로직 및 명령 처리
- **joystick**: 조이스틱 입력 처리
- **monitor**: 시스템 모니터링 및 로깅
- **opencv**: OpenCV 기반 비전 처리 예제
- **topst_utils**: 공통 유틸리티 함수

<br>

## 4 ) D-Racer 패키지 빌드 가이드
D-Racer ROS2 패키지를 빌드합니다.

### 4.1 빌드 환경 확인
빌드 이전에 ROS2 환경이 제대로 설정되었는지 확인합니다.
```bash
echo $ROS_DISTRO
```

### 4.2 패키지 빌드
colcon을 사용하여 모든 패키지를 빌드합니다.
지정된 패키지를 빌드할 땐 워크스페이스 내에서 `colcon build` 를 진행하셔야 합니다.
```bash
cd ~/D-Racer-Kit
colcon build
```

### 4.3 빌드 결과 확인
빌드가 완료되면 install 디렉토리에 설치 파일이 생성됩니다.
추가 개발 이후에도 반드시 `colcon build` 를 통해 패키지 빌드를 진행해 주시고,
아래 커맨드를 통해 워크스페이스 환경 적용을 진행해주세요.
```bash
source install/setup.bash
```

### 4.4 개별 패키지 빌드
특정 패키지만 빌드하려면 다음 명령어를 사용합니다.
```bash
colcon build --packages-select <package_name>
```

<br>
