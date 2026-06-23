# Joystick & Control Package 가이드

<br>

## 1) 이 문서에서 다루는 범위

이 문서는 두 패키지를 함께 설명합니다.
- `joystick` 패키지: 게임패드 입력을 주행 명령/상태로 변환
- `control` 패키지: 최종 조향/스로틀 명령을 실제 차량 액추에이터로 출력

즉, **입력(조이스틱) -> 제어 명령 -> 차량 구동** 경로의 핵심입니다.

<br>

## 2) 전체 동작 구조
### 2-1. Joystick Node 역할

- 게임패드 입력을 읽어서 `joystick_msgs/msg/Joystick` publish
- 내부에 `control_msgs/msg/Control`(throttle/steering) 포함
- 버튼 이벤트로 아래와 같은 기능을 수행합니다(figure 1)
  - 수동 주행 모드에서 동작가능한 조이스틱 기능은 아래와 같습니다.
    - 가감속 비율(accel_ratio) 조정 - (1) (2)
    - Steering Calibration : steering trim 조절 & 저장 - (4) (5)
    - 전진 기어 모드 - (9)
    - 후진 기어 모드 - (10)
    - 종방향 제어 스틱
      - 전진 기어 모드 일때 - (6)방향으로 전진을 할 수 있습니다.
      - 후진 기어 모드 일때 - (7)방향으로 후진을 할 수 있습니다. (**후진시, 스틱을 살살 동작하세요**)
    - 횡방향 제어 스틱 - (8)
  
  
  - 자율주행 모드에서 동작가능한 조이스틱 기능은 아래와 같습니다.
    - E-STOP 래치 - (3)
  
  - bagfile record start/stop 토글 - (11)
    - ROS2 bagfile을 취득한 일자-시간 순으로 기록합니다. 

<p align="center">  
  ![Figure 1. Joystick Mapping](topst-development url 첨부예정)
</p>

### 2-2. Control Node 역할

- `Joystick` 또는 `Control` 토픽 중 하나를 선택해 사용
- 최종 steering/throttle 명령을 `D3Racer`(PCA9685 기반)로 출력
- E-STOP 수신 시 throttle 강제 0

<br>

## 3) 기본 실행 방법
### 3-1. 빌드

```bash
cd /home/topst/D-Racer
colcon build --packages-select joystick control
source install/setup.bash
```
### 3-2. 수동 주행(조이스틱 직접 제어)

```bash
ros2 launch control manual_driving.launch.py
```
이 런치는 내부적으로:
- `control_node` (`use_joystick_control=True`)
- `joystick_node` (`calibration_mode=True`)
를 함께 실행합니다.
### 3-3. 자동 주행(외부/추론 제어)

```bash
ros2 launch control auto_driving.launch.py
```
이 런치에서 `control_node`는 `use_joystick_control=False`로 동작합니다.
(이때, 조이스틱은  E-STOP 용도로 동작하기 위해 사용됩니다.)
### 3-4. 노드 단독 실행 예시

```bash
ros2 run joystick joystick_node
ros2 run control control_node --ros-args -p use_joystick_control:=true
```
실행 후애는 아래와 같은 로그가 출력됩니다.
```bash
[joystick_node-2] [INFO] [1780024865.456599591] [joystick_node]: [Joystick DBG] 
[joystick_node-2] left_y=0.00 
[joystick_node-2] right_x=0.00 right_y=0.00 
[joystick_node-2] steering=0.10 throttle=0.00 
[joystick_node-2] accel_ratio=0.120 
[joystick_node-2] Gear=[D] 
[joystick_node-2] trim=0.10 
[joystick_node-2] e_stop=0 
[joystick_node-2] recording=0 
[joystick_node-2] L1=0 R1=0
[joystick_node-2] 
```

### 3-5. 모니터 패키지 결과 확인
```bash
ros2 run monitor monitor_node
```
1. 자율주행 모드에서 최종적으로 /control 토픽이 publish 되며, 해당 토픽 데이터는 대쉬보드 내 "실시간 제어값" 패널에서ㅇ 출력됩니다. (Figure1)  
* 수동주행 모드에서 대쉬보드에 제어값이 표출되지 않습니다. joystick 패키지가 실행되는 터미널을 통해 throttle, steering 값을 확인할 수 있습니다.
<p align="center">  
  ![Figure 1. Control Statua](topst-development url 첨부예정)
</p>

<br>

## 4) 토픽 구성
### 4-1. JoystickNode 출력

- 토픽: `joystick` (기본값)
- 타입: `joystick_msgs/msg/Joystick`
- 포함 데이터:
  - `control_msg.throttle`
  - `control_msg.steering`
  - `e_stop_en`
  - `is_recording`

### 4-2. ControlNode 입력
- 수동주행시,  `joystick_topic` (기본: `/joystick`) - 타입 `Joystick`
- `control_topic` (기본: `/control`) - 타입 `Control`

선택 규칙:
- `use_joystick_control=True` -> `Joystick` 기준으로 구동 - 수동주행을 위함
- `use_joystick_control=False` -> `Control` 기준으로 구동 - 자율주행을 위함 
  - E-STOP은 use_joystick_control 과 무관하게 항상 joystick에서 처리됌

<br>


## 5) ROS 파라미터 설명

<br>

## 5-1. JoystickNode 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `publish_topic` | string | `joystick` | Joystick 메시지 publish 토픽 |
| `publish_hz` | float | `50.0` | publish 주기(Hz) |
| `throttle_scale` | float | `0.12` | 기본 throttle 스케일 |
| `throttle_deadzone` | float | `0.05` | throttle deadzone (수정 불필요)|
| `steering_deadzone` | float | `0.05` | steering deadzone |
| `steering_axis` | string | `auto` | `right_x`/`right_y`/`auto` |
| `steering_trim` | float | `0.0` | steering 오프셋 |
| `calibration_mode` | bool | `False` | trim 버튼 캘리브레이션 모드 |
| `calibration_step` | float | `0.1` | trim 증감 단위 |
| `vehicle_config_file` | string | 자동 탐색 경로 | trim 저장 YAML 경로 |
| `data_acquisition_script` | string | 자동 탐색 경로 | 녹화 토글용 스크립트 경로 |
| `accel_ratio_step` | float | `0.005` | L1/R1 조정 단위 |
| `accel_ratio_min` | float | `0.12` | accel_ratio 최소값 |
| `accel_ratio_max` | float | `0.4` | accel_ratio 최대값 |
| `debug_log_enable` | bool | `True` | 디버그 로그 출력 여부 |
| `debug_log_hz` | float | `5.0` | 디버그 로그 주기 |

<br>

## 5-2. ControlNode 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `i2c_bus` | int | `3` | PCA9685 I2C bus 번호 (수정 불필요)|
| `pca9685_addr` | int | `0x40` | PCA9685 I2C 주소 (수정 불필요) |
| `steering_channel` | int | `0` | 조향 채널 (수정 불필요) |
| `throttle_channel` | int | `1` | 스로틀 채널 (수정 불필요) |
| `vehicle_config_file` | string | 자동 탐색 경로 | `STEER_TRIM` 로드 경로 |
| `use_joystick_control` | bool | `False` | 입력 소스 선택 플래그 |
| `joystick_topic` | string | `joystick` | Joystick 입력 토픽 |
| `control_topic` | string | `/control` | 외부 제어 입력 토픽 |
| `command_hz` | float | `10.0` | 액추에이터 출력 주기(Hz) |

<br>

## 6) 안전 동작(E-STOP)

- `Joystick`에서 `e_stop_en=True`가 들어오면 `ControlNode`가 즉시 E-STOP 상태로 진입합니다.
- E-STOP 상태에서는 throttle을 0으로 강제하고, 이후 들어오는 throttle 명령을 무시합니다.

운용 시에는 E-STOP 해제 절차를 별도 운영 규칙으로 정해두는 것을 권장합니다.

<br>

## 7) 상태 점검 명령

```bash
ros2 node list | grep -E 'joystick_node|control_node|gamepad_publisher'
ros2 topic list | grep -E 'joystick|control'
ros2 topic echo /joystick --once
ros2 topic echo /control --once
```

<br>

## 8) 자주 발생하는 이슈
### 8-1. 차량이 움직이지 않을 때

1. `control_node`가 실행 중인지 확인
2. `use_joystick_control` 값이 현재 운용 모드와 맞는지 확인
3. E-STOP이 걸렸는지(`X` 버튼) 확인
4. I2C/PCA9685 연결 확인
### 8-2. 조향이 한쪽으로 치우칠 때

- `STEER_TRIM` 값 점검
- `calibration_mode=true`에서 trim 재보정
### 8-3. 녹화 토글이 안 될 때

- `data_acquisition_script` 경로 유효성 확인
- 실행 권한(`chmod +x`) 확인

<br>

## 9) 권장 운영 시나리오

- 수동 디버깅: `manual_driving.launch.py`
- 자율 주행 테스트: `auto_driving.launch.py` + inference/control 입력 점검
- 변경 후 검증: 토픽 echo/hz + 실제 액추에이터 반응 순서로 확인
