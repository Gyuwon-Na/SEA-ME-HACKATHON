# Camera Package 가이드

<br>

## 1) Camera 패키지가 하는 일

`camera` 패키지는 카메라(USB 또는 MIPI)에서 프레임을 읽어 JPEG 압축 이미지로 ROS 토픽에 publish 합니다.

핵심 기능:
- 카메라 입력(USB/MIPI) 선택
- GStreamer 파이프라인으로 프레임 수집
- JPEG 인코딩 후 `sensor_msgs/msg/CompressedImage` publish
- 해상도/디바이스/플립 방식/주기 파라미터 제어

<br>

## 2) 동작 구조

`camera_node` 동작 순서:

1. ROS 파라미터 + `vehicle_config.yaml` 로드
2. 카메라 소스(USB/MIPI) 및 디바이스 결정
3. 후보 GStreamer 파이프라인으로 카메라 오픈 시도
4. 타이머 주기(`publish_hz`)마다 프레임 읽기
5. JPEG 인코딩 후 `CompressedImage` publish

출력 메시지:
- 타입: `sensor_msgs/msg/CompressedImage`
- `format`: `jpeg`
- `header.frame_id`: `camera`

<br>

## 3) camera_node 구동 방법
### 3-1. 빌드

```bash
cd /home/topst/D-Racer
colcon build --packages-select camera
source install/setup.bash
```
### 3-2. 기본 실행

```bash
ros2 run camera camera_node
```
입력시, 아래 카메라 관련 정보 로그가 출력됩니다.

```bash
[INFO] [1780031364.989311613] [camera_node]: 
[Camera Node] : topic=camera/image/compressed 
[camera source] : usb 
[width] : 320, [height] : 160 
[camera_device] : /dev/video1 
[flip_method] : rotate-180 
[jpeg_quality] : 90 
[vehicle_config_file] : /home/topst/D-Racer/src/config/vehicle_config.yaml 
[debug_log] : False 
```


### 3-3. 파라미터 오버라이드 실행 예시

```bash
ros2 run camera camera_node --ros-args \
  -p publish_topic:=/camera/image/compressed \
  -p publish_hz:=30.0 \
  -p camera_device:=/dev/video1 \ #  video0 / video1 사용 가능
  -p jpeg_quality:=90
```
### 3-4. 카메라 작동 여부 확인 - 디버그 로그 출력

```bash
ros2 run camera camera_node --ros-args -p debug_log:=true
```
입력시, 아래 카메라 관련 정보 로그가 출력됩니다.

```bash
[INFO] [1780044122.011738490] [camera_node]: 
[Camera Node] : topic=camera/image/compressed 
[camera source] : usb 
[width] : 320, [height] : 160 
[camera_device] : /dev/video1 
[flip_method] : rotate-180 
[jpeg_quality] : 90 
[vehicle_config_file] : /home/topst/D-Racer/src/config/vehicle_config.yaml 
[debug_log] : False 
```

### 3-5. 카메라 프레임 사이즈 조절
config/vehicle_config.yaml에서 변경 가능합니다. 
```yaml
# Camera Setting 
USB_CAM: true
MIPI_CAM: false
USB_CAM_DEVICE: /dev/video1
IMAGE_WIDTH: 320
IMAGE_HEIGHT: 160
ROI_TOP: 50
ROI_LEFT: 0
```
### 3-6. 모니터 패키지 결과 확인
```bash
ros2 run monitor monitor_node
```
대쉬보드에 카메라 프레임을 출력한다. (Figure1) 

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/7/figure1-camera-status.png" alt="Camera Streaming">
  <br>
  <b>Figure 1. Dashboard-Camera Streaming</b>
</p>

<br>

## 4) 출력 토픽/메시지

- 기본 출력 토픽: `/camera/image/compressed`
- 메시지 타입: `sensor_msgs/msg/CompressedImage`

확인 명령:
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image/compressed
ros2 topic echo /camera/image/compressed --once
```

<br>

## 5) ROS 파라미터 설명

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `vehicle_config_file` | string | 자동 탐색 경로 | 차량 설정 YAML 파일 경로 |
| `publish_topic` | string | `/camera/image/compressed` | JPEG 이미지 publish 토픽 |
| `publish_hz` | float | `30.0` | 프레임 publish 주기(Hz), *현재 30hz 이상 성능은 불가 |
| `camera_device` | string | `/dev/video0` | 기본 카메라 디바이스 경로 |
| `usb_camera_device` | string | `/dev/video1` | USB 카메라 디바이스 경로(수정가능) |
| `mipi_camera_device` | string | `/dev/video0` | MIPI 카메라 디바이스 경로(수정가능) |
| `flip_method` | string | `rotate-180` | MIPI 파이프라인 `videoflip` 방식 |
| `jpeg_quality` | int | `90` | JPEG 품질(0~100) |
| `debug_log` | bool | `False` | 프레임 publish 로그 출력 여부 |

<br>

## 6) vehicle_config.yaml 연동 키

`camera_node`는 아래 키를 `vehicle_config.yaml`에서 읽어 동작을 보정합니다.

- `IMAGE_WIDTH`, `IMAGE_HEIGHT`: 출력 해상도
- `USB_CAM`, `MIPI_CAM`: 사용 카메라 소스 선택
- `USB_CAM_DEVICE`, `MIPI_CAM_DEVICE`: 디바이스 경로 오버라이드

규칙:
- `USB_CAM`, `MIPI_CAM` 중 **하나만 true** 여야 합니다.
- 둘 다 true 또는 둘 다 false면 노드가 예외를 발생시킵니다.

<br>

## 7) 내부 파이프라인 요약
### USB 선택 시

- MJPG 우선 파이프라인 시도
- 실패 시 raw 파이프라인 fallback
### MIPI 선택 시

- NV12 입력 -> `videoconvert` -> `videoflip` -> BGR 변환 -> appsink

즉, 카메라 종류에 따라 후보 파이프라인을 다르게 구성해 오픈 성공률을 높입니다.

<br>

## 8) 트러블슈팅
### 8-1. 카메라 오픈 실패

1. 디바이스 경로 확인 (`/dev/video0`, `/dev/video1` 등)
    - `MIPI_CAM` : /dev/video0
    - `USB_CAM` : /dev/video1
2. 카메라 USB 연결 상태 확인 후 재부팅 
3. 해상도(`IMAGE_WIDTH`, `IMAGE_HEIGHT`)를 낮춰 재시도
4. `flip_method` 값 변경 시도 - 때에 따라 화면을 전환해야하는 경우 
### 8-2. 토픽은 있는데 화면이 안 뜰 때

1. `ros2 topic hz`로 프레임 주기 확인
2. Monitor의 `image_topic`이 Camera publish 토픽과 일치하는지 확인
### 8-3. 프레임 publish가 불안정할 때

- `publish_hz`를 낮춰 CPU/카메라 부하 완화
- `jpeg_quality`를 낮춰 인코딩 부하 완화

<br>

## 9) OpenCV/Monitor 연동

- `opencv_node` 입력은 Camera 출력 토픽을 subscribe합니다.
- `monitor_node`는 Camera 메인 토픽과(옵션으로) OpenCV 결과 토픽을 대시보드에 표시합니다.

권장 연결 예시:
- Camera: `/camera/image/compressed`
- OpenCV subscribe: `/camera/image/compressed`
- Monitor image_topic: `/camera/image/compressed`
