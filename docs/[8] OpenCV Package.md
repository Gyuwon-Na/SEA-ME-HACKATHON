# OpenCV Package 가이드

<br>

## 1) OpenCV 패키지가 하는 일

`opencv` 패키지는 카메라의 JPEG 압축 이미지 토픽을 subscribe해서,
영상처리 결과 3가지를 별도 토픽으로 publish 합니다.

처리 순서(프레임당):
1. Grayscale (`cvtColor`)
2. Blur (`GaussianBlur`)
3. Edge (`Canny`)

<br>

## 2) 입출력 토픽 구조
### 2-1. 입력

- `subscribe_topic` (기본: `/camera/image/compressed`)
- 타입: `sensor_msgs/msg/CompressedImage`
### 2-2. 출력

- `/opencv/image/grayscale` (`CompressedImage`)
- `/opencv/image/blur` (`CompressedImage`)
- `/opencv/image/edge` (`CompressedImage`)

모든 출력은 JPEG(`format='jpeg'`)로 publish 됩니다.

<br>

## 3) opencv_node 구동 방법
### 3-1. 빌드

```bash
cd /home/topst/D-Racer
colcon build --packages-select opencv
source install/setup.bash
```
### 3-2. 기본 실행

```bash
ros2 run opencv opencv_node
```
### 3-3. 입력 토픽 지정 실행 예시

```bash
ros2 run opencv opencv_node --ros-args \
  -p subscribe_topic:=/camera/image/compressed
```
### 3-4. JPEG 품질/로그 설정 예시

```bash
ros2 run opencv opencv_node --ros-args \
  -p jpeg_quality:=85 \
  -p debug_log:=false
```

<br>

## 4) ROS 파라미터 설명

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `subscribe_topic` | string | `/camera/image/compressed` | 입력 이미지 토픽 |
| `jpeg_quality` | int | `90` | 출력 JPEG 품질(0~100) |
| `debug_log` | bool | `True` | 프레임 처리/publish 로그 출력 여부 |

참고:
- `jpeg_quality`가 높을수록 화질은 좋아지지만 대역폭/CPU 사용량이 증가합니다.

<br>

## 5) 내부 처리 방식

`opencv_node`는 콜백에서 다음을 수행합니다.

1. `CompressedImage.data`를 `numpy.frombuffer`로 디코드 준비
2. `cv2.imdecode(..., cv2.IMREAD_COLOR)`로 BGR 프레임 복원
3. `cv2.cvtColor(..., cv2.COLOR_BGR2GRAY)`
4. `cv2.GaussianBlur(..., (5,5), 0)`
5. `cv2.Canny(..., 50, 150)`
6. 각 결과를 `cv2.imencode('.jpg', ...)` 후 토픽 publish

즉, 원본 1프레임 입력 시 결과 3프레임이 각각 publish 됩니다.

<br>

## 6) 토픽 점검 명령

```bash
ros2 topic list | grep opencv
ros2 topic hz /opencv/image/grayscale
ros2 topic hz /opencv/image/blur
ros2 topic hz /opencv/image/edge
ros2 topic echo /opencv/image/grayscale --once
```

<br>

## 7) Monitor 패키지 연동

`monitor_node`에서 `debug_image=true`일 때,
다음 토픽을 받아 대시보드 디버그 패널 3개에 표시합니다.

- `/opencv/image/grayscale`
- `/opencv/image/blur`
- `/opencv/image/edge`

연동 체크 순서:
1. `camera_node` 실행
2. `opencv_node` 실행
3. `monitor_node --ros-args -p debug_image:=true` 실행
4. `ros2 topic hz`로 OpenCV 3개 토픽 publish 확인

<br>

## 8) 자주 발생하는 이슈
### 8-1. OpenCV 토픽이 생성되지 않을 때

1. `subscribe_topic`이 Camera 출력 토픽과 일치하는지 확인
2. `camera_node`가 실제로 프레임 publish 중인지 확인
3. `opencv_node` 로그에 decode 실패 경고가 있는지 확인
### 8-2. 모니터에 디버그 패널이 안 뜰 때

1. `monitor_node`의 `debug_image`가 `true`인지 확인
2. Monitor 파라미터의 OpenCV 토픽명이 실제 publish 토픽과 일치하는지 확인
### 8-3. 지연/부하가 클 때

- `jpeg_quality`를 낮추기
- 카메라 FPS(`publish_hz`)를 조정하기
