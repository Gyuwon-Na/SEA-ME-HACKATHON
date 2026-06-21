# Monitor Package 가이드 

<br>

## 1) Monitor 패키지가 하는 일
`monitor` 패키지는 ROS 토픽 데이터를 웹 대시보드로 보여주는 패키지입니다.

핵심 기능:
- 배터리 상태 표시
- 메인 카메라 스트림 표시
- 제어값(Throttle/Steering) 표시
- 녹화 상태 표시
- 저장장치 사용량 표시
- `debug_image=true`일 때 OpenCV 처리 영상 3종(Grayscale/Blur/Edge) 표시
  - 자세한 사항은 [8] OpenCV Pacage 가이드 참고
  
<br>


## 2) 동작 구조 한눈에 보기
`monitor_node`는 다음 순서로 동작합니다.

1. ROS 파라미터와 `vehicle_config.yaml`을 읽음
2. 필요한 토픽을 subscribe
3. 수신 데이터를 `MonitorState`에 저장
4. 내장 Flask 서버가 `/api/status`, `/api/frame*` API로 상태/이미지를 제공
5. 웹 페이지(`index.html` + `app.js`)가 API를 주기적으로 호출해 화면 갱신

위와 같이, **ROS 토픽 웹 대시보드 제공**을 하나의 노드에서 처리합니다.

<br>

## 3) monitor_node 구동 방법
### 3-1. 빌드
```bash
cd /home/topst/D-Racer
colcon build --packages-select monitor
source install/setup.bash
```

### 3-2. 기본 실행
```bash
ros2 run monitor monitor_node
```

### 3-3. OpenCV 디버그 패널 포함 실행
```bash
ros2 run monitor monitor_node --ros-args -p debug_image:=true
```

### 3-4. 호스트/포트 변경 실행 예시
```bash
ros2 run monitor monitor_node --ros-args \
  -p web_host:=0.0.0.0 \
  -p web_port:=5000
```
혹은, config/vehicle_config.yaml 파일에서 수정 후 적용해도 동일합니다.  
브라우저 접속 주소는 로그의 `web=http://...` 값을 확인하면 됩니다.(사용자의 IP)  
해당 주소에 접속하면  figure 1 과 같이 대쉬보드가 웹화면에 동작합니다.

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/4/figure1-dashboard-default.png" alt="Dashboard Default">
  <br>
  <b>Figure 1. Dashboard Default</b>
</p>

<br>

## 4) 주요 ROS 파라미터 설명
아래 파라미터는 `monitor_node`에서 직접 사용됩니다.

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `vehicle_config_file` | string | 자동 탐색 경로 | YAML 설정 파일 경로 |
| `battery_topic` | string | `battery_status` | 배터리 토픽 |
| `image_topic` | string | `/camera/image/compressed` | 메인 카메라 이미지 토픽 (`sensor_msgs/CompressedImage`) |
| `debug_image` | bool | `true` | OpenCV 디버깅용 영상 패널 사용 여부 |
| `opencv_grayscale_topic` | string | `/opencv/image/grayscale` | 디버깅용 grayscale 이미지 토픽 |
| `opencv_blur_topic` | string | `/opencv/image/blur` | 디버깅용 blur 이미지 토픽 |
| `opencv_edge_topic` | string | `/opencv/image/edge` | 디버깅용 edge 이미지 토픽 |
| `control_topic` | string | `/control` | 제어 토픽 |
| `joystick_topic` | string | `joystick` | 조이스틱/녹화 상태 토픽 |
| `storage_path` | string | `/` | 저장공간 사용량 계산 대상 경로 |
| `storage_poll_interval_sec` | float | `1.0` | 저장공간 갱신 주기(초) |
| `web_host` | string | `192.168.0.12` | Flask 바인딩 호스트(수정가능) |
| `web_port` | int | `5000` | Flask 포트 |
| `page_title` | string | `D-Racer Monitor` | 대시보드 페이지 제목 |
| `refresh_interval_ms` | int | `1000` | 상태 API polling 주기(ms) |
| `image_refresh_interval_ms` | int | `100` | 이미지 갱신 주기(ms) |
| `stale_timeout_sec` | float | `3.0` | 데이터 stale 판단 기준 시간(초) |
| `image_source_width` | int | `160` | 원본 이미지 폭 fallback |
| `image_source_height` | int | `120` | 원본 이미지 높이 fallback |
| `image_display_width` | int | `160` | placeholder 표시 폭 |
| `image_display_height` | int | `120` | placeholder 표시 높이 |
| `debug_log` | bool | `false` | 모니터링 디버깅 로그 출력 여부 |

참고:
- 일부 값은 `vehicle_config.yaml`의 키(`BATTERY_TOPIC`, `IMAGE_TOPIC`, `DEBUG_IMAGE` 등)로 덮어쓸 수 있습니다.
- `debug_image=true`일 때만 OpenCV 3개 토픽을 subscribe 합니다.

<br>


## 5) 입력 토픽 요약
`monitor_node`가 subscribe하는 대표 토픽:
- 배터리: `battery_topic`
- 메인 카메라: `image_topic`
- 제어: `control_topic`
- 녹화 상태: `joystick_topic`
- 디버그 영상(옵션): `opencv_grayscale_topic`, `opencv_blur_topic`, `opencv_edge_topic`

<br>

## 6) 문제 해결 체크리스트
OpenCV 디버깅 영상 패널이 안 보일 때:
1. `debug_image`가 `true`인지 확인
2. `/opencv/image/grayscale`, `/opencv/image/blur`, `/opencv/image/edge` 토픽이 실제 publish 중인지 확인
3. `monitor_node` 실행 로그의 토픽/웹 주소가 기대값과 일치하는지 확인

유용한 확인 명령:
```bash
ros2 param get /monitor_node debug_image
ros2 topic list | grep opencv
ros2 topic hz /opencv/image/grayscale
```
