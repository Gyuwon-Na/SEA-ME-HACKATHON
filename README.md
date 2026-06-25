# TOPST D-Racer Kit

<!--
Cover image area

Add the main D-Racer Kit cover image here.
Recommended width: 900~1200px

Example:

<p align="center">
  <img src="docs/asset/readme/d-racer-cover.jpg" alt="TOPST D-Racer Kit" width="900">
</p>
-->

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/readme/D-Racer-main-figure.png" alt="D-Racer KIT" width="720">
  <br>
  <b>D-Racer KIT</b>
</p>

<br>


TOPST D-Racer Kit는 **D3-G를 이용한 ROS2 기반 Racing Kit**입니다. 사용자는 D3-G 플랫폼 위에서 카메라, 조이스틱, 모터 제어, 배터리 상태, 웹 모니터링 기능을 ROS2 패키지로 다루며 RC Racing 시스템을 구성하고 확장할 수 있습니다.

이 저장소는 D-Racer Kit의 ROS2 패키지 소스 코드와 조립, 개발환경 설정, 패키지별 사용 가이드를 제공합니다.

<br>

## 1. Overview

D-Racer Kit는 D3-G 기반 차량 플랫폼을 ROS2에서 제어할 수 있도록 구성한 Racing Kit입니다. 하드웨어 제어와 센서 데이터를 ROS2 토픽과 노드 구조로 분리해, 사용자가 수동주행, 데이터 수집, 모니터링, 영상처리 실험 등을 단계적으로 구현할 수 있습니다.

주요 구성 요소:

- D3-G 기반 Racing Kit 플랫폼
- ROS2 패키지 기반 소프트웨어 구조
- 카메라 영상 스트리밍
- 조이스틱 기반 수동주행
- 차량 throttle / steering 제어
- 배터리 상태 모니터링
- 웹 기반 대시보드
- OpenCV 영상처리 테스트 패키지

<br>

## 2. Key Features

### 2-1. D3-G 기반 Racing Kit

D-Racer Kit는 D3-G를 차량 제어 플랫폼으로 사용합니다. ROS2 환경에서 차량의 구동부와 센서 데이터를 다룰 수 있도록 패키지가 구성되어 있어, RC Racing 실습과 ROS2 기반 로봇 소프트웨어 개발에 활용할 수 있습니다.

### 2-2. ROS2 Compatible Platform

각 기능은 ROS2 노드와 토픽 중심으로 분리되어 있습니다. 사용자는 필요한 패키지만 실행하거나, launch 파일을 통해 여러 노드를 함께 실행할 수 있습니다.

```text
Joystick Node  --->  Control Node  --->  Motor / Steering

Camera Node    --->  Monitor Node  --->  Web Dashboard

Battery Node   --->  Monitor Node
```

### 2-3. Manual Driving

조이스틱 입력을 ROS2 토픽으로 변환하고, 제어 패키지에서 throttle과 steering 명령으로 전달해 차량을 수동으로 주행할 수 있습니다. 하드웨어 조립 후 동작 확인, 주행 테스트, 데이터 수집 전에 기본 제어 상태를 확인하는 용도로 사용할 수 있습니다.

### 2-4. Camera Streaming

카메라 패키지는 차량에 장착된 카메라 영상을 ROS2 `sensor_msgs/msg/CompressedImage` 형식으로 publish합니다.

기본 이미지 토픽:

```text
/camera/image/compressed
```

### 2-5. Web Monitoring Dashboard

Monitor 패키지는 ROS2 토픽 데이터를 웹 대시보드로 제공합니다. 차량 상태를 브라우저에서 확인할 수 있어 주행 테스트 중 디버깅과 상태 확인에 활용할 수 있습니다.

대시보드에서 확인할 수 있는 정보:

- 메인 카메라 스트림
- 배터리 상태
- 제어값 throttle / steering
- 녹화 상태
- 저장장치 사용량
- ROS2 노드 및 토픽 상태
- OpenCV 디버그 이미지

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/readme/dashboard-example.png" alt="D-Racer Monitor Dashboard" width="720">
  <br>
  <b>D-Racer Monitor Dashboard</b>
</p>

<br>

## 3. Demo

### 3-1. OpenCV 기반 자율주행 테스트

OpenCV 기반 자율주행은 D-Racer Kit에서 카메라 영상처리와 차량 제어 흐름을 검증하기 위해 테스트한 데모입니다. 기본 제품 소개의 핵심 기능이라기보다는, ROS2 토픽 구조 위에서 영상처리 결과를 주행 제어에 연결할 수 있음을 보여주는 예시로 볼 수 있습니다.

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/readme/D-Racer-opencv-lane-following.gif" alt="D-Racer OpenCV Lane Following Test" width="720">
  <br>
  <b>D-Racer OpenCV Lane Following Test</b>
</p>

### 3-2. OpenCV Debug View

OpenCV 패키지는 카메라 입력 영상을 Grayscale, Blur, Edge 영상으로 변환합니다. Monitor 패키지의 `debug_image=true` 옵션을 사용하면 웹 대시보드에서 처리 결과를 실시간으로 확인할 수 있습니다.

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/9/figure1-dashboard-opencv-debug-mode.png" alt="D-Racer OpenCV Debug View" width="720">
  <br>
  <b>OpenCV Debug View</b>
</p>

<br>

## 4. Package Structure

| Package | Description |
|---|---|
| `camera` | 카메라 이미지 publish |
| `control` | 차량 throttle / steering 제어 |
| `joystick` | 조이스틱 입력 처리 및 수동주행 |
| `monitor` | 웹 기반 상태 모니터링 |
| `battery` | 배터리 상태 publish |
| `opencv` | OpenCV 기반 영상처리 테스트 |
| `topst_utils` | D3-G 및 하드웨어 제어 유틸리티 |
| `battery_msgs` | 배터리 custom message |
| `control_msgs` | 제어 custom message |
| `joystick_msgs` | 조이스틱 custom message |

<br>

## 5. Quick Start

### 5-1. Build

```bash
cd ~/D-Racer-Kit
colcon build
source install/setup.bash
```

### 5-2. Manual Driving

```bash
ros2 launch control manual_driving.launch.py
```

### 5-3. Monitor Dashboard

```bash
ros2 run monitor monitor_node
```

실행 로그에 표시되는 웹 주소로 접속하면 D-Racer 상태와 카메라 영상을 브라우저에서 확인할 수 있습니다.

예시:

```text
http://<D-Racer-IP>:5000
```

### 5-4. OpenCV Debug Mode

OpenCV 처리 영상을 모니터 대시보드에서 함께 확인하려면 `debug_image=true` 옵션을 사용합니다.

```bash
ros2 run monitor monitor_node --ros-args -p debug_image:=true
```

<br>

## 6. OpenCV Topic Flow

OpenCV 패키지는 카메라 이미지 토픽을 입력으로 받아 영상처리 결과를 별도 토픽으로 publish합니다.

Input:

```text
/camera/image/compressed
```

Output:

```text
/opencv/image/grayscale
/opencv/image/blur
/opencv/image/edge
```

토픽 상태 확인:

```bash
ros2 topic list | grep opencv
ros2 topic hz /opencv/image/grayscale
ros2 topic hz /opencv/image/blur
ros2 topic hz /opencv/image/edge
```

<br>

## 7. Recommended README Assets

README 소개 자료를 완성할 때 아래 자료를 추가하면 좋습니다.

| Asset | Purpose | Suggested Path |
|---|---|---|
| 표지 이미지 | D-Racer Kit 첫인상 전달 | `docs/asset/readme/d-racer-cover.jpg` |
| 키트 구성 이미지 | D3-G 기반 Racing Kit 구성 소개 | `docs/asset/readme/d-racer-kit.jpg` |
| Monitor Dashboard 화면 | 웹 모니터링 기능 소개 | `docs/asset/readme/monitor-dashboard.png` |
| OpenCV 테스트 영상 | 자율주행 테스트 데모 소개 | YouTube link 또는 GitHub uploaded mp4 |
| OpenCV Debug 화면 | 영상처리 테스트 결과 소개 | `docs/asset/readme/opencv-debug-view.png` |

<br>

## 8. Documentation

자세한 조립, 개발환경 설정, 패키지별 사용법은 아래 문서를 참고합니다.

| No. | Document |
|---|---|
| 1 | [D-Racer Hardware Assembly Guide](docs/%5B1%5D%20D-Racer%20Hardware%20Assembly%20Guide.md) |
| 2 | [Development Environment Setup Guide](docs/%5B2%5D%20Development%20Environment%20Setup%20Guide.md) |
| 3 | [Claude Code CLI Guide](docs/%5B3%5D%20Claude%20Code%20CLI%20Guide.md) |
| 4 | [D-Racer ROS2 Package Build Guide](docs/%5B4%5D%20D-Racer%20ROS2%20Package%20Build%20Guide.md) |
| 5 | [Monitor Package](docs/%5B5%5D%20Monitor%20Package.md) |
| 6 | [Battery Package](docs/%5B6%5D%20Battery%20Package.md) |
| 7 | [Camera Package](docs/%5B7%5D%20Camera%20Package.md) |
| 8 | [Joystick & Control Package](docs/%5B8%5D%20Joystick%20%26%20Control%20Package.md) |
| 9 | [OpenCV Package](docs/%5B9%5D%20OpenCV%20Package.md) |

<br>

## 9. Repository Layout

```text
D-Racer-Kit/
├── src/
│   ├── camera/
│   ├── control/
│   ├── joystick/
│   ├── monitor/
│   ├── opencv/
│   ├── battery/
│   ├── topst_utils/
│   ├── battery_msgs/
│   ├── control_msgs/
│   └── joystick_msgs/
├── docs/
├── bagfile/
├── README.md
└── LICENSE
```

<br>

## 10. License

This project is licensed under the terms described in [LICENSE](LICENSE).
