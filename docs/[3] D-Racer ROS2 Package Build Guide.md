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

### 1.1 필요 유틸리티 설치 - Flask
웹 인터페이스 관련 기능을 위해 Flask를 설치합니다.
```bash
python3 -m pip install --user Flask==2.2.5
pip3 show Flask # 설치 확인
```

<br>

## 2 ) ROS2 기초
이 섹션에서는 ROS2의 기본 개념과 D-Racer에 적용된 ROS2 구조를 설명합니다.

### 2.1 ROS2 개요
<내용 작성 예정>

### 2.2 ROS2 Node와 Topic
<내용 작성 예정>

### 2.3 D-Racer ROS2 구조
<내용 작성 예정>

<br>

## 3 ) D-Racer ROS2 Package Diagram
D-Racer의 ROS2 패키지 구조와 각 패키지 간의 관계를 다음과 같이 구성됩니다.

<D-Racer ROS2 Package Diagram 이미지 첨부>

### 주요 패키지 설명
- **battery**: 배터리 상태 모니터링 및 전력 관리
- **camera**: 카메라 이미지 처리 및 전송
- **control**: 차량 제어 로직 및 명령 처리
- **joystick**: 조이스틱 입력 처리
- **monitor**: 시스템 모니터링 및 로깅
- **opencv**: OpenCV 기반 비전 처리
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
```bash
cd ~/D-Racer
colcon build
```

### 4.3 빌드 결과 확인
빌드가 완료되면 install 디렉토리에 설치 파일이 생성됩니다.
```bash
source install/setup.bash
```

### 4.4 개별 패키지 빌드
특정 패키지만 빌드하려면 다음 명령어를 사용합니다.
```bash
colcon build --packages-select <package_name>
```

<br>