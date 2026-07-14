# D-Racer-Kit 현재 작업 요약

최종 갱신: 2026-07-14

## 2026-07-14 OUT 곡률 기반 pre-fork FSM

- OUT의 시간 기반 `OUT_ENTRY`/`OUT_S_CURVE` 상태를 제거했다. 초록불 이후 표지판 확정 전까지 단일 `OUT_TO_FORK` 상태로 흰 차선을 계속 추종한다.
- 직선은 `speed_max=0.30`, 커브는 흰 차선의 조향 요구량과 곡률에 따라 `speed_min=0.20` 방향으로 연속 감속한다.
- 시간이 지나거나 영상이 움직이는 것만으로는 상태가 바뀌지 않고, 좌/우 표지판 투표가 확정될 때만 `OUT_FORK_SIGN_ADVANCE`로 전이한다.
- 출발 격자에서 흰 차선을 아직 못 잡아도 `follow_with_startup()`이 중앙 조향과 `speed_min=0.20`을 출력하므로 AUTO 0 스로틀 교착은 해결된 상태다.
- OUT의 중심·곡률·조향·fork 목표는 모두 흰색 마스크 전용이다. 함께 보이는 노란 IN 점선은 사용하지 않는다.
- 비구동 C++ 시험: 표지판 없는 이동 영상은 `OUT_TO_FORK` 유지, 좌 표지판 투표에서만 fork 상태 전이, 직선 최대 0.300/커브 최대 0.259. 회귀 시험 `18 passed`.

## 작업 기준

- 실제 수정·빌드 대상: `topst@192.168.1.104:/home/topst/D-Racer-Kit`
- 로컬 `/home/hyun/D-Racer-Kit`은 rosbag 입력과 분석에만 사용하며 차량 코드의 원본으로 덮어쓰지 않는다.
- 앞으로 모든 ROS 실행·조회·시험은 `ROS_DOMAIN_ID=2`를 사용한다.
- 실차 모터/서보를 움직이는 시험은 사용자 명시 승인 없이 실행하지 않는다.
- rosbag 시험은 `enable_camera:=false`, `enable_actuation:=false`, `enable_joystick:=false`로 격리한다.
- rosbag 시험 후 bag player, launch 및 자식 노드가 모두 종료됐는지 확인한다.

## 속도 정책

- 전역 속도 범위는 `speed_min=0.20`, `speed_max=0.30`이다.
- 직선 판단 deadband 안에서는 목표 속도가 `0.30`까지 올라간다.
- 커브에서는 흰색 차선 기준 조향량과 곡률에 따라 `0.20` 방향으로 연속 감속한다.
- 초록불~표지판 구간은 별도 S cap 없이 `0.30`에서 곡률 기반 감속한다.
- fork 접근 `0.24`, fork commit `0.22`, post-fork 직선 `0.30` cap은 유지한다.

## 미션 FSM 현재 상태

### OUT — 이번 작업에서 구현·검증한 생산 경로

`OUT_WAIT_GREEN`
→ `OUT_TO_FORK`
→ `OUT_FORK_SIGN_ADVANCE`
→ `OUT_FORK_COMMIT`
→ `OUT_TO_ARUCO`
→ `OUT_ARUCO_STOP`
→ `OUT_RESUME`
→ `OUT_FINISH_STOP`

- 녹색 신호를 연속 프레임으로 확정한 뒤 출발한다.
- OUT 전체 차선 중심, 곡률, 조향 보정 및 속도 스케줄링은 흰색 마스크만 사용한다. 노란 IN 점선은 OUT 조향에 반영하지 않는다.
- `OUT_TO_FORK`에서는 S자를 별도 미션으로 보지 않고 흰 차선 곡률대로 주행한다.
- 좌/우 표지판을 투표로 확정하면 방향을 고정하고 약 `1.5 s` 전진한다.
  - odometry가 없으므로 요청한 `0.3~0.5 m`를 시간으로 근사한 값이다.
  - 조정 파라미터: `mission.fork_sign_advance_sec`
- X자 상단 ROI에서 선택 방향의 흰색 후보를 찾아 해당 목표점으로 조향한다.
- X자 목표 검출 실패 시 `steering.fork_forced_error=0.45`를 좌/우 부호에 맞춰 사용한다.
- ArUco는 2프레임 연속 검출 시 정지, 3프레임 연속 미검출 시 재출발한다.
- 빨간불 정지는 ArUco 정지 및 재출발 이후에만 허용된다. 주행 중 빨간 물체 오인으로 조기 종료하지 않는다.

### IN — OUT과 분리된 후속 개발 골격

- `IN_WAIT_GREEN`, `IN_ENTRY`, `IN_LAP`, `IN_EXIT`, `IN_ARUCO_STOP`, `IN_RESUME`, `IN_FINISH_STOP`을 OUT과 별도 enum/FSM으로 분리했다.
- 현재 자동 전이는 녹색 신호 → `IN_ENTRY`까지만 활성화했다.
- 다음 작업에서 구현할 항목:
  - 진입 시 노란 점선 검출 및 왼쪽 차선 선택
  - 정지선 검출과 lap count
  - count 임계값 이후 `IN_EXIT` 전이
  - 탈출 시 오른쪽 차선 선택
- 미완성 IN 조건이 OUT 상태로 잘못 넘어가지는 않는다.

## 런타임 및 노드 정리

- 생산 자율주행 명령 발행 경로는 C++ `bisa_autonomous_node` 하나로 통일했다.
- 중복 Python `bisa_autonomous_node` console entry를 제거했다.
- `onboard.launch.py`에서 `battery_node`, `system_telemetry_node`를 제거했다.
- battery source 제거에 맞춰 low-level control의 voltage guard를 명시적으로 비활성화했다.
- `power_gui_node` console entry와 사용하지 않는 `battery`, `monitor` 실행 의존성을 제거했다.
- `viz_node`와 `param_gui_node`는 유지했다.
- rosbag 검증용 `enable_camera` launch argument를 추가했으며 생산 기본값은 `true`다.

## 주요 파라미터

- `lane.out_white_only: true`
- `lane.fork_target_y0: 0.05`
- `lane.fork_target_y1: 0.45`
- `lane.fork_target_min_area_ratio: 0.002`
- `steering.fork_forced_error: 0.45`
- `mission.fork_sign_advance_sec: 1.5`
- `mission.fork_commit_min_sec: 0.8`
- `mission.fork_commit_timeout_sec: 1.8`
- `aruco.confirm_frames: 2`
- `aruco.clear_frames: 3`

## 이번에 수정한 파일

- `src/bisa_cpp/src/bisa_autonomous_node.cpp`
- `src/bisa/config/dracer_params.yaml`
- `src/bisa/launch/onboard.launch.py`
- `src/bisa/setup.py`
- `src/bisa/package.xml`
- `src/bisa/src/dracer_config.py`
- `src/bisa/src/mission_controller.py`
- `src/bisa/src/lane_perception.py`
- `src/bisa/src/param_gui_node.py`
- `src/bisa/test/test_perception_timing.py`

## 검증 결과

- 차량에서 `bisa`, `bisa_cpp` 빌드 성공.
- 최종 C++ 재빌드 성공.
- Python 회귀 시험: `16 passed`.
- `git diff --check` 통과.
- 로컬 OUT bag `rosbag2_2026_07_14-19_00_35`는 카메라 압축 토픽 1개, 749프레임, 약 91.27초다.
- bag을 차량 `/tmp`로 임시 복사하여 비구동 시험했다. 영상 → C++ perception → NCNN detector 경로가 정상 동작했다.
- 해당 bag에서는 초록 신호가 확정되지 않아 FSM은 안전 상태 `OUT_WAIT_GREEN`, throttle `0`에 머물렀다. 전체 OUT 전이 순서는 결정론 회귀 시험으로 검증했다.
- 시험 종료 후 bag player, detector, C++ core, launch를 종료했고 임시 bag도 삭제했다.
- 시험용 ROS 도메인에는 실행 노드가 남지 않았으며 기본 graph 토픽만 확인했다.

### 0.30 차선 추종 재점검

- 도메인 2 비구동 1배속 bag 시험에서 C++ 차선 처리시간은 안정 구간 평균 약 `18~20 ms/frame`이었다.
- 전체 CPU는 측정 순간 약 `31% idle`이었으므로 0.30 추종 불안의 주원인은 CPU 포화보다 20 Hz 제어 지연과 차량 관성 증가로 판단했다.
- 기존 안정값 0.20에서 전 구간을 0.30으로 한 번에 50% 높였던 상태별 cap을 분리했다.
- 최초 현장 권장값은 진입/S/fork 접근 `0.24`, fork commit `0.22`, post-fork 직선 `0.30`이다.
- 이 cap에서도 좌우 진동이 남는 경우에만 Pure Pursuit lookahead/rate를 별도로 조정한다.

## 다음 우선순위

1. 도메인 2에서 실제 트랙 비구동/바퀴 공중 시험으로 OUT 표지판 검출과 X자 목표를 확인한다.
2. 실제 거리 기준으로 `mission.fork_sign_advance_sec`를 조정한다.
3. 좌/우 각각 `fork_target_y0/y1`, `fork_forced_error`, commit 시간을 튜닝한다.
4. OUT 완주 후 IN 점선 선택, 정지선 및 lap count를 구현한다.

## 도메인 2 실행 예시

```bash
cd /home/topst/D-Racer-Kit
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch bisa onboard.launch.py route_mode:=OUT
```

안전한 rosbag 검증 시에는 다음 인자를 추가한다.

```text
enable_camera:=false enable_actuation:=false enable_joystick:=false
```

## Git 상태 메모

- 위 소스 변경은 차량 `clean_code` 브랜치 작업 트리에 있으며 아직 커밋하지 않았다.
- 기존 사용자 변경과 작업 기록을 임의로 reset하거나 삭제하지 않는다.
