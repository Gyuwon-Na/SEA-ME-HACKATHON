# YOLO 객체인식 GPU(Vulkan) 가속 마이그레이션

## 배경

- **TOPST D3-G (TCC8050)** 보드에 **PowerVR GT9524 GPU** 탑재 (168 GFLOPS, Vulkan 1.2 지원)
- 기존: NCNN 모델을 **CPU-only**로 추론 → A72 코어 4개 중 일부를 YOLO가 독점, 차선/제어 루프에 영향
- 목표: **Vulkan GPU**로 YOLO 추론을 오프로드 → CPU는 차선/제어에 전념

## 수정 파일 (2개)

### 1. [object_detector.py](file:///home/hyun/D-Racer-Kit/src/bisa/src/object_detector.py)

| 변경 위치 | 내용 |
|---|---|
| [`_resolve_device()`](file:///home/hyun/D-Racer-Kit/src/bisa/src/object_detector.py#L80-L116) | `vulkan`, `vulkan:0`, `vulkan:N` 디바이스 지원 추가. `auto` 모드에서 CUDA → Vulkan(NCNN일 때) → CPU 순으로 자동 선택 |
| [`_is_ncnn_model()`](file:///home/hyun/D-Racer-Kit/src/bisa/src/object_detector.py#L124-L132) | 신규 메서드. 모델 경로가 NCNN export인지 판별 (디렉토리 + `model.ncnn.param` 존재) |
| [`load_model()`](file:///home/hyun/D-Racer-Kit/src/bisa/src/object_detector.py#L134-L166) | `torch` import를 조건부로 변경 (NCNN-only 환경에서 torch 없어도 동작). 로그에 백엔드(NCNN/PyTorch) 표시 |

```diff
-    def _resolve_device(self) -> str:
-        """Picks the inference device, auto-detecting a CUDA GPU on the PC."""
-        ...
-        if preference in ("cuda", "gpu", "0", "cuda:0"):
-            return "cuda:0"
-        ...
-        return "cpu"
+    def _resolve_device(self) -> str:
+        """Picks the inference device: CUDA GPU, Vulkan GPU, or CPU."""
+        ...
+        if preference.startswith("vulkan"):
+            return preference if ":" in preference else "vulkan:0"
+        ...
+        if self._is_ncnn_model():
+            return "vulkan:0"
+        return "cpu"
```

### 2. [onboard.launch.py](file:///home/hyun/D-Racer-Kit/src/bisa/launch/onboard.launch.py)

| 변경 위치 | 이전 | 이후 |
|---|---|---|
| `detector.device` | `"cpu"` | `"vulkan:0"` |
| `inference_hz` | `4.0` | `8.0` (GPU가 더 빠름) |
| `sign_vote_k/n` | `2/3` (4Hz 기준) | `3/5` (8Hz 기준, 벽시계 ~0.625s 유지) |
| 주석/docstring | CPU-only 설명 | Vulkan GPU 가속 설명 |

## 기술적 핵심

```
기존 흐름:
  Camera → NCNN model.predict(device="cpu") → A72 CPU 독점 (~67ms/frame)
  
변경 후:
  Camera → NCNN model.predict(device="vulkan:0") → PowerVR GPU 오프로드
  CPU 코어는 차선인식/제어 루프에 전념
```

> [!IMPORTANT]
> TOPST D3-G의 Ubuntu 22.04 이미지에 **Vulkan 드라이버가 설치되어 있어야** 합니다.
> 만약 Vulkan 드라이버가 없으면 `onboard.launch.py`에서 `device:=cpu`로 오버라이드하면 기존 CPU 모드로 폴백됩니다:
> ```bash
> ros2 launch bisa onboard.launch.py device:=cpu inference_hz:=4.0
> ```

## 검증 결과

| 항목 | 결과 |
|---|---|
| Python 문법 체크 (`py_compile`) | ✅ OK |
| bisa 모듈 import 테스트 | ✅ OK |
| `_resolve_device()` 단위 테스트 (cpu/cuda/vulkan/vulkan:0/vulkan:1/auto) | ✅ 모두 통과 |
| `_is_ncnn_model()` 단위 테스트 (NCNN dir / best.pt) | ✅ 모두 통과 |
| `colcon build --packages-select bisa` | ✅ 성공 |
| `ros2 launch bisa onboard.launch.py --print-description` | ✅ 6개 노드 정상 등록 |

## 패키지 구조 영향

> [!NOTE]
> 패키지 구조 **변경 없음**. 기존 파일 2개만 수정, 신규 파일 0개.
> - `setup.py`, `package.xml`: 변경 없음
> - 다른 노드 (camera, control, joystick, battery, monitor 등): 변경 없음
> - `dracer_params.yaml`: 변경 없음 (launch에서 오버라이드)

## 차량 보드에서 확인할 사항

1. `vulkaninfo --summary` 로 Vulkan 드라이버 인식 확인
2. 실 주행 시 로그의 `YOLO infer: N ms/frame, M FPS effective (device=vulkan:0)` 확인
3. GPU 열 관리 — 서멀 스로틀링 발생 시 `inference_hz` 낮추기
