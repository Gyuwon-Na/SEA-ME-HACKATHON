import os
import subprocess
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
import yaml


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/D-Racer/src/config/vehicle_config.yaml'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('publish_topic', 'camera/image/compressed')
        self.declare_parameter('publish_hz', 30.0)
        # USB 카메라 디바이스 (vehicle_config.yaml의 USB_CAM_DEVICE로 오버라이드됨)
        self.declare_parameter('camera_device', '/dev/video1')
        self.declare_parameter('jpeg_quality', 90)
        # USB 카메라가 MJPG를 하드웨어 압축으로 내보내므로
        # 패스스루가 기본값: decode→re-encode CPU 낭비 없음
        self.declare_parameter('mjpg_passthrough', True)
        self.declare_parameter('debug_log', False)
        # raw BGR Image 추가 발행 (기본 OFF — MJPG 패스스루 시 DDS 30× 증가)
        self.declare_parameter('publish_raw_image', False)
        self.declare_parameter('raw_image_topic', 'camera/image/raw')
        # V4L2 카메라 컨트롤 고정 여부 (자율주행 시 ON 권장)
        self.declare_parameter('lock_camera_controls', True)

        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        publish_topic = str(self.get_parameter('publish_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')
        camera_device = str(self.get_parameter('camera_device').value)
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not 0 <= jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in range [0, 100]')
        self.debug_log = bool(self.get_parameter('debug_log').value)
        self.mjpg_passthrough = bool(self.get_parameter('mjpg_passthrough').value)
        self.publish_raw_image = bool(self.get_parameter('publish_raw_image').value)
        raw_image_topic = str(self.get_parameter('raw_image_topic').value)
        self.lock_camera_controls = bool(self.get_parameter('lock_camera_controls').value)
        self.publish_hz = publish_hz
        self.jpeg_quality = jpeg_quality
        self.passthrough = False

        self.image_width, self.image_height = self._load_image_size()
        # vehicle_config.yaml의 USB_CAM_DEVICE가 있으면 우선 적용
        self.camera_device = self._load_camera_device(camera_device)

        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher_ = self.create_publisher(CompressedImage, publish_topic, self.image_qos)
        self.raw_publisher_ = (
            self.create_publisher(Image, raw_image_topic, self.image_qos)
            if self.publish_raw_image else None
        )

        self.cap = None
        self.pipeline = None
        if not self.open_capture():
            raise RuntimeError(
                f'Failed to open USB camera '
                f'(device={self.camera_device}, '
                f'{self.image_width}x{self.image_height})'
            )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        self.get_logger().info(
            f'[Camera Node] topic={publish_topic} | '
            f'device={self.camera_device} | '
            f'{self.image_width}x{self.image_height} | '
            f'passthrough={self.passthrough} | '
            f'raw={self.publish_raw_image}'
        )

    # -------------------------------------------------------------------------

    def _load_image_size(self):
        default = (640, 480)
        cfg = self._read_yaml()
        if cfg is None:
            return default
        return (
            int(cfg.get('IMAGE_WIDTH', default[0])),
            int(cfg.get('IMAGE_HEIGHT', default[1])),
        )

    def _load_camera_device(self, default: str) -> str:
        cfg = self._read_yaml()
        if cfg is None:
            return default
        return str(cfg.get('USB_CAM_DEVICE', default)).strip() or default

    def _read_yaml(self):
        if not os.path.exists(self.vehicle_config_file):
            return None
        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warning(f'vehicle_config 읽기 실패: {exc}')
            return None

    # -------------------------------------------------------------------------

    def _apply_camera_controls(self):
        """자율주행용 V4L2 컨트롤 고정.

        - focus_auto OFF         : 포커스 안정화, 처리 지연 제거
        - exposure_auto MANUAL   : 노출 고정 → 밝기 불일치 방지
        - exposure_auto_priority OFF : FPS 드롭 방지 (어두울 때 24→7fps 현상)
        - white_balance_auto OFF : 색 기반 차선/신호등 검출 일관성 확보
        """
        if not self.lock_camera_controls:
            return
        device = self.camera_device

        # OpenCV 레벨로 설정 가능한 것들
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # focus_auto OFF
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1=manual (OpenCV V4L2)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)         # white_balance_auto OFF

        # exposure_auto_priority는 OpenCV 표준 프로퍼티 없음 → v4l2-ctl 사용
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', device,
                 '--set-ctrl=exposure_auto_priority=0'],
                check=True, capture_output=True, timeout=3,
            )
            self.get_logger().info(
                'V4L2 컨트롤 고정: focus_auto=0, exposure_auto=manual, '
                'exposure_auto_priority=0, white_balance_auto=0'
            )
        except FileNotFoundError:
            self.get_logger().warning(
                'v4l2-ctl 없음 — exposure_auto_priority 고정 실패 '
                '(FPS 드롭 발생 가능). apt install v4l-utils 권장.'
            )
        except Exception as exc:
            self.get_logger().warning(f'exposure_auto_priority 설정 실패: {exc}')

    # -------------------------------------------------------------------------

    def _build_usb_pipelines(self):
        """USB 카메라용 GStreamer 파이프라인 후보 목록."""
        # MJPG → BGR 파이프라인 (USB 카메라 대부분 지원)
        mjpg = (
            f"v4l2src device={self.camera_device} io-mode=2 ! "
            "image/jpeg,framerate=30/1 ! jpegdec ! "
            "videoconvert ! videoscale ! "
            f"video/x-raw,format=BGR,"
            f"width={self.image_width},height={self.image_height},framerate=30/1 ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        # YUYV 폴백 (MJPG 불가 시)
        raw = (
            f"v4l2src device={self.camera_device} io-mode=2 ! "
            "videoconvert ! videoscale ! "
            f"video/x-raw,format=BGR,"
            f"width={self.image_width},height={self.image_height},framerate=30/1 ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        return [mjpg, raw]

    def open_capture(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.cap = None
        self.passthrough = False

        # MJPG 패스스루 시도 (USB 카메라 하드웨어 JPEG → 디코딩 없이 발행)
        if self.mjpg_passthrough:
            if self.open_mjpg_passthrough():
                return True
            self.get_logger().warning(
                'MJPG 패스스루 불가 → GStreamer 디코드 파이프라인으로 폴백'
            )

        for pipeline in self._build_usb_pipelines():
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.cap = cap
                self.pipeline = pipeline
                self._apply_camera_controls()
                self.get_logger().info(f'카메라 열림 (GStreamer): {pipeline}')
                return True
            cap.release()
            self.get_logger().warning(f'파이프라인 실패: {pipeline}')

        self.cap = None
        self.pipeline = None
        return False

    def open_mjpg_passthrough(self):
        """USB 카메라를 MJPG 패스스루로 열기.

        카메라가 하드웨어로 압축한 JPEG을 그대로 발행하므로
        encode/decode CPU 비용이 없음. 프레임이 유효한 JPEG이고
        설정 해상도와 일치할 때만 True 반환.
        """
        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
        cap.set(cv2.CAP_PROP_FPS, 30.0)
        # 0 = 원시 JPEG 버퍼를 그대로 반환 (디코딩 안 함)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)

        validated = False
        for _ in range(5):
            ret, buf = cap.read()
            if not ret or buf is None:
                continue
            raw = np.asarray(buf, dtype=np.uint8).reshape(-1)
            # JPEG SOI 마커 확인
            if raw.size < 2 or raw[0] != 0xFF or raw[1] != 0xD8:
                break
            decoded = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if decoded is None:
                break
            h, w = decoded.shape[:2]
            if (w, h) != (self.image_width, self.image_height):
                self.get_logger().warning(
                    f'MJPG 해상도 불일치 {w}x{h} != '
                    f'{self.image_width}x{self.image_height}'
                )
                break
            validated = True
            break

        if not validated:
            cap.release()
            return False

        self.cap = cap
        self.passthrough = True
        self.pipeline = f'MJPG passthrough ({self.camera_device})'
        # 패스스루 모드에서도 V4L2 컨트롤 고정 (같은 cap 객체에 적용)
        self._apply_camera_controls()
        self.get_logger().info(f'카메라 열림 (MJPG 패스스루): {self.camera_device}')
        return True

    # -------------------------------------------------------------------------

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warning('카메라 캡처가 열려 있지 않음')
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning('프레임 읽기 실패')
            return

        stamp = self.get_clock().now().to_msg()

        if self.passthrough:
            # 패스스루: frame이 이미 JPEG 버퍼
            data = np.asarray(frame, dtype=np.uint8).tobytes()
        else:
            success, encoded = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not success:
                self.get_logger().warning('JPEG 인코딩 실패')
                return
            data = encoded.tobytes()

        # CompressedImage 발행
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = data
        self.publisher_.publish(msg)

        # raw BGR Image 추가 발행 (옵션, 기본 OFF)
        if self.raw_publisher_ is not None:
            bgr = cv2.imdecode(
                np.asarray(frame, dtype=np.uint8).reshape(-1), cv2.IMREAD_COLOR
            ) if self.passthrough else frame
            if bgr is not None:
                raw_msg = Image()
                raw_msg.header.stamp = stamp
                raw_msg.header.frame_id = 'camera'
                raw_msg.height = bgr.shape[0]
                raw_msg.width = bgr.shape[1]
                raw_msg.encoding = 'bgr8'
                raw_msg.is_bigendian = 0
                raw_msg.step = bgr.shape[1] * 3
                raw_msg.data = bgr.tobytes()
                self.raw_publisher_.publish(raw_msg)

        if self.debug_log:
            self.get_logger().info(f'프레임 발행: {len(data)} bytes')

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                self.cap = None
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
