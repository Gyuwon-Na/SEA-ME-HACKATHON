"""ArUco marker detection with OpenCV version-robust API handling."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .dracer_config import AutonomousConfig


@dataclass
class ArucoMarker:
    """One detected ArUco marker in original image coordinates."""

    id: int
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2


def _resolve_dictionary(name: str):
    """Maps a dictionary name string to an OpenCV predefined ArUco dictionary."""

    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        dict_id = getattr(cv2.aruco, "DICT_6X6_50")
    # New API (OpenCV >= 4.7): getPredefinedDictionary; old API: Dictionary_get.
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


class ArucoDetector:
    """Detects ArUco markers, rate-limited, across OpenCV API versions."""

    def __init__(self, config: AutonomousConfig):
        """Builds the dictionary/detector and caches the last detection result."""

        self.config = config
        self._dict_name = config.aruco.dictionary
        self.last_detect_time = 0.0
        self.last_markers: list[ArucoMarker] = []
        self._build_detector()

    def _build_detector(self) -> None:
        """Creates the dictionary, parameters, and (new API) ArucoDetector."""

        self.dictionary = _resolve_dictionary(self.config.aruco.dictionary)
        self._dict_name = self.config.aruco.dictionary
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
            self.params = params
        else:
            self.detector = None
            self.params = cv2.aruco.DetectorParameters_create()

    def _maybe_rebuild(self) -> None:
        """Rebuilds the detector if the configured dictionary changed at runtime."""

        if self.config.aruco.dictionary != self._dict_name:
            self._build_detector()

    def should_run(self, now_sec: float) -> bool:
        """Rate-limits detection so it does not compete with the control loop."""

        hz = max(float(self.config.aruco.detect_hz), 0.1)
        if now_sec - self.last_detect_time < 1.0 / hz:
            return False
        self.last_detect_time = now_sec
        return True

    def detect(self, frame_bgr: np.ndarray, now_sec: float) -> list[ArucoMarker]:
        """Returns detected markers; reuses cached result between rate-limited runs."""

        if not self.config.aruco.enabled:
            self.last_markers = []
            return self.last_markers
        if not self.should_run(now_sec):
            return self.last_markers

        self._maybe_rebuild()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.params
            )

        markers: list[ArucoMarker] = []
        if ids is not None and len(ids) > 0:
            for marker_id, corner in zip(ids.flatten(), corners):
                pts = corner.reshape(4, 2).astype(float)
                x1, y1 = float(pts[:, 0].min()), float(pts[:, 1].min())
                x2, y2 = float(pts[:, 0].max()), float(pts[:, 1].max())
                markers.append(ArucoMarker(id=int(marker_id), bbox=(x1, y1, x2, y2)))
        self.last_markers = markers
        return markers
