#!/usr/bin/env python3
"""Benchmark detector variants on exactly the same image set."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_variant(spec: str) -> tuple[str, Path, int, str]:
    """Parse NAME,MODEL,IMGSZ,DEVICE into a benchmark definition."""

    parts = spec.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "variant must be NAME,MODEL,IMGSZ,DEVICE"
        )
    name, model, imgsz, device = parts
    return name, Path(model).expanduser(), int(imgsz), device


def percentile(values: list[float], quantile: float) -> float:
    """Return a simple nearest-rank percentile for a non-empty list."""

    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(quantile * (len(ordered) - 1)))))
    return ordered[index]


def serialize_detections(result) -> list[dict]:
    """Convert one Ultralytics result into JSON-safe detection records."""

    records = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return records
    names = getattr(result, "names", {}) or {}
    for box in boxes:
        class_id = int(box.cls[0].item())
        records.append(
            {
                "class_id": class_id,
                "class_name": str(names.get(class_id, class_id)),
                "confidence": float(box.conf[0].item()),
                "xyxy": [float(value) for value in box.xyxy[0].tolist()],
            }
        )
    return records


def benchmark_variant(variant, images: list[Path], data_yaml: Path | None) -> dict:
    """Warm and measure one model/device/input-size combination."""

    name, model_path, imgsz, device = variant
    if not model_path.exists():
        raise FileNotFoundError(model_path)
    model = YOLO(str(model_path), task="detect" if model_path.is_dir() else None)
    first = cv2.imread(str(images[0]))
    if first is None:
        raise RuntimeError(f"Cannot decode {images[0]}")
    warm_start = time.perf_counter()
    model.predict(first, imgsz=imgsz, device=device, verbose=False)
    warmup_sec = time.perf_counter() - warm_start

    latencies = []
    per_image = []
    for path in images:
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        start = time.perf_counter()
        results = model.predict(frame, imgsz=imgsz, device=device, verbose=False)
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        latencies.append(elapsed_ms)
        detections = serialize_detections(results[0]) if results else []
        per_image.append(
            {"file": str(path), "latency_ms": elapsed_ms, "detections": detections}
        )

    summary = {
        "name": name,
        "model": str(model_path),
        "imgsz": imgsz,
        "device": device,
        "warmup_sec": warmup_sec,
        "images": len(latencies),
        "mean_ms": statistics.fmean(latencies),
        "p50_ms": statistics.median(latencies),
        "p95_ms": percentile(latencies, 0.95),
        "fps": 1000.0 / statistics.fmean(latencies),
        "detections": sum(len(item["detections"]) for item in per_image),
        "per_image": per_image,
    }
    if data_yaml is not None:
        metrics = model.val(
            data=str(data_yaml), imgsz=imgsz, device=device, plots=False, verbose=False
        )
        box = getattr(metrics, "box", None)
        if box is not None:
            summary["validation"] = {
                "map50": float(box.map50),
                "map50_95": float(box.map),
                "precision_per_class": np.asarray(box.p).astype(float).tolist(),
                "recall_per_class": np.asarray(box.r).astype(float).tolist(),
            }
    return summary


def main() -> None:
    """Run all requested variants and write one comparable JSON report."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True, type=Path)
    parser.add_argument("--variant", action="append", type=parse_variant, required=True)
    parser.add_argument("--data-yaml", type=Path)
    parser.add_argument("--output", type=Path, default=Path("detector_benchmark.json"))
    args = parser.parse_args()
    images = sorted(
        path for path in args.images.rglob("*") if path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not images:
        raise SystemExit(f"No images found under {args.images}")
    results = [
        benchmark_variant(variant, images, args.data_yaml) for variant in args.variant
    ]
    args.output.write_text(
        json.dumps({"images_root": str(args.images), "variants": results}, indent=2),
        encoding="utf-8",
    )
    for item in results:
        validation = item.get("validation", {})
        print(
            f"{item['name']}: {item['fps']:.2f} FPS, p95={item['p95_ms']:.1f} ms, "
            f"warmup={item['warmup_sec']:.1f} s, "
            f"mAP50={validation.get('map50', 'n/a')}"
        )
    print(f"report={args.output}")


if __name__ == "__main__":
    main()
