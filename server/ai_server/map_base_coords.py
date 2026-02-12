from __future__ import annotations

from pathlib import Path
import time
from typing import Optional

import logging
import cv2
import numpy as np
import yaml
from ultralytics import YOLO


# ----------------------------
# Geometry helpers
# ----------------------------
def pixel_to_map(u: float, v: float, h_matrix: np.ndarray) -> tuple[float, float]:
    src = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, h_matrix)
    return float(dst[0, 0, 0]), float(dst[0, 0, 1])


def map_to_base(x: float, y: float, t_base_from_map: np.ndarray) -> tuple[float, float]:
    p_map_h = np.array([x, y, 0.0, 1.0], dtype=np.float32)
    p_base_h = t_base_from_map @ p_map_h
    return float(p_base_h[0]), float(p_base_h[1])


# ----------------------------
# YAML loaders
# ----------------------------
def load_homography(yaml_path: Path) -> np.ndarray:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "homography" not in data:
        raise ValueError(f"'homography' key not found: {yaml_path}")
    h = np.array(data["homography"], dtype=np.float32)
    if h.shape != (3, 3):
        raise ValueError(f"homography must be (3,3), got {h.shape}")
    return h


def load_base_transform(yaml_path: Path) -> np.ndarray:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "T_base_from_map" not in data:
        raise ValueError(f"'T_base_from_map' key not found: {yaml_path}")
    tf_data = data["T_base_from_map"]
    if not isinstance(tf_data, dict) or "data" not in tf_data:
        raise ValueError(f"'T_base_from_map.data' key not found: {yaml_path}")
    t = np.array(tf_data["data"], dtype=np.float32)
    if t.shape != (4, 4):
        raise ValueError(f"T_base_from_map must be (4,4), got {t.shape}")
    return t


# ----------------------------
# Defaults / utils
# ----------------------------
def default_model_path() -> Path:
    return Path.home() / "workspace" / "fire_detetion" / "results" / "weights" / "best.pt"


def default_homography_path() -> Path:
    # ROS package 탐색 제거
    return Path(__file__).resolve().parents[1] / "config" / "homography_params.yaml"


def default_robot_tf_path() -> Path:
    return Path.home() / "workspace" / "fov_slam_transform" / "config" / "robot_tf_matrix.yaml"


def normalize_path(path_value: str | Path) -> Path:
    return Path(path_value).expanduser().resolve()


def parse_camera_device(device: str):
    # "0" 같은 문자열이면 int(0)로, "/dev/video1"이면 그대로
    if isinstance(device, str) and device.isdigit():
        return int(device)
    return device


def label_from_result(names, cls_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(cls_id, cls_id))
    if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)

def load_map_base_transforms() -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    repo_root = Path(__file__).resolve().parents[2]
    config_dir = repo_root / "fire_detection" / "fire_detection" / "config"
    homography_yaml = config_dir / "homography_params.yaml"
    robot_tf_yaml = config_dir / "robot_tf_matrix.yaml"

    if not homography_yaml.exists() or not robot_tf_yaml.exists():
        logging.warning(
            "map/base transform YAML not found: %s, %s",
            homography_yaml,
            robot_tf_yaml,
        )
        return None, None

    try:
        return load_homography(homography_yaml), load_base_transform(robot_tf_yaml)
    except Exception as exc:
        logging.warning("failed to load map/base transforms: %s", exc)
        return None, None

# ----------------------------
# Main loop
# ----------------------------
def run(
    model_path: Path = default_model_path(),
    camera_device: str = "/dev/video1",
    target_label: str = "fire",
    homography_yaml: Path = default_homography_path(),
    robot_tf_yaml: Path = default_robot_tf_path(),
    fps: float = 10.0,
    confidence_threshold: float = 0.25,
    visualize: bool = False,
):
    model_path = normalize_path(model_path)
    homography_yaml = normalize_path(homography_yaml)
    robot_tf_yaml = normalize_path(robot_tf_yaml)

    if not model_path.exists():
        raise FileNotFoundError(f"model file not found: {model_path}")
    if not homography_yaml.exists():
        raise FileNotFoundError(f"homography yaml not found: {homography_yaml}")
    if not robot_tf_yaml.exists():
        raise FileNotFoundError(f"robot tf yaml not found: {robot_tf_yaml}")

    h_matrix = load_homography(homography_yaml)
    t_base_from_map = load_base_transform(robot_tf_yaml)

    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(parse_camera_device(camera_device))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {camera_device}")
    cap.set(cv2.CAP_PROP_FPS, float(max(0.1, fps)))

    dt = 1.0 / float(max(0.1, fps))
    last_print_t = 0.0

    try:
        while True:
            start_t = time.time()
            ret, frame = cap.read()
            if not ret:
                print("[WARN] failed to read camera frame")
                time.sleep(0.1)
                continue

            result = model(frame, verbose=False)[0]
            annotated_frame = result.plot() if visualize else frame

            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls.item())
                    label = label_from_result(result.names, cls_id)
                    conf = float(box.conf.item()) if box.conf is not None else 0.0
                    if label != target_label or conf < confidence_threshold:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    map_x, map_y = pixel_to_map(cx, cy, h_matrix)
                    base_x, base_y = map_to_base(map_x, map_y, t_base_from_map)

                    # 너무 자주 찍히면 보기 힘드니 1초에 한 번만
                    now = time.time()
                    if now - last_print_t >= 1.0:
                        print(
                            f"[fire conf={conf:.2f}] "
                            f"map: (x={map_x:.3f}, y={map_y:.3f})  |  "
                            f"base: (x={base_x:.3f}, y={base_y:.3f})"
                        )
                        last_print_t = now

                    if visualize:
                        cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                        cv2.putText(
                            annotated_frame,
                            f"map:({map_x:.2f},{map_y:.2f}) base:({base_x:.2f},{base_y:.2f})",
                            (int(cx) + 8, int(cy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            2,
                        )

            if visualize:
                cv2.imshow("top_view_fire", annotated_frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break

            # fps 맞추기
            elapsed = time.time() - start_t
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run(
        # model_path=Path("/path/to/best.pt"),
        homography_yaml=Path("/roscamp-repo-3/fire_detection/fire_detection/config/homography_params.yaml"),
        robot_tf_yaml=Path("/roscamp-repo-3/fire_detection/fire_detection/config/robot_tf_matrix.yaml"),
        camera_device="/dev/video0",
        target_label="fire",
        fps=10.0,
        confidence_threshold=0.25,
        visualize=False,
    )
