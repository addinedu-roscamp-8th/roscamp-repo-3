from pathlib import Path
from typing import Optional, Tuple
import logging

import cv2
import numpy as np

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


def pixel_to_map(u: float, v: float, h_matrix: np.ndarray) -> Tuple[float, float]:
    src = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, h_matrix)
    return float(dst[0, 0, 0]), float(dst[0, 0, 1])


def map_to_base(x: float, y: float, t_base_from_map: np.ndarray) -> Tuple[float, float]:
    p_map_h = np.array([x, y, 0.0, 1.0], dtype=np.float32)
    p_base_h = t_base_from_map @ p_map_h
    return float(p_base_h[0]), float(p_base_h[1])


def _load_homography(yaml_path: Path) -> np.ndarray:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    h = np.array(data["homography"], dtype=np.float32)
    if h.shape != (3, 3):
        raise ValueError(f"homography must be (3,3), got {h.shape}")
    return h


def _load_base_transform(yaml_path: Path) -> np.ndarray:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    t = np.array(data["T_base_from_map"]["data"], dtype=np.float32)
    if t.shape != (4, 4):
        raise ValueError(f"T_base_from_map must be (4,4), got {t.shape}")
    return t


def load_map_base_transforms() -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    if yaml is None:
        logging.warning("PyYAML is not installed. map/base transform is disabled.")
        return None, None

    config_dir = Path(__file__).resolve().parent / "config"
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
        return _load_homography(homography_yaml), _load_base_transform(robot_tf_yaml)
    except Exception as exc:
        logging.warning("failed to load map/base transforms: %s", exc)
        return None, None
