from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from ultralytics import YOLO


# Convert pixel coordinates (u, v) to map coordinates using homography matrix
def pixel_to_map(u: float, v: float, h_matrix: np.ndarray) -> tuple[float, float]:
    src = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, h_matrix)
    return float(dst[0, 0, 0]), float(dst[0, 0, 1])


# Load homography matrix from a YAML file
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


# Get default homography YAML path from package share directory
def default_homography_path() -> str:
    try:
        share_dir = Path(get_package_share_directory("top_view_fire"))
        return str(share_dir / "config" / "homography_params.yaml")
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "config" / "homography_params.yaml")


def default_model_path() -> str:
    return str(Path.home() / "workspace" / "fire_detetion" / "results" / "weights" / "best.pt")


def default_robot_tf_path() -> str:
    return str(Path.home() / "workspace" / "fov_slam_transform" / "config" / "robot_tf_matrix.yaml")


def normalize_path(path_value: str) -> Path:
    return Path(path_value).expanduser().resolve()


def map_to_base(x: float, y: float, t_base_from_map: np.ndarray) -> tuple[float, float]:
    p_map_h = np.array([x, y, 0.0, 1.0], dtype=np.float32)
    p_base_h = t_base_from_map @ p_map_h
    return float(p_base_h[0]), float(p_base_h[1])


# Fire Map Publisher Node
class FireMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fire_map_publisher")

        self.declare_parameter(
            "model_path",
            default_model_path(),
        )
        self.declare_parameter("camera_device", "/dev/video1")
        self.declare_parameter("target_label", "fire")
        self.declare_parameter("homography_yaml", default_homography_path())
        self.declare_parameter("robot_tf_yaml", default_robot_tf_path())
        self.declare_parameter("topic_name", "/top_view/fire_base_point")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("visualize", False)

        model_path = normalize_path(str(self.get_parameter("model_path").value))
        camera_device = self.get_parameter("camera_device").value
        homography_yaml = normalize_path(str(self.get_parameter("homography_yaml").value))
        robot_tf_yaml = normalize_path(str(self.get_parameter("robot_tf_yaml").value))
        self.target_label = self.get_parameter("target_label").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.visualize = bool(self.get_parameter("visualize").value)
        fps = max(0.1, float(self.get_parameter("fps").value))
        base_topic_name = self.get_parameter("topic_name").value

        if not model_path.exists():
            raise FileNotFoundError(f"model file not found: {model_path}")
        if not homography_yaml.exists():
            raise FileNotFoundError(f"homography yaml not found: {homography_yaml}")
        if not robot_tf_yaml.exists():
            raise FileNotFoundError(f"robot tf yaml not found: {robot_tf_yaml}")

        self.h_matrix = load_homography(homography_yaml)
        self.t_base_from_map = load_base_transform(robot_tf_yaml)
        self.model = YOLO(str(model_path))
        self.cap = cv2.VideoCapture(self._parse_camera_device(camera_device))
        if not self.cap.isOpened():
            raise RuntimeError(f"failed to open camera: {camera_device}")
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.base_pub = self.create_publisher(PointStamped, base_topic_name, 10)
        self.timer = self.create_timer(1.0 / fps, self._timer_cb)
        self.get_logger().info(f"publishing fire base points to {base_topic_name}")

    @staticmethod
    def _parse_camera_device(device):
        if isinstance(device, str) and device.isdigit():
            return int(device)
        return device

    @staticmethod
    def _label_from_result(names, cls_id: int) -> str:
        if isinstance(names, dict):
            return str(names.get(cls_id, cls_id))
        if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
            return str(names[cls_id])
        return str(cls_id)

    def _timer_cb(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("failed to read camera frame")
            return

        result = self.model(frame, verbose=False)[0]
        annotated_frame = result.plot() if self.visualize else frame

        if result.boxes is None:
            self._show_frame_if_needed(annotated_frame)
            return

        for box in result.boxes:
            cls_id = int(box.cls.item())
            label = self._label_from_result(result.names, cls_id)
            conf = float(box.conf.item()) if box.conf is not None else 0.0
            if label != self.target_label or conf < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            map_x, map_y = pixel_to_map(cx, cy, self.h_matrix)
            base_x, base_y = map_to_base(map_x, map_y, self.t_base_from_map)
            self._publish_point(self.base_pub, self.base_frame_id, base_x, base_y)

            if self.visualize:
                cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.putText(
                    annotated_frame,
                    f"base: ({base_x:.2f}, {base_y:.2f})",
                    (int(cx) + 8, int(cy) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )

        self._show_frame_if_needed(annotated_frame)

    def _publish_point(self, publisher, frame_id: str, x: float, y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        publisher.publish(msg)
        self.get_logger().info(
            f"fire point published [{frame_id}]: x={x:.3f}, y={y:.3f}",
            throttle_duration_sec=1.0,
        )

    def _show_frame_if_needed(self, frame):
        if not self.visualize:
            return
        cv2.imshow("top_view_fire", frame)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FireMapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
