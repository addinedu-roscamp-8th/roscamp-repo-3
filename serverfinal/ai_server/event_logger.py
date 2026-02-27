import base64
import json
import logging
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set

import cv2
import httpx


class FireEventLogger:
    def __init__(
        self,
        main_server_event_url: str,
        event_image_dir: Path,
        event_log_file: Path,
        fire_timeout_sec: float = 15.0,
        fire_labels: Optional[Set[str]] = None,
        ashes_labels: Optional[Set[str]] = None,
    ):
        self.main_server_event_url = main_server_event_url
        self.event_image_dir = event_image_dir
        self.event_log_file = event_log_file
        self.fire_timeout_sec = fire_timeout_sec
        self.fire_labels = fire_labels or {"fire"}
        self.ashes_labels = ashes_labels or {"ashes", "ash"}
        self._robot_state: Dict[str, Dict[str, object]] = {}
        self.image_saving_enabled = True  # 이미지 저장 활성화 플래그

    def on_frame_received(self, robot_id: str, received_time: Optional[float] = None):
        state = self._get_or_create_state(robot_id)
        if state["cycle_start_time"] is None:
            state["cycle_start_time"] = received_time if received_time is not None else time.time()

    def handle_detection(
        self,
        robot_id: str,
        is_udp_source: bool,
        now: float,
        seq: int,
        labels: List[str],
        frame_for_log,
        map_coord: Optional[dict] = None,
        fallback_start_time: Optional[float] = None,
    ):
        if not is_udp_source:
            return

        state = self._get_or_create_state(robot_id)
        if state["cycle_start_time"] is None:
            state["cycle_start_time"] = fallback_start_time if fallback_start_time is not None else now
        if map_coord is not None:
            state["last_map_coord"] = map_coord

        fire_detected = any(label in self.fire_labels for label in labels)
        ashes_detected = any(label in self.ashes_labels for label in labels)

        if fire_detected:
            state["fire_detected_in_cycle"] = True
            state["fire_error_logged"] = False
            if state["last_event_type"] != "fire_detected":
                self._save_and_send(
                    robot_id=robot_id,
                    event_type="fire_detected",
                    message="화재 감지",
                    frame=frame_for_log,
                    seq=seq,
                    label="fire",
                )
                state["last_event_type"] = "fire_detected"

        if ashes_detected:
            if state["last_event_type"] != "fire_extinguished":
                self._save_and_send(
                    robot_id=robot_id,
                    event_type="fire_extinguished",
                    message="화재 진압",
                    frame=frame_for_log,
                    seq=seq,
                    label="ashes",
                    map_coord=map_coord if map_coord is not None else state["last_map_coord"],
                )
            state["last_event_type"] = "fire_extinguished"
            state["cycle_start_time"] = now
            state["fire_detected_in_cycle"] = False
            state["fire_error_logged"] = False

        if (
            not state["fire_detected_in_cycle"]
            and state["cycle_start_time"] is not None
            and (now - state["cycle_start_time"]) >= self.fire_timeout_sec
            and not state["fire_error_logged"]
        ):
            self._save_and_send(
                robot_id=robot_id,
                event_type="fire_detection_error",
                message="화재 감지 오류",
                frame=frame_for_log,
                seq=seq,
                label=None,
                map_coord=state["last_map_coord"],
            )
            state["fire_error_logged"] = True
            state["last_event_type"] = "fire_detection_error"

    def read_local_events(self, limit: int = 50) -> List[dict]:
        if not self.event_log_file.exists():
            return []
        rows = []
        try:
            with self.event_log_file.open("r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        rows.append(json.loads(line))
        except Exception as e:
            raise RuntimeError(f"Failed to read event logs: {e}") from e
        return list(reversed(rows[-max(1, min(limit, 500)):]))

    def _get_or_create_state(self, robot_id: str) -> Dict[str, object]:
        if robot_id not in self._robot_state:
            self._robot_state[robot_id] = {
                "cycle_start_time": None,
                "fire_detected_in_cycle": False,
                "fire_error_logged": False,
                "last_event_type": None,
                "last_map_coord": None,
            }
        return self._robot_state[robot_id]

    def _ensure_event_dirs(self):
        self.event_image_dir.mkdir(parents=True, exist_ok=True)
        self.event_log_file.parent.mkdir(parents=True, exist_ok=True)

    def _encode_frame_to_jpeg_base64(self, frame) -> Optional[str]:
        ok, jpeg = cv2.imencode(".jpg", frame)
        if not ok:
            return None
        return base64.b64encode(jpeg.tobytes()).decode("utf-8")

    def set_image_saving(self, enabled: bool):
        """이미지 저장 ON/OFF 설정"""
        self.image_saving_enabled = enabled
        status = "활성화" if enabled else "비활성화"
        logging.info(f"🖼️ AI 이벤트 이미지 저장: {status}")
    
    def get_image_saving_status(self) -> bool:
        """이미지 저장 상태 반환"""
        return self.image_saving_enabled
    
    def _save_and_send(
        self,
        robot_id: str,
        event_type: str,
        message: str,
        frame,
        seq: int,
        label: Optional[str],
        map_coord: Optional[dict] = None,
    ):
        self._ensure_event_dirs()
        now = time.time()
        timestamp_iso = datetime.fromtimestamp(now).isoformat(timespec="seconds")
        
        # 이미지 저장 제어
        image_path = None
        image_saved = False
        if self.image_saving_enabled:
            file_stem = f"{robot_id}_{event_type}_{int(now * 1000)}"
            image_path = self.event_image_dir / f"{file_stem}.jpg"
            image_saved = cv2.imwrite(str(image_path), frame)
        
        image_b64 = self._encode_frame_to_jpeg_base64(frame)

        entry = {
            "robot_id": robot_id,
            "event_type": event_type,
            "message": message,
            "timestamp": timestamp_iso,
            "seq": seq,
            "label": label,
            "map_coord": map_coord,
            "image_path": str(image_path) if image_saved else None,
            "image_b64": image_b64,
        }

        try:
            with self.event_log_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception as e:
            logging.debug(f"Failed to write local event log: {e}")

        try:
            with httpx.Client() as client:
                client.post(self.main_server_event_url, json=entry, timeout=1.0)
        except Exception as e:
            logging.debug(f"Main Server Event Report Error: {e}")
