from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from typing import List, Optional, Dict
import numpy as np
import time
import socket
import threading
import cv2
import json
import logging
import httpx
from pathlib import Path
from ultralytics import YOLO
from event_logger import FireEventLogger
from coord_transform import load_map_base_transforms, pixel_to_map, map_to_base
from gui_alert_overlay import GuiAlertOverlay

# --- Configuration ---
MAIN_SERVER_IP = "192.168.0.30"
MAIN_SERVER_URL = f"http://{MAIN_SERVER_IP}:5000/api/ai/coordinates"
MAIN_SERVER_EVENT_URL = f"http://{MAIN_SERVER_IP}:5000/api/ai/event-log"
USB_CAMERA_INDEX = 2  # USB Camera index
EVENT_IMAGE_DIR = Path(__file__).resolve().parent / "event_logs" / "images"
EVENT_LOG_FILE = Path(__file__).resolve().parent / "event_logs" / "events.jsonl"
GUI_PC_IP = "192.168.0.13"
GUI_UDP_PORT = 5930
GUI_JPEG_QUALITY = 70
GUI_FIRE_TRIGGER_LABELS = {"fire"}
GUI_EXTINGUISH_TRIGGER_LABELS = {"ashes", "ash"}


# Local camera crop settings (pixels)
CROP_TOP = 50
CROP_BOTTOM = 145
CROP_LEFT = 40
CROP_RIGHT = 20

# Local camera flip settings
CAMERA_FLIP_HORIZONTAL = True
CAMERA_FLIP_VERTICAL = True

app = FastAPI(title="LOVO Multi-Robot AI Analysis Server")

# --- Model & Robot Configuration ---
ROBOT_CONFIG = {
    "robot1": {"port": 9510, "obs_port": 9512, "name": "컴1 영상(네트워크)"},
    "local":  {"port": None, "obs_port": None, "name": "컴2 영상(USB)"},
}

# --- State Management ---
class RobotState:
    def __init__(self, robot_id, config):
        self.robot_id = robot_id
        self.name = config["name"]
        self.port = config["port"]
        self.is_udp_source = self.port is not None
        
        self.latest_frame = None
        self.processed_frame = None
        self.inference_result = {"status": "Waiting for data"}
        self.lock = threading.Lock()
        
        # Performance Metrics
        self.receive_count = 0
        self.inference_count = 0
        self.receive_fps = 0.0
        self.inference_fps = 0.0
        self.latency_ms = 0.0
        
        self.last_receive_time = time.time()
        self.last_inference_time = time.time()
        self.fps_calc_time = time.time()

        # Tracking & Architecture Update
        self.last_addr = None
        self.seq_counter = 0
        self.tracking_state = "SEARCH"
        self.last_target = None  # {track_id, conf, cx, cy, bbox}
        self.last_track_time = 0
        self.width = 0
        self.height = 0

    def update_receive_stats(self):
        self.receive_count += 1
        self.last_receive_time = time.time()
        now = time.time()
        if now - self.fps_calc_time > 1.0:
            self.receive_fps = self.receive_count / (now - self.fps_calc_time)
            self.inference_fps = self.inference_count / (now - self.fps_calc_time)
            self.receive_count = 0
            self.inference_count = 0
            self.fps_calc_time = now

class RobotManager:
    def __init__(self, configs):
        self.robots: Dict[str, RobotState] = {
            rid: RobotState(rid, cfg) for rid, cfg in configs.items()
        }
        # Load single specific model
        self.model = YOLO("models/best_fire_det.pt")
        self.h_matrix, self.t_base_from_map = load_map_base_transforms()
        logging.info("YOLO Model (best_fire_det.pt) loaded successfully.")

    def get_robot(self, robot_id) -> Optional[RobotState]:
        return self.robots.get(robot_id)

manager = RobotManager(ROBOT_CONFIG)

# --- Event Logger ---
event_logger = FireEventLogger(
    main_server_event_url=MAIN_SERVER_EVENT_URL,
    event_image_dir=EVENT_IMAGE_DIR,
    event_log_file=EVENT_LOG_FILE,
    fire_timeout_sec=15.0,
)

gui_alert_overlay = GuiAlertOverlay(
    fire_labels={"fire"},
    ashes_labels={"ashes", "ash"},
    blink_hz=2.0,
    extinguish_text_duration_sec=5.0,
)

def _apply_local_camera_adjustments(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    start_y = max(0, CROP_TOP)
    end_y = h - max(0, CROP_BOTTOM)
    start_x = max(0, CROP_LEFT)
    end_x = w - max(0, CROP_RIGHT)

    if start_y < end_y and start_x < end_x:
        frame = frame[start_y:end_y, start_x:end_x]

    if CAMERA_FLIP_HORIZONTAL and CAMERA_FLIP_VERTICAL:
        frame = cv2.flip(frame, -1)
    elif CAMERA_FLIP_HORIZONTAL:
        frame = cv2.flip(frame, 1)
    elif CAMERA_FLIP_VERTICAL:
        frame = cv2.flip(frame, 0)
    return frame


# --- UDP Receiver Task ---
def udp_receiver_task(robot_state: RobotState):
    """Independent UDP Receiver. Also forwards RAW frame to GUI."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # GUI 전송용

    try:
        sock.bind(("0.0.0.0", robot_state.port))
        sock.settimeout(1.0)
        logging.info(f"Receiver started for {robot_state.robot_id} on port {robot_state.port}")
    except Exception as e:
        logging.error(f"Error binding port {robot_state.port}: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(65507)
            # 1. 받은 그대로 GUI에 즉시 토스 (컴1 원본)
            if robot_state.robot_id == "robot1":
                gui_sock.sendto(data, (GUI_PC_IP, GUI_PORT_COMP1_RAW))

            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                with robot_state.lock:
                    robot_state.latest_frame = frame
                    robot_state.last_addr = addr[0]
                    robot_state.height, robot_state.width = frame.shape[:2]
                robot_state.update_receive_stats()
        except socket.timeout:
            continue
        except Exception as e:
            logging.debug(f"UDP Error ({robot_state.robot_id}): {e}")

# --- USB Camera Task ---
def usb_camera_task(robot_state: RobotState):
    """Captures frames from local USB camera."""
    cap = cv2.VideoCapture(USB_CAMERA_INDEX)
    if not cap.isOpened():
        logging.error(f"Could not open USB camera at index {USB_CAMERA_INDEX}")
        return
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    logging.info(f"USB Camera started for {robot_state.robot_id}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.warning("Failed to grab frame from USB camera")
            time.sleep(0.1)
            continue
        frame = _apply_local_camera_adjustments(frame)

        with robot_state.lock:
            robot_state.latest_frame = frame
            robot_state.height, robot_state.width = frame.shape[:2]

        # GUI로 원본 전송 (컴2 USB -> GUI)
        try:
            ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), GUI_JPEG_QUALITY])
            if ok:
                gui_sock.sendto(jpeg.tobytes(), (GUI_PC_IP, GUI_PORT_USB_RAW))
        except Exception as e:
            logging.debug(f"USB GUI RAW stream error: {e}")
        
        robot_state.update_receive_stats()
        # Small sleep to prevent high CPU usage if camera has high FPS
        time.sleep(0.01)

    cap.release()
    gui_sock.close()

def _labels_from_result(result) -> List[str]:
    if not result or result.boxes is None or result.boxes.cls is None:
        return []
    cls_ids = result.boxes.cls.int().cpu().tolist()
    names = result.names if hasattr(result, "names") else {}
    labels = []
    for cid in cls_ids:
        label = names.get(cid, str(cid))
        labels.append(str(label).lower())
    return labels

# --- Inference Worker Task ---
def inference_worker_task():
    """Central scheduler for inference with tracking and UDP broadcast."""
    INFERENCE_FPS_CAP = 10.0
    MIN_INTERVAL = 1.0 / INFERENCE_FPS_CAP
    
    # UDP Socket for broadcasting observations
    broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        for robot_id, state in manager.robots.items():
            now = time.time()
            if now - state.last_inference_time < MIN_INTERVAL:
                continue

            frame_to_process = None
            with state.lock:
                if state.latest_frame is not None:
                    frame_to_process = state.latest_frame.copy()
            
            if frame_to_process is not None:
                start_time = time.time()
                obs_msg = {
                    "robot_id": state.robot_id,
                    "t": round(now, 3),
                    "seq": state.seq_counter,
                    "img": {"W": state.width, "H": state.height},
                    "state": state.tracking_state,
                    "target": state.last_target
                }
                try:
                    # 1. YOLO Inference with Tracking
                    results = manager.model.track(frame_to_process, persist=True, verbose=False, conf=0.3)

                    target_data = None
                    labels = []
                    event_map_coord = None
                    if results and len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
                        boxes = results[0].boxes
                        labels = _labels_from_result(results[0])
                        track_ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [None] * len(boxes)
                        confs = boxes.conf.cpu().tolist()
                        xyxy = boxes.xyxy.cpu().tolist()

                        idx = 0 
                        x1, y1, x2, y2 = xyxy[idx]
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0

                        local_map_coord = None
                        local_base_coord = None
                        if manager.h_matrix is not None and manager.t_base_from_map is not None:
                            try:
                                map_x, map_y = pixel_to_map(cx, cy, manager.h_matrix)
                                event_map_coord = {"frame_id": "map", "x": round(map_x, 3), "y": round(map_y, 3)}
                                if robot_id == "local":
                                    base_x, base_y = map_to_base(map_x, map_y, manager.t_base_from_map)
                                    local_map_coord = event_map_coord
                                    local_base_coord = {
                                        "frame_id": "base_link",
                                        "x": round(base_x, 3),
                                        "y": round(base_y, 3),
                                    }
                            except Exception as e:
                                logging.debug(f"Coord Transform Error ({robot_id}): {e}")

                        target_data = {
                            "track_id": track_ids[idx],
                            "conf": round(confs[idx], 2),
                            "cx": round(cx, 1),
                            "cy": round(cy, 1),
                            "bbox": [int(x1), int(y1), int(x2), int(y2)],
                            "map_coord": local_map_coord,
                            "base_coord": local_base_coord,
                        }
                        
                        state.tracking_state = "TRACK"
                        state.last_target = target_data
                        state.last_track_time = now
                        processed_frame = results[0].plot()
                    else:
                        # State Logic: TRACK -> LOST -> SEARCH
                        if state.tracking_state == "TRACK":
                            state.tracking_state = "LOST"
                        
                        if state.tracking_state == "LOST" and (now - state.last_track_time > 1.0):
                            state.tracking_state = "SEARCH"
                            state.last_target = None

                        target_data = state.last_target
                        processed_frame = frame_to_process.copy()

                    # 2. Build Observation Message
                    state.seq_counter += 1
                    obs_msg = {
                        "robot_id": state.robot_id,
                        "t": round(now, 3),
                        "seq": state.seq_counter,
                        "img": {"W": state.width, "H": state.height},
                        "state": state.tracking_state,
                        "target": target_data
                    }

                    gui_labels: List[str] = []
                    if robot_id == "local":
                        gui_labels = [label for label in labels if label in GUI_FIRE_TRIGGER_LABELS]
                    elif state.is_udp_source:
                        gui_labels = [label for label in labels if label in GUI_EXTINGUISH_TRIGGER_LABELS]
                    if gui_labels:
                        gui_alert_overlay.update_from_labels(gui_labels, now)

                    # 2-1. Fire Event Logging (UDP sources only)
                    event_logger.handle_detection(
                        robot_id=state.robot_id,
                        is_udp_source=state.is_udp_source,
                        now=now,
                        seq=state.seq_counter,
                        labels=labels,
                        frame_for_log=processed_frame,
                        map_coord=event_map_coord,
                        fallback_start_time=state.last_receive_time,
                    )

                    # 3. Broadcast over UDP
                    if state.last_addr:
                        target_port = ROBOT_CONFIG[robot_id]["obs_port"]
                        try:
                            msg_json = json.dumps(obs_msg).encode('utf-8')
                            broadcast_sock.sendto(msg_json, (state.last_addr, target_port))
                            if state.seq_counter % 100 == 1:
                                logging.debug(f"Broadcast sent to {robot_id} ({state.last_addr}:{target_port})")
                        except Exception as e:
                            logging.debug(f"Broadcast Error ({robot_id}): {e}")

                    # 4. Report to Main Server
                    # local only: send map/base coordinates to main
                    if (
                        target_data
                        and robot_id == "local"
                        and target_data.get("map_coord") is not None
                        and target_data.get("base_coord") is not None
                    ):
                        try:
                            # Using synchronous httpx call or a thread-safe way
                            # For simplicity in this threaded environment:
                            with httpx.Client() as client:
                                client.post(MAIN_SERVER_URL, json=obs_msg, timeout=0.1)
                        except Exception as e:
                            # Don't let reporting errors crash the inference loop
                            if state.seq_counter % 100 == 1:
                                logging.debug(f"Main Server Report Error: {e}")

                except Exception as e:
                    logging.debug(f"Inference Error on {robot_id}: {e}")
                    processed_frame = frame_to_process.copy()

                # [추가] 가공된 영상(Overlay)을 GUI 전송 (컴1 가공본 전용)
                if robot_id == "robot1" and processed_frame is not None:
                    try:
                        ok, jpeg = cv2.imencode(".jpg", processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), GUI_JPEG_QUALITY])
                        if ok:
                            broadcast_sock.sendto(jpeg.tobytes(), (GUI_PC_IP, GUI_PORT_COMP1_OVERLAY))
                    except Exception as e:
                        logging.debug(f"Overlay Stream Error: {e}")

                with state.lock:
                    state.processed_frame = processed_frame
                    state.inference_result = obs_msg 
                
                state.inference_count += 1
                state.latency_ms = (time.time() - start_time) * 1000
                state.last_inference_time = time.time()
        
        time.sleep(0.01)

# --- App Lifecycle ---
@app.on_event("startup")
async def startup_event():
    # Start receivers for all robots
    for robot_id, state in manager.robots.items():
        if robot_id == "local":
            t = threading.Thread(target=usb_camera_task, args=(state,), daemon=True)
        else:
            t = threading.Thread(target=udp_receiver_task, args=(state,), daemon=True)
        t.start()
    
    # Start inference scheduler
    t_inf = threading.Thread(target=inference_worker_task, daemon=True)
    t_inf.start()

# --- API Routes ---
def generate_mjpeg_stream(robot_id, use_overlay=False):
    """Generator for MJPEG stream (Raw or Processed)."""
    while True:
        state = manager.get_robot(robot_id)
        if not state: break
        
        display_frame = None
        with state.lock:
            if use_overlay:
                display_frame = state.processed_frame if state.processed_frame is not None else state.latest_frame
            else:
                display_frame = state.latest_frame
        
        if display_frame is None:
            # Placeholder
            display_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(display_frame, f"No Signal: {robot_id}", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        _, jpeg = cv2.imencode('.jpg', display_frame)
        if jpeg is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.05) # ~20 FPS display limit

@app.get("/api/video/{robot_id}")
async def video_feed(robot_id: str):
    if robot_id not in manager.robots:
        raise HTTPException(status_code=404, detail="Robot not found")
    return StreamingResponse(generate_mjpeg_stream(robot_id, use_overlay=False), 
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/video/{robot_id}/overlay")
async def video_overlay_feed(robot_id: str):
    if robot_id not in manager.robots:
        raise HTTPException(status_code=404, detail="Robot not found")
    return StreamingResponse(generate_mjpeg_stream(robot_id, use_overlay=True), 
                             media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/infer/{robot_id}/latest")
async def get_latest_inference(robot_id: str):
    state = manager.get_robot(robot_id)
    if not state:
        raise HTTPException(status_code=404, detail="Robot not found")
    return state.inference_result

@app.get("/api/status")
async def get_status():
    """Full system status monitor."""
    status = {}
    for rid, s in manager.robots.items():
        status[rid] = {
            "name": s.name,
            "port": s.port,
            "model": "best_fire_det.pt",
            "receive_fps": round(s.receive_fps, 2),
            "inference_fps": round(s.inference_fps, 2),
            "latency_ms": round(s.latency_ms, 2),
            "last_addr": s.last_addr,
            "is_active": (time.time() - s.last_receive_time) < 2.0
        }
    return status

@app.get("/api/events")
async def get_events(limit: int = 50):
    """Latest locally saved AI event logs (newest first)."""
    try:
        return event_logger.read_local_events(limit=limit)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to read event logs: {e}")

@app.get("/")
def read_root():
    return {"message": "LOVO Multi-Robot AI Server is running", "robots": list(ROBOT_CONFIG.keys())}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
