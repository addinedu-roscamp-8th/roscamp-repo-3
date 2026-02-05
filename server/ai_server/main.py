from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional, Dict
import numpy as np
import time
import socket
import threading
import cv2
import json
from collections import deque
from ultralytics import YOLO

app = FastAPI(title="LOVO Multi-Robot AI Analysis Server")

# --- Model & Robot Configuration ---
ROBOT_CONFIG = {
    "robot1": {"port": 9511, "obs_port": 9512, "model": "Model_A", "name": "상차 로봇"},
    "robot2": {"port": 9521, "obs_port": 9522, "model": "Model_A", "name": "하차 로봇"},
    "robot3": {"port": 9541, "obs_port": 9541, "model": "Model_B", "name": "청소 로봇 (Pinky)"},
}

# --- State Management ---
class RobotState:
    def __init__(self, robot_id, config):
        self.robot_id = robot_id
        self.name = config["name"]
        self.port = config["port"]
        self.model_type = config["model"]
        
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
        # Load Models
        self.model_a = YOLO("models/yolov8n.pt")      # 상차/하차용
        self.model_b = YOLO("models/yolov8n-seg.pt")  # 핑키 세그멘테이션용
        print("YOLO Models loaded successfully.")

    def get_robot(self, robot_id) -> Optional[RobotState]:
        return self.robots.get(robot_id)

manager = RobotManager(ROBOT_CONFIG)

# --- UDP Receiver Task ---
def udp_receiver_task(robot_state: RobotState):
    """Independent UDP Receiver for each robot port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", robot_state.port))
        sock.settimeout(1.0)
        print(f"Receiver started for {robot_state.robot_id} on port {robot_state.port}")
    except Exception as e:
        print(f"Error binding port {robot_state.port} for {robot_state.robot_id}: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(65507)
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
            print(f"UDP Error ({robot_state.robot_id}): {e}")

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
                try:
                    target_model = manager.model_a if state.model_type == "Model_A" else manager.model_b
                    
                    # 1. YOLO Inference with Tracking
                    # We use tracking if model_b or if specifically requested
                    results = target_model.track(frame_to_process, persist=True, verbose=False, conf=0.3)
                    
                    target_data = None
                    if results and len(results) > 0 and results[0].boxes.id is not None:
                        # Find the best target (e.g., first detected person if multiple)
                        # For simplicity, take the first one with a track_id
                        boxes = results[0].boxes
                        track_ids = boxes.id.int().cpu().tolist()
                        confs = boxes.conf.cpu().tolist()
                        xyxy = boxes.xyxy.cpu().tolist()
                        
                        # Use the first one
                        idx = 0 
                        x1, y1, x2, y2 = xyxy[idx]
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0
                        
                        target_data = {
                            "track_id": track_ids[idx],
                            "conf": round(confs[idx], 2),
                            "cx": round(cx, 1),
                            "cy": round(cy, 1),
                            "bbox": [int(x1), int(y1), int(x2), int(y2)]
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

                    # 3. Broadcast over UDP
                    if state.last_addr:
                        target_port = ROBOT_CONFIG[robot_id]["obs_port"]
                        try:
                            msg_json = json.dumps(obs_msg).encode('utf-8')
                            broadcast_sock.sendto(msg_json, (state.last_addr, target_port))
                            if state.seq_counter % 100 == 1:
                                print(f"Broadcast sent to {robot_id} ({state.last_addr}:{target_port})")
                        except Exception as e:
                            print(f"Broadcast Error ({robot_id}): {e}")

                except Exception as e:
                    print(f"Inference Error on {robot_id}: {e}")
                    processed_frame = frame_to_process.copy()

                with state.lock:
                    state.processed_frame = processed_frame
                    state.inference_result = obs_msg # Store the full observation
                
                state.inference_count += 1
                state.latency_ms = (time.time() - start_time) * 1000
                state.last_inference_time = time.time()
        
        time.sleep(0.01)

# --- App Lifecycle ---
@app.on_event("startup")
async def startup_event():
    # Start receivers for all robots
    for robot_id, state in manager.robots.items():
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
            "model": s.model_type,
            "receive_fps": round(s.receive_fps, 2),
            "inference_fps": round(s.inference_fps, 2),
            "latency_ms": round(s.latency_ms, 2),
            "last_addr": s.last_addr,
            "is_active": (time.time() - s.last_receive_time) < 2.0
        }
    return status

@app.get("/")
def read_root():
    return {"message": "LOVO Multi-Robot AI Server is running", "robots": list(ROBOT_CONFIG.keys())}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
