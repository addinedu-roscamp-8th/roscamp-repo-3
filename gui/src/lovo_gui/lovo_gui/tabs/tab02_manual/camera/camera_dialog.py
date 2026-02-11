"""
카메라 뷰 위젯
"""
import time
import json
import yaml
import cv2
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QGridLayout, QComboBox, QRadioButton, QButtonGroup,
    QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
import numpy as np

# 알고리즘 모듈 (robot_arm에서 임포트)
from ..robot_arm.algorithm import (
    CoordinateTransformer,
    HandEyeTransformer,
    PickupSequence
)


class CameraWidget(QWidget):
    """카메라 뷰 위젯"""
    COMMAND_PICKUP = 0
    COMMAND_PICKDOWN = 1
    COMMAND_NONE = 2
    
    # Signal
    work_log_signal = pyqtSignal(str)  # 작업 로그 메시지
    
    def __init__(self, robot_name, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.camera_controller = None
        self.robot_dashboard = None  # 좌표 저장을 위한 로봇 대시보드 참조
        self.is_aligning = False
        self.align_frame = None
        # ArUco 관련 기본 상태 (기존 코드에서 참조됨 — 안전한 기본값 유지)
        self.aruco_detected = False
        self.aruco_frozen_frame = None
        self.aruco_target_coords = None
        self.aruco_marker_id = None
        
        # ArUco 관련 기능 제거 — 관련 상태는 사용하지 않음
        
        # 픽업 시퀀스 관리
        self.pickup_sequence = PickupSequence()
        
        # 캘리브레이션 데이터 로드
        # Disabled: calibration file loading is turned off per user request
        self.camera_matrix = None
        self.dist_coeffs = None
        self.hand_eye_matrix = None
        
        # 좌표 변환 알고리즘 내부 인스턴스
        self.coord_transformer = CoordinateTransformer()
        self.hand_eye_transformer = None
        if self.hand_eye_matrix is not None and self.camera_matrix is not None:
            self.hand_eye_transformer = HandEyeTransformer(self.hand_eye_matrix, self.camera_matrix)
        
        # ArUco 관련 기능 제거 — 관련 상태는 사용하지 않음
        
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 2, 3, 2)
        layout.setSpacing(2)
        
        vision_group = QGroupBox(f"📷 {self.robot_name} Camera")
        vision_group.setStyleSheet("""
            QGroupBox { 
                font-size: 10px; 
                padding-top: 8px; 
                margin-top: 0px; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; 
                subcontrol-position: top left; 
                padding: 0px 3px;
                top: -2px;
            }
        """)
        vision_layout = QVBoxLayout()
        vision_layout.setContentsMargins(2, 5, 2, 2)
        vision_layout.setSpacing(2)
        
        # 카메라 뷰
        self.cam_view = QLabel("카메라 대기 중...")
        self.cam_view.setFixedSize(640, 480)
        self.cam_view.setStyleSheet("background-color: black; color: white; border: 1px solid #555; border-radius: 4px;")
        self.cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vision_layout.addWidget(self.cam_view)
        
        # 컨트롤 버튼 (2줄로 배치, 4개씩)
        btn_container = QWidget()
        btn_container.setMaximumWidth(640)
        btn_layout = QVBoxLayout(btn_container)
        btn_layout.setContentsMargins(0, 5, 0, 0)
        btn_layout.setSpacing(3)
        
        # ============ 버튼 크기 설정 (여기서 수정) ============
        BTN_WIDTH = 100   # 버튼 가로 크기 (축소)
        BTN_HEIGHT = 24   # 버튼 세로 크기 (축소)
        # =====================================================
        
        # 첫 번째 줄: Connect, 캡쳐, Live
        btn_row1 = QHBoxLayout()
        btn_row1.setSpacing(3)
        
        self.btn_connect = QPushButton("🔌 Connect")
        self.btn_connect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_connect.clicked.connect(self._camera_connect)
        btn_row1.addWidget(self.btn_connect)
        
        self.btn_capture = QPushButton("📸 캡쳐")
        self.btn_capture.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture.setEnabled(False)
        self.btn_capture.clicked.connect(self._camera_capture)
        btn_row1.addWidget(self.btn_capture)
        
        self.btn_live = QPushButton("📺 Live")
        self.btn_live.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_live.setEnabled(False)
        self.btn_live.clicked.connect(self._camera_live)
        btn_row1.addWidget(self.btn_live)
        
        btn_row1.addStretch()
        btn_layout.addLayout(btn_row1)
        
        # 두 번째 줄: Disconnect, 캡쳐+좌표, Send Video
        btn_row2 = QHBoxLayout()
        btn_row2.setSpacing(3)
        
        self.btn_disconnect = QPushButton("❌ Disconnect")
        self.btn_disconnect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._camera_disconnect)
        btn_row2.addWidget(self.btn_disconnect)
        
        self.btn_capture_with_coords = QPushButton("📸 캡쳐+좌표")
        self.btn_capture_with_coords.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture_with_coords.setEnabled(False)
        self.btn_capture_with_coords.clicked.connect(self._camera_capture_with_coords)
        btn_row2.addWidget(self.btn_capture_with_coords)

        # Send Video 버튼: ROS로 영상 전송 기능 연결 예정
        self.btn_send_video = QPushButton("📡 Send Video")
        self.btn_send_video.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_send_video.setEnabled(False)
        self.btn_send_video.clicked.connect(self._camera_send_video)
        btn_row2.addWidget(self.btn_send_video)
        
        btn_row2.addStretch()
        btn_layout.addLayout(btn_row2)

        # 세 번째 줄: PickUp, PickDown
        btn_row3 = QHBoxLayout()
        btn_row3.setSpacing(3)

        self.btn_pickup = QPushButton("⬆️ PickUp")
        self.btn_pickup.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_pickup.setEnabled(False)
        self.btn_pickup.clicked.connect(self._camera_pickup)
        btn_row3.addWidget(self.btn_pickup)

        self.btn_pickdown = QPushButton("⬇️ PickDown")
        self.btn_pickdown.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_pickdown.setEnabled(False)
        self.btn_pickdown.clicked.connect(self._camera_pickdown)
        btn_row3.addWidget(self.btn_pickdown)

        btn_row3.addStretch()
        btn_layout.addLayout(btn_row3)
        vision_layout.addWidget(btn_container)
        vision_group.setLayout(vision_layout)
        layout.addWidget(vision_group)

        # 카메라 아래의 추가 레이아웃 패널 (상단 카메라 뷰와 작업 로그 사이)
        from pathlib import Path
        import json

        # 캡션은 'offset'으로 고정하고, 값은 config/camera_parameters.json에서 읽어옵니다.
        offset_value = "-"
        try:
            cfg_path = Path("config") / "camera_parameters.json"
            if cfg_path.exists():
                with open(cfg_path, 'r', encoding='utf-8') as f:
                    cfg = json.load(f)
                    entry = cfg.get(self.robot_name)
                    if entry and isinstance(entry, dict):
                        coord = entry.get('coord', {})
                        offset_value = coord.get('z_offset', offset_value)
        except Exception:
            offset_value = "-"

        self.camera_extra_group = QGroupBox("offset")
        self.camera_extra_group.setFixedHeight(120)
        extra_layout = QVBoxLayout()
        extra_layout.setContentsMargins(8, 8, 8, 8)
        # 값 표시 라벨 (work log 스타일과 유사하게 테두리 적용)
        self.camera_offset_label = QLabel(str(offset_value))
        self.camera_offset_label.setStyleSheet("background-color: black; color: white; border: 2px solid #555; border-radius: 4px; padding:6px;")
        extra_layout.addWidget(self.camera_offset_label)
        self.camera_extra_group.setLayout(extra_layout)
        layout.addWidget(self.camera_extra_group)
        
        # 파라미터 섹션: ArUco 및 좌표 변환 관련 UI/변수는 제거됨

        layout.addStretch()
    
    def set_camera_controller(self, camera_controller):
        """카메라 컨트롤러 설정 및 Signal 연결"""
        self.camera_controller = camera_controller
        
        if camera_controller:
            # QueuedConnection으로 스레드 안전하게 연결
            from PyQt6.QtCore import Qt
            camera_controller.frame_updated.connect(
                self.update_camera_frame, 
                Qt.ConnectionType.QueuedConnection
            )
            camera_controller.connection_changed.connect(
                self.on_camera_connection_changed,
                Qt.ConnectionType.QueuedConnection
            )

    def set_robot_dashboard(self, robot_dashboard):
        """로봇 대시보드 참조 설정 (좌표 저장용)"""
        self.robot_dashboard = robot_dashboard
    
    
    def _camera_connect(self):
        """카메라 연결"""
        if self.camera_controller:
            self.camera_controller.start()
    
    def _camera_disconnect(self):
        """카메라 연결 해제"""
        if self.camera_controller:
            # 로컬에서만 수신을 중지하고 원격 송출에는 영향 주지 않음
            try:
                self.camera_controller.stop(local_only=True)
            except TypeError:
                # 구 버전 호환성: 인자가 없는 stop() 호출 지원
                self.camera_controller.stop()
    
    def _camera_capture(self):
        """카메라 캡처"""
        if self.camera_controller:
            # 절대 경로 사용 (home 디렉토리 기준)
            home_dir = Path.home()
            captures_dir = home_dir / "lovo_ws" / "captures" / self.robot_name
            
            # 타임스탬프 기반 파일명 생성
            timestamp = int(time.time())
            filename = captures_dir / f"capture_{timestamp}.jpg"
            
            # 캡처 저장 (경로 전달, 폴더 생성은 controller에서 처리)
            result = self.camera_controller.capture(str(filename))
            if result:
                log_msg = f"📸 캡처 저장됨: {filename}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
            else:
                log_msg = f"⚠️ 캡처 저장 실패"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
    
    def _camera_live(self):
        """Live 버튼: 라이브 영상 모드로 전환 (ArUco 관련 동작 제거)"""
        log_msg = "📺 라이브 영상 모드로 전환"
        print(log_msg)
        try:
            self.work_log_signal.emit(log_msg)
        except Exception:
            pass
    

    def _camera_capture_with_coords(self):
        """카메라 캡처 및 현재 좌표 저장"""
        if not self.camera_controller:
            log_msg = "⚠️ 카메라가 연결되지 않았습니다"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            return

        home_dir = Path.home()
        captures_dir = home_dir / "lovo_ws" / "captures" / self.robot_name

        timestamp = int(time.time())
        image_filename = captures_dir / f"capture_{timestamp}.jpg"
        coords_filename = captures_dir / f"coords_{timestamp}.json"

        result = self.camera_controller.capture(str(image_filename))
        if not result:
            log_msg = "❌ 이미지 캡처 실패"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            return

        if self.robot_dashboard and hasattr(self.robot_dashboard, 'pose_target_inputs'):
            try:
                coords = []
                for i in range(6):
                    if self.robot_dashboard.pose_target_inputs[i]:
                        text = self.robot_dashboard.pose_target_inputs[i].text()
                        coords.append(float(text) if text else 0.0)
                    else:
                        coords.append(0.0)

                data = {
                    "timestamp": timestamp,
                    "robot_name": self.robot_name,
                    "image": image_filename.name,
                    "coordinates": {
                        "X": coords[0],
                        "Y": coords[1],
                        "Z": coords[2],
                        "R": coords[3],
                        "P": coords[4],
                        "Y": coords[5]
                    }
                }

                with open(coords_filename, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)

                log_msg = (
                    f"✅ 캡처+좌표 저장 완료: {image_filename.name}, {coords_filename.name}"
                )
                print(log_msg)
                self.work_log_signal.emit(log_msg)
            except Exception as e:
                log_msg = f"❌ 좌표 저장 오류: {e}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
        else:
            log_msg = "⚠️ 로봇 대시보드가 연결되지 않았습니다"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
    
    def _camera_send_video(self):
        self._send_frame_with_command(self.COMMAND_NONE, "Send Video")

    def _camera_pickup(self):
        """PickUp 커맨드 전송"""
        self._send_frame_with_command(self.COMMAND_PICKUP, "PickUp")

    def _camera_pickdown(self):
        """PickDown 커맨드 전송"""
        self._send_frame_with_command(self.COMMAND_PICKDOWN, "PickDown")

    def _send_frame_with_command(self, command_value, action_name):
        """현재 프레임을 JPEG+command 형태로 전송"""
        if not self.camera_controller:
            msg = "⚠️ 카메라 컨트롤러가 연결되지 않았습니다"
            print(msg)
            self.work_log_signal.emit(msg)
            return

        frame = getattr(self.camera_controller, 'latest_frame', None)
        if frame is None:
            msg = "⚠️ 전송할 프레임이 없습니다"
            print(msg)
            self.work_log_signal.emit(msg)
            return

        try:
            # JPEG 인코딩
            ret, buf = cv2.imencode('.jpg', frame)
            if not ret:
                raise RuntimeError('JPEG encoding failed')

            jpg_bytes = buf.tobytes()

            # CameraController의 with-command ROS 발행 함수 사용
            if hasattr(self.camera_controller, 'publish_frame_with_command_ros'):
                published = self.camera_controller.publish_frame_with_command_ros(
                    jpg_bytes, command_value=command_value
                )
                if published:
                    msg = f"📡 {action_name} 전송 완료 (command={command_value})"
                    print(msg)
                    self.work_log_signal.emit(msg)
                    return
                print("⚠️ publish_frame_with_command_ros 전송 실패, 파일 저장으로 대체")

            # 대체 동작: 파일 저장
            home_dir = Path.home()
            captures_dir = home_dir / "lovo_ws" / "captures" / self.robot_name / "send_video"
            captures_dir.mkdir(parents=True, exist_ok=True)
            timestamp = int(time.time())
            filename = captures_dir / f"sendframe_{action_name.lower()}_{timestamp}.jpg"
            with open(filename, 'wb') as f:
                f.write(jpg_bytes)

            msg = f"📡 {action_name} 프레임 저장: {filename}"
            print(msg)
            self.work_log_signal.emit(msg)

        except Exception as e:
            msg = f"❌ {action_name} 전송 오류: {e}"
            print(msg)
            self.work_log_signal.emit(msg)
    
    
    def update_camera_frame(self, frame):
        """카메라 프레임 업데이트"""
        try:
            if frame is None:
                return
            
            # ArUco 감지 후 영상 정지 상태면 정지된 프레임 사용
            if self.aruco_detected and self.aruco_frozen_frame is not None:
                display_frame = self.aruco_frozen_frame
            # 얼라인 모드면 십자가가 그려진 프레임 사용
            elif self.is_aligning and self.align_frame is not None:
                display_frame = self.align_frame
            else:
                display_frame = frame
            
            if display_frame is None or display_frame.size == 0:
                return
            
            # 프레임 복사 (메모리 안전성)
            display_frame = np.ascontiguousarray(display_frame)
            
            # OpenCV BGR → RGB 변환
            if len(display_frame.shape) == 3 and display_frame.shape[2] == 3:
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            elif len(display_frame.shape) == 2:
                # 그레이스케일인 경우
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_GRAY2RGB)
            else:
                rgb_frame = display_frame
            
            rgb_frame = np.ascontiguousarray(rgb_frame)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            
            # QImage 생성 (데이터 복사)
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            
            # QLabel 크기에 맞게 스케일링
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.cam_view.width(), 
                self.cam_view.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            self.cam_view.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"⚠️ 프레임 업데이트 오류: {e}")
    
    
    def on_camera_connection_changed(self, connected):
        """카메라 연결 상태 변경"""
        if connected:
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.btn_capture.setEnabled(True)
            self.btn_live.setEnabled(True)
            # Align/Pickup/ArUco removed -> enable Send Video instead
            try:
                self.btn_send_video.setEnabled(True)
            except Exception:
                pass
            try:
                self.btn_pickup.setEnabled(True)
                self.btn_pickdown.setEnabled(True)
            except Exception:
                pass
            self.btn_capture_with_coords.setEnabled(True)
            self.cam_view.setText("")
        else:
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.btn_capture.setEnabled(False)
            self.btn_live.setEnabled(False)
            try:
                self.btn_send_video.setEnabled(False)
            except Exception:
                pass
            try:
                self.btn_pickup.setEnabled(False)
                self.btn_pickdown.setEnabled(False)
            except Exception:
                pass
            self.btn_capture_with_coords.setEnabled(False)
            self.cam_view.clear()
            self.cam_view.setText("카메라 연결 끊김")
