"""
카메라 뷰 위젯
"""
import time
import json
import csv
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
    OFFSET_TARGET_ROBOTS = {"상차 로봇팔", "하차 로봇팔"}
    
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
        self._is_loading_offsets = False
        self.offset_memory_dir = Path("config") / "pose_memory"
        self.offset_memory_enabled = self.robot_name in self.OFFSET_TARGET_ROBOTS
        
        # 픽업/픽다운 메모리 슬롯 (4개: 상판, 다리, 바퀴, tool박스)
        self.slot_count = 4
        self.slot_labels = {1: "상판", 2: "다리", 3: "바퀴", 4: "tool박스"}
        self.current_slot = 1  # 기본 슬롯: 1 (상판)
        self.pickup_memory = {i: {"x": 0.0, "y": 0.0, "z": 0.0, "z_lift": 0.0} for i in range(1, self.slot_count + 1)}
        self.pickdown_memory = {i: {"x": 0.0, "y": 0.0, "z": 0.0, "z_lift": 0.0} for i in range(1, self.slot_count + 1)}
        
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
        self._initialize_offset_memory()
    
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
        
        # 컨트롤 버튼 (4열 x 2행)
        btn_container = QWidget()
        btn_container.setMaximumWidth(640)
        btn_layout = QGridLayout(btn_container)
        btn_layout.setContentsMargins(0, 5, 0, 0)
        btn_layout.setSpacing(3)
        
        # ============ 버튼 크기 설정 (여기서 수정) ============
        BTN_WIDTH = 100   # 버튼 가로 크기 (축소)
        BTN_HEIGHT = 34   # 버튼 세로 크기 (+10)
        # =====================================================
        
        self.btn_connect = QPushButton("🔌 Connect")
        self.btn_connect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_connect.clicked.connect(self._camera_connect)
        
        self.btn_capture = QPushButton("📸 캡쳐")
        self.btn_capture.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture.setEnabled(False)
        self.btn_capture.clicked.connect(self._camera_capture)
        
        self.btn_live = QPushButton("📺 Live")
        self.btn_live.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_live.setEnabled(False)
        self.btn_live.clicked.connect(self._camera_live)
        
        self.btn_disconnect = QPushButton("❌ Disconnect")
        self.btn_disconnect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._camera_disconnect)
        
        self.btn_capture_with_coords = QPushButton("📸 캡쳐+좌표")
        self.btn_capture_with_coords.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture_with_coords.setEnabled(False)
        self.btn_capture_with_coords.clicked.connect(self._camera_capture_with_coords)

        # Send Video 버튼: ROS로 영상 전송 기능 연결 예정
        self.btn_send_video = QPushButton("📡 Send Video")
        self.btn_send_video.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_send_video.setEnabled(False)
        self.btn_send_video.clicked.connect(self._camera_send_video)

        self.btn_pickup = QPushButton("⬆️ PickUp")
        self.btn_pickup.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_pickup.setEnabled(False)
        self.btn_pickup.clicked.connect(self._camera_pickup)

        self.btn_pickdown = QPushButton("⬇️ PickDown")
        self.btn_pickdown.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_pickdown.setEnabled(False)
        self.btn_pickdown.clicked.connect(self._camera_pickdown)
        
        # 4x2 배치
        # 1행: Connect / Capture / Live / PickUp
        # 2행: Disconnect / Capture+Coords / Send Video / PickDown
        btn_layout.addWidget(self.btn_connect, 0, 0)
        btn_layout.addWidget(self.btn_capture, 0, 1)
        btn_layout.addWidget(self.btn_live, 0, 2)
        btn_layout.addWidget(self.btn_pickup, 0, 3)
        btn_layout.addWidget(self.btn_disconnect, 1, 0)
        btn_layout.addWidget(self.btn_capture_with_coords, 1, 1)
        btn_layout.addWidget(self.btn_send_video, 1, 2)
        btn_layout.addWidget(self.btn_pickdown, 1, 3)
        vision_layout.addWidget(btn_container)
        vision_group.setLayout(vision_layout)
        layout.addWidget(vision_group)

        # 카메라 아래 offset 패널 (상차/하차 로봇팔에서만 표시)
        self.camera_extra_group = None
        self.slot_buttons = {}  # 슬롯 버튼 참조
        
        if self.offset_memory_enabled:
            self.camera_extra_group = QGroupBox()  # 타이틀 제거
            self.camera_extra_group.setFixedHeight(135)
            extra_layout = QVBoxLayout()
            extra_layout.setContentsMargins(4, 4, 4, 4)
            extra_layout.setSpacing(2)

            # ========== 📐 UI 크기 설정 변수 (여기서 조정) ==========
            # 그리드 간격
            GRID_H_SPACING = 0      # 가로 간격
            GRID_V_SPACING = 2      # 세로 간격
            
            # 그리드 셀 크기
            HEADER_HEIGHT = 28      # 헤더 높이
            HEADER_WIDTH = 65       # 첫 번째 열(슬롯명) 너비
            ROW_HEIGHT = 28         # 행 라벨 높이
            SPINBOX_WIDTH = 90      # 스핀박스 너비
            SPINBOX_HEIGHT = 28     # 스핀박스 높이
            
            # 폰트 및 여백
            FONT_SIZE = 14          # 폰트 크기
            CELL_PADDING = 2        # 셀 내부 여백
            SPINBOX_PADDING_V = 2   # 스핀박스 상하 여백
            SPINBOX_PADDING_H = 4   # 스핀박스 좌우 여백
            # ======================================================

            # === 헤더: 라벨 + 슬롯 선택 버튼 (같은 줄) ===
            header_layout = QHBoxLayout()   
            header_layout.setSpacing(10)
            
            # 왼쪽: Offset 설정 라벨
            offset_title_label = QLabel(f"{self.robot_name} Offset 설정")
            offset_title_label.setStyleSheet("font-weight: bold; font-size: 11px; color: #1976D2;")
            header_layout.addWidget(offset_title_label)
            
            # 오른쪽: 슬롯 버튼 (가로 배치) - 4개
            for slot_idx in range(1, self.slot_count + 1):
                slot_label = self.slot_labels[slot_idx]
                btn = QPushButton(f"{slot_idx}. {slot_label}")
                btn.setFixedSize(95, 28)
                btn.setCheckable(True)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #2d2d2d;
                        border: 2px solid #555;
                        border-radius: 4px;
                        padding: 5px;
                        font-size: 11px;
                        font-weight: bold;
                    }
                    QPushButton:checked {
                        background-color: #1976D2;
                        border: 2px solid #1565C0;
                    }
                    QPushButton:hover {
                        background-color: #3d3d3d;
                    }
                """)
                btn.clicked.connect(lambda checked, s=slot_idx: self._select_slot(s))
                self.slot_buttons[slot_idx] = btn
                header_layout.addWidget(btn)
            
            
            header_layout.addStretch()
            extra_layout.addLayout(header_layout)
            
            # === Offset 그리드 ===
            unified_grid = QGridLayout()
            unified_grid.setHorizontalSpacing(GRID_H_SPACING)
            unified_grid.setVerticalSpacing(GRID_V_SPACING)

            col_headers = ["X", "Y", "Z", "Z 상승"]
            keys = ["x", "y", "z", "z_lift"]

            header_style = f"border: none; padding: {CELL_PADDING}px; background-color: #3a3a3a; font-size: {FONT_SIZE}px; font-weight: bold;"
            row_label_style = f"border: none; padding: {CELL_PADDING}px; background-color: #4a4a4a; font-size: {FONT_SIZE}px; font-weight: bold;"
            spin_style = (
                f"QDoubleSpinBox {{ border: none; padding: {SPINBOX_PADDING_V}px {SPINBOX_PADDING_H}px; background-color: #2f2f2f; color: white; font-size: {FONT_SIZE}px; }}"
                "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 14px; }"
            )

            # 헤더 행 - 첫 번째 셀은 슬롯 이름 (동적)
            self.slot_header_label = QLabel("상판")
            self.slot_header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.slot_header_label.setStyleSheet(header_style)
            self.slot_header_label.setFixedHeight(HEADER_HEIGHT)
            self.slot_header_label.setFixedWidth(HEADER_WIDTH)
            unified_grid.addWidget(self.slot_header_label, 0, 0)
            
            # 나머지 헤더 (X, Y, Z, Z상승)
            for col, header in enumerate(col_headers, start=1):
                lbl = QLabel(header)
                lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lbl.setStyleSheet(header_style)
                lbl.setFixedHeight(HEADER_HEIGHT)
                unified_grid.addWidget(lbl, 0, col)

            # 픽업 행
            pickup_label = QLabel("픽업")
            pickup_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pickup_label.setStyleSheet(row_label_style)
            pickup_label.setFixedHeight(ROW_HEIGHT)
            unified_grid.addWidget(pickup_label, 1, 0)
            
            self.pickup_offset_inputs = {}
            for col, key in enumerate(keys, start=1):
                spin = QDoubleSpinBox()
                spin.setRange(-9999.0, 9999.0)
                spin.setDecimals(2)
                spin.setSingleStep(0.1)
                spin.setValue(0.0)
                spin.setFixedHeight(SPINBOX_HEIGHT)
                spin.setFixedWidth(SPINBOX_WIDTH)
                spin.setStyleSheet(spin_style)
                spin.valueChanged.connect(self._on_pickup_offset_changed)
                self.pickup_offset_inputs[key] = spin
                unified_grid.addWidget(spin, 1, col)

            # 픽다운 행
            pickdown_label = QLabel("픽다운")
            pickdown_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pickdown_label.setStyleSheet(row_label_style)
            pickdown_label.setFixedHeight(ROW_HEIGHT)
            unified_grid.addWidget(pickdown_label, 2, 0)
            
            self.pickdown_offset_inputs = {}
            for col, key in enumerate(keys, start=1):
                spin = QDoubleSpinBox()
                spin.setRange(-9999.0, 9999.0)
                spin.setDecimals(2)
                spin.setSingleStep(0.1)
                spin.setValue(0.0)
                spin.setFixedHeight(SPINBOX_HEIGHT)
                spin.setFixedWidth(SPINBOX_WIDTH)
                spin.setStyleSheet(spin_style)
                spin.valueChanged.connect(self._on_pickdown_offset_changed)
                self.pickdown_offset_inputs[key] = spin
                unified_grid.addWidget(spin, 2, col)
            
            extra_layout.addLayout(unified_grid)
            
            # 기본 슬롯(1) 선택
            self.slot_buttons[1].setChecked(True)
            self._update_row_labels()  # 라벨 업데이트
            
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
        if self.offset_memory_enabled:
            self.save_offsets_to_csv()
        self._send_frame_with_command(self.COMMAND_PICKUP, "PickUp")

    def _camera_pickdown(self):
        """PickDown 커맨드 전송"""
        if self.offset_memory_enabled:
            self.save_offsets_to_csv()
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
            offsets = self._get_offsets_for_action(action_name)

            # CameraController의 with-command ROS 발행 함수 사용
            if hasattr(self.camera_controller, 'publish_frame_with_command_ros'):
                published = self.camera_controller.publish_frame_with_command_ros(
                    jpg_bytes, command_value=command_value, offsets=offsets
                )
                if published:
                    msg = (
                        f"📡 {action_name} 전송 완료 (command={command_value}, "
                        f"offsets={offsets})"
                    )
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

    def _get_offsets_for_action(self, action_name):
        """액션에 맞는 offset 4개를 반환 (x, y, z, z_lift)."""
        if action_name == "PickUp":
            # 픽업: 현재 화면의 픽업 Offset 값을 읽어옴
            try:
                if hasattr(self, 'pickup_offset_inputs'):
                    return [
                        float(self.pickup_offset_inputs["x"].value()),
                        float(self.pickup_offset_inputs["y"].value()),
                        float(self.pickup_offset_inputs["z"].value()),
                        float(self.pickup_offset_inputs["z_lift"].value()),
                    ]
            except Exception as e:
                print(f"⚠️ 픽업 Offset 읽기 실패: {e}")
                pass
        
        elif action_name == "PickDown":
            # 픽다운: 현재 화면의 픽다운 Offset 값을 읽어옴
            try:
                if hasattr(self, 'pickdown_offset_inputs'):
                    return [
                        float(self.pickdown_offset_inputs["x"].value()),
                        float(self.pickdown_offset_inputs["y"].value()),
                        float(self.pickdown_offset_inputs["z"].value()),
                        float(self.pickdown_offset_inputs["z_lift"].value()),
                    ]
            except Exception as e:
                print(f"⚠️ 픽다운 Offset 읽기 실패: {e}")
                pass
        
        return [0.0, 0.0, 0.0, 0.0]

    def _get_offset_csv_path(self):
        """현재 로봇의 offset 메모리 CSV 경로 반환."""
        robot_id = self.robot_name.replace(" ", "_")
        return self.offset_memory_dir / f"{robot_id}_offset.csv"

    def _collect_offsets(self):
        """UI 스핀박스의 offset 값을 dict로 수집 (pickup + pickdown 슬롯)."""
        data = {"pickup_slots": {}, "pickdown_slots": {}}
        
        # 현재 화면 값을 메모리에 저장 (최신 상태 유지)
        if self.offset_memory_enabled and not self._is_loading_offsets:
            self._save_ui_to_slot()

        # 픽업 슬롯 메모리 수집
        for slot_idx in range(1, self.slot_count + 1):
            slot_data = self.pickup_memory.get(slot_idx, {})
            # slot_data가 비어있으면 기본값 0.0으로 채움
            data["pickup_slots"][slot_idx] = {
                "x": float(slot_data.get("x", 0.0)),
                "y": float(slot_data.get("y", 0.0)),
                "z": float(slot_data.get("z", 0.0)),
                "z_lift": float(slot_data.get("z_lift", 0.0)),
            }
        
        # 픽다운 슬롯 메모리 수집
        for slot_idx in range(1, self.slot_count + 1):
            slot_data = self.pickdown_memory.get(slot_idx, {})
            data["pickdown_slots"][slot_idx] = {
                "label": self.slot_labels.get(slot_idx, f"Slot{slot_idx}"),
                "x": float(slot_data.get("x", 0.0)),
                "y": float(slot_data.get("y", 0.0)),
                "z": float(slot_data.get("z", 0.0)),
                "z_lift": float(slot_data.get("z_lift", 0.0)),
            }
        
        return data

    def _apply_offsets(self, offsets):
        """dict 형태의 offset 값을 UI 스핀박스 및 메모리에 반영."""
        self._is_loading_offsets = True
        try:
            # 픽업 슬롯 메모리에 적용
            pickup_slots = offsets.get("pickup_slots", {})
            for slot_idx, slot_data in pickup_slots.items():
                if isinstance(slot_idx, str):
                    slot_idx = int(slot_idx)
                # 이전 데이터 구조 호환성 (pickup 키가 바로 있는 경우)
                if slot_idx not in self.pickup_memory:
                    self.pickup_memory[slot_idx] = {}
                    
                self.pickup_memory[slot_idx] = {
                    "x": float(slot_data.get("x", 0.0)),
                    "y": float(slot_data.get("y", 0.0)),
                    "z": float(slot_data.get("z", 0.0)),
                    "z_lift": float(slot_data.get("z_lift", 0.0)),
                }

            # 구버전 데이터 호환성 (offsets["pickup"] -> 슬롯 1로)
            # 주의: offsets["pickup"]이 비어있지 않을 때만 적용 (빈 dict면 0.0으로 덮어써짐 방지)
            if "pickup" in offsets and isinstance(offsets["pickup"], dict) and offsets["pickup"]:
                p_data = offsets["pickup"]
                self.pickup_memory[1] = {
                    "x": float(p_data.get("x", 0.0)),
                    "y": float(p_data.get("y", 0.0)),
                    "z": float(p_data.get("z", 0.0)),
                    "z_lift": float(p_data.get("z_lift", 0.0)),
                }
            
            # 픽다운 슬롯 메모리에 적용
            pickdown_slots = offsets.get("pickdown_slots", {})
            for slot_idx, slot_data in pickdown_slots.items():
                if isinstance(slot_idx, str):
                    slot_idx = int(slot_idx)
                
                if slot_idx not in self.pickdown_memory:
                    self.pickdown_memory[slot_idx] = {}

                self.pickdown_memory[slot_idx] = {
                    "x": float(slot_data.get("x", 0.0)),
                    "y": float(slot_data.get("y", 0.0)),
                    "z": float(slot_data.get("z", 0.0)),
                    "z_lift": float(slot_data.get("z_lift", 0.0)),
                }
            
            # 현재 선택된 슬롯의 값을 UI에 반영
            self._update_ui_from_slot()
        finally:
            self._is_loading_offsets = False
    
    def _select_slot(self, slot_idx):
        """슬롯 선택 (픽업/픽다운 모두)"""
        if not self.offset_memory_enabled:
            return
        
        # 현재 UI의 값을 이전 슬롯 메모리에 저장
        self._save_ui_to_slot()
        
        # 새 슬롯으로 변경
        self.current_slot = slot_idx
        
        # 버튼 상태 업데이트
        for idx, btn in self.slot_buttons.items():
            btn.setChecked(idx == slot_idx)
        
        # 새 슬롯의 값을 UI에 로드
        self._update_ui_from_slot()
        
        # 행 라벨 업데이트
        self._update_row_labels()
        
        # 로그 메시지
        slot_label = self.slot_labels.get(slot_idx, f"Slot{slot_idx}")
        try:
            self.work_log_signal.emit(f"🔄 슬롯 전환: {slot_idx}. {slot_label}")
        except Exception:
            pass
        
        # CSV 저장
        self.save_offsets_to_csv()
    
    def _update_row_labels(self):
        """슬롯 헤더 업데이트 (현재 선택된 슬롯 표시)"""
        if not self.offset_memory_enabled:
            return
        
        slot_label = self.slot_labels.get(self.current_slot, f"Slot{self.current_slot}")
        self.slot_header_label.setText(slot_label)
    
    def _update_ui_from_slot(self):
        """현재 선택된 슬롯의 offset 값을 UI에 반영 (픽업+픽다운)"""
        if not self.offset_memory_enabled:
            return
        
        self._is_loading_offsets = True
        try:
            # 픽업 슬롯 데이터 로드
            pickup_data = self.pickup_memory.get(self.current_slot, {})
            for key in ("x", "y", "z", "z_lift"):
                spin = self.pickup_offset_inputs.get(key)
                if spin:
                    spin.setValue(float(pickup_data.get(key, 0.0)))
            
            # 픽다운 슬롯 데이터 로드
            pickdown_data = self.pickdown_memory.get(self.current_slot, {})
            for key in ("x", "y", "z", "z_lift"):
                spin = self.pickdown_offset_inputs.get(key)
                if spin:
                    spin.setValue(float(pickdown_data.get(key, 0.0)))
        finally:
            self._is_loading_offsets = False
    
    def _save_ui_to_slot(self):
        """현재 UI의 offset 값을 현재 슬롯 메모리에 저장 (픽업+픽다운)"""
        if not self.offset_memory_enabled:
            return
        
        # 픽업 데이터 저장
        self.pickup_memory[self.current_slot] = {
            "x": float(self.pickup_offset_inputs.get("x").value()),
            "y": float(self.pickup_offset_inputs.get("y").value()),
            "z": float(self.pickup_offset_inputs.get("z").value()),
            "z_lift": float(self.pickup_offset_inputs.get("z_lift").value()),
        }
        
        # 픽다운 데이터 저장
        self.pickdown_memory[self.current_slot] = {
            "x": float(self.pickdown_offset_inputs.get("x").value()),
            "y": float(self.pickdown_offset_inputs.get("y").value()),
            "z": float(self.pickdown_offset_inputs.get("z").value()),
            "z_lift": float(self.pickdown_offset_inputs.get("z_lift").value()),
        }
    
    def _on_pickup_offset_changed(self, _value):
        """픽업 offset 입력값 변경 시 메모리 및 CSV 저장"""
        if not self.offset_memory_enabled:
            return
        if self._is_loading_offsets:
            return
        
        # 현재 슬롯 메모리에 저장
        self.pickup_memory[self.current_slot] = {
            "x": float(self.pickup_offset_inputs.get("x").value()),
            "y": float(self.pickup_offset_inputs.get("y").value()),
            "z": float(self.pickup_offset_inputs.get("z").value()),
            "z_lift": float(self.pickup_offset_inputs.get("z_lift").value()),
        }
        
        # CSV 저장
        self.save_offsets_to_csv()
    
    def _on_pickdown_offset_changed(self, _value):
        """픽다운 offset 입력값 변경 시 메모리 및 CSV 저장"""
        if not self.offset_memory_enabled:
            return
        if self._is_loading_offsets:
            return
        
        # 현재 슬롯 메모리에 저장
        self.pickdown_memory[self.current_slot] = {
            "x": float(self.pickdown_offset_inputs.get("x").value()),
            "y": float(self.pickdown_offset_inputs.get("y").value()),
            "z": float(self.pickdown_offset_inputs.get("z").value()),
            "z_lift": float(self.pickdown_offset_inputs.get("z_lift").value()),
        }
        
        # CSV 저장
        self.save_offsets_to_csv()
    
    def _select_pickdown_slot(self, slot_idx):
        """픽다운 슬롯 선택"""
        if not self.offset_memory_enabled:
            return
        
        # 현재 UI의 값을 이전 슬롯 메모리에 저장
        self._save_pickdown_ui_to_slot()
        
        # 새 슬롯으로 변경
        self.current_pickdown_slot = slot_idx
        
        # 버튼 상태 업데이트
        for idx, btn in self.pickdown_slot_buttons.items():
            btn.setChecked(idx == slot_idx)
        
        # 새 슬롯의 값을 UI에 로드
        self._update_pickdown_ui_from_slot()
        
        # 로그 메시지
        slot_label = self.pickdown_slot_labels.get(slot_idx, f"PD{slot_idx}")
        try:
            self.work_log_signal.emit(f"🔄 픽다운 슬롯 전환: PD{slot_idx} ({slot_label})")
        except Exception:
            pass
        
        # CSV 저장
        self.save_offsets_to_csv()
    
    def _update_pickdown_ui_from_slot(self):
        """현재 선택된 슬롯의 offset 값을 UI에 반영"""
        if not self.offset_memory_enabled:
            return
        
        slot_data = self.pickdown_memory.get(self.current_pickdown_slot, {})
        self._is_loading_offsets = True
        try:
            for key in ("x", "y", "z", "z_lift"):
                spin = self.pickdown_offset_inputs.get(key)
                if spin:
                    spin.setValue(float(slot_data.get(key, 0.0)))
        finally:
            self._is_loading_offsets = False
    
    def _save_pickdown_ui_to_slot(self):
        """현재 UI의 픽다운 offset 값을 현재 슬롯 메모리에 저장"""
        if not self.offset_memory_enabled:
            return
        
        self.pickdown_memory[self.current_pickdown_slot] = {
            "x": float(self.pickdown_offset_inputs.get("x").value()),
            "y": float(self.pickdown_offset_inputs.get("y").value()),
            "z": float(self.pickdown_offset_inputs.get("z").value()),
            "z_lift": float(self.pickdown_offset_inputs.get("z_lift").value()),
        }
    
    def _on_pickup_offset_changed(self, _value):
        """픽업 offset 입력값 변경 시 CSV 저장"""
        if not self.offset_memory_enabled:
            return
        if self._is_loading_offsets:
            return
        self.save_offsets_to_csv()
    
    def _on_pickdown_offset_changed(self, _value):
        """픽다운 offset 입력값 변경 시 메모리 및 CSV 저장"""
        if not self.offset_memory_enabled:
            return
        if self._is_loading_offsets:
            return
        
        # 현재 슬롯 메모리에 저장
        self._save_pickdown_ui_to_slot()
        
        # CSV 저장
        self.save_offsets_to_csv()

    def _initialize_offset_memory(self):
        """offset CSV를 로드하고, 없으면 기본값 파일 생성."""
        if not self.offset_memory_enabled:
            return
        self.offset_memory_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self._get_offset_csv_path()
        if csv_path.exists():
            self._load_offsets_from_csv()
            return
        self.save_offsets_to_csv()
        try:
            self.work_log_signal.emit(f"✅ offset CSV 생성: {csv_path}")
        except Exception:
            pass

    def _load_offsets_from_csv(self):
        """CSV 파일에서 offset 값을 로드 (새 형식: slot, label 지원)."""
        if not self.offset_memory_enabled:
            return
        csv_path = self._get_offset_csv_path()
        if not csv_path.exists():
            return

        offsets = {
            "pickup_slots": {},
            "pickdown_slots": {},
            "pickup": {} # 구버전 호환용
        }
        
        try:
            with open(csv_path, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)  # header
                
                # 헤더 확인: 새 형식(slot, label 포함) vs 구 형식
                has_slot_column = header and "slot" in [h.strip().lower() for h in header]
                
                for row in reader:
                    if len(row) < 5:
                        continue
                    
                    action = row[0].strip().lower()
                    
                    if has_slot_column:
                        # 새 형식: action, slot, label, x, y, z, z_lift
                        # 예: pickup, 1, 기본픽업, 0.7, 1.0, 3.5, 3.0
                        # 예: pickdown, 1, 상판, 0.0, 2.0, 5.5, 16.0
                        if len(row) < 7:
                            continue
                        
                        try:
                            slot = int(row[1].strip())
                            label = row[2].strip()
                            x = float(row[3])
                            y = float(row[4])
                            z = float(row[5])
                            z_lift = float(row[6])
                        except (ValueError, IndexError):
                            continue
                        
                        if action == "pickup":
                            offsets["pickup_slots"][slot] = {"x": x, "y": y, "z": z, "z_lift": z_lift}
                        elif action == "pickdown":
                            offsets["pickdown_slots"][slot] = {
                                "label": label,
                                "x": x,
                                "y": y,
                                "z": z,
                                "z_lift": z_lift,
                            }
                    else:
                        # 구 형식 (호환성): action, x, y, z, z_lift
                        # 구 pickdown 데이터는 슬롯 1로 마이그레이션
                        try:
                            x = float(row[1])
                            y = float(row[2])
                            z = float(row[3])
                            z_lift = float(row[4])
                        except (ValueError, IndexError):
                            continue
                        
                        if action == "pickup":
                            offsets["pickup"] = {"x": x, "y": y, "z": z, "z_lift": z_lift}
                        elif action == "pickdown":
                            # 구 형식의 pickdown을 슬롯 1(상판)으로 마이그레이션
                            offsets["pickdown_slots"][1] = {
                                "label": "상판",
                                "x": x,
                                "y": y,
                                "z": z,
                                "z_lift": z_lift,
                            }

            self._apply_offsets(offsets)
            try:
                self.work_log_signal.emit(f"✅ offset CSV 로드: {csv_path}")
            except Exception:
                pass
        except Exception as e:
            print(f"❌ offset CSV 로드 실패 ({self.robot_name}): {e}")

    def save_offsets_to_csv(self):
        """현재 offset 값을 CSV에 저장 (새 형식: action, slot, label, x, y, z, z_lift)."""
        if not self.offset_memory_enabled:
            return
        
        # 저장 전 현재 상태 업데이트
        self._collect_offsets()
        
        self.offset_memory_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self._get_offset_csv_path()
        offsets = self._collect_offsets()
        
        try:
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                # 헤더
                writer.writerow(["action", "slot", "label", "x", "y", "z", "z_lift"])
                
                # 픽업 슬롯 저장
                pickup_slots = offsets["pickup_slots"]
                for slot_idx in sorted(pickup_slots.keys()):
                    slot_data = pickup_slots[slot_idx]
                    writer.writerow([
                        "pickup",
                        slot_idx,
                        self.slot_labels.get(slot_idx, f"Slot{slot_idx}"),
                        slot_data["x"],
                        slot_data["y"],
                        slot_data["z"],
                        slot_data["z_lift"],
                    ])
                
                # 픽다운 슬롯 저장
                pickdown_slots = offsets["pickdown_slots"]
                for slot_idx in sorted(pickdown_slots.keys()):
                    slot_data = pickdown_slots[slot_idx]
                    writer.writerow([
                        "pickdown",
                        slot_idx,
                        slot_data["label"],
                        slot_data["x"],
                        slot_data["y"],
                        slot_data["z"],
                        slot_data["z_lift"],
                    ])
        except Exception as e:
            print(f"❌ offset CSV 저장 실패 ({self.robot_name}): {e}")
    
    
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

