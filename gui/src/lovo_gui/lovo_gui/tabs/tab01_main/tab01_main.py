"""
Main 탭
"""
from PyQt6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
    QGridLayout, QPushButton, QSizePolicy
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
import cv2
from lovo_gui.constants import MAIN_SYSTEM_MAP, MAIN_ORDER_LOG, MAIN_ROBOT_GRID, MAIN_CAMERA_VIEW


class MainTab(QWidget):
    """Main 탭 - 시스템 맵, 주문 로그, 로봇 상태, 카메라 뷰"""
    
    def __init__(self, robot_settings, parent=None):
        super().__init__(parent)
        self.robot_settings = robot_settings
        self.camera_title = None
        self.camera_view_label = None
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        # 좌상단: 시스템 맵
        self._create_system_map()
        
        # 우상단: 주문 로그
        self._create_order_log()
        
        # 좌하단: 로봇 상태 그리드
        self._create_robot_grid()
        
        # 우하단: 카메라 뷰
        self._create_camera_view()
    
    def _create_system_map(self):
        """시스템 맵"""
        x, y, w, h = MAIN_SYSTEM_MAP
        system_map = QFrame(self)
        system_map.setGeometry(x, y, w, h)
        system_map.setStyleSheet("QFrame { background-color: #e8e8e8; border: none; }")
        
        layout = QVBoxLayout(system_map)
        layout.addWidget(QLabel("시스템 맵", alignment=Qt.AlignmentFlag.AlignCenter))
    
    def _create_order_log(self):
        """주문 로그"""
        x, y, w, h = MAIN_ORDER_LOG
        order_log_frame = QFrame(self)
        order_log_frame.setGeometry(x, y, w, h)
        order_log_frame.setStyleSheet("QFrame { background-color: #f5f5f5; border: none; }")
        
        layout = QVBoxLayout(order_log_frame)
        layout.setContentsMargins(5, 5, 5, 5)
        
        title = QLabel("주문 로그")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        self.order_log_viewer = QTextEdit()
        self.order_log_viewer.setReadOnly(True)
        self.order_log_viewer.setStyleSheet("""
            background-color: white;
            color: #333;
            font-family: Consolas;
            font-size: 11px;
            border: 1px solid #ccc;
        """)
        layout.addWidget(self.order_log_viewer)
    
    def _create_robot_grid(self):
        """로봇 상태 그리드"""
        x, y, w, h = MAIN_ROBOT_GRID
        grid_container = QWidget(self)
        grid_container.setGeometry(x, y, w, h)
        grid_container.setStyleSheet("background-color: white;")
        
        grid_layout = QGridLayout(grid_container)
        grid_layout.setSpacing(0)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        
        # 헤더
        headers = ["로봇 이름", "통신 연결 상태", "배터리 잔량", "현재 상태", "캠 연결"]
        for col, header in enumerate(headers):
            header_label = QLabel(header)
            header_label.setStyleSheet("""
                background-color: #4a4a4a;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border: 1px solid #333;
            """)
            header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(header_label, 0, col)
        
        # 로봇별 정보
        robots = self.robot_settings.get_robots()
        for row, robot in enumerate(robots, start=1):
            robot_name = robot.get("name", f"로봇 {row}")
            
            # 로봇 이름
            name_label = QLabel(robot_name)
            name_label.setStyleSheet("""
                background-color: #f0f0f0;
                color: black;
                font-size: 13px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(name_label, row, 0)
            
            # 통신 연결 상태
            status_widget = QWidget()
            status_layout = QHBoxLayout(status_widget)
            status_layout.setContentsMargins(5, 0, 5, 0)
            status_layout.setSpacing(5)
            
            indicator = QFrame()
            indicator.setFixedSize(15, 15)
            indicator.setStyleSheet("""
                background-color: #28a745;
                border-radius: 7px;
                border: 1px solid #1e7e34;
            """)
            
            status_text = QLabel("Connected")
            status_text.setStyleSheet("color: black; font-size: 12px;")
            
            status_layout.addWidget(indicator)
            status_layout.addWidget(status_text)
            status_layout.addStretch()
            
            status_widget.setStyleSheet("background-color: white; border: 1px solid #ccc;")
            grid_layout.addWidget(status_widget, row, 1)
            
            # 배터리 잔량
            battery_label = QLabel("85%")
            battery_label.setStyleSheet("""
                background-color: white;
                color: black;
                font-size: 13px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            battery_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(battery_label, row, 2)
            
            # 현재 상태
            if row > 2:  # 운송/청소 로봇
                state_label = QLabel("위치: (X: 10.5, Y: 25.3)")
            else:  # 로봇팔
                state_label = QLabel("대기 중")
            state_label.setStyleSheet("""
                background-color: white;
                color: black;
                font-size: 12px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(state_label, row, 3)
            
            # 캠 연결 버튼
            cam_btn = QPushButton("📷 CAM")
            cam_btn.setFixedSize(80, 35)
            cam_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    font-size: 12px;
                    font-weight: bold;
                    border-radius: 4px;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #1976D2;
                }
                QPushButton:pressed {
                    background-color: #0D47A1;
                }
            """)
            cam_btn.clicked.connect(lambda checked, r=robot: self.show_camera_view(r))
            
            btn_container = QWidget()
            btn_container.setStyleSheet("background-color: white; border: 1px solid #ccc;")
            btn_layout = QHBoxLayout(btn_container)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            btn_layout.addWidget(cam_btn, alignment=Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(btn_container, row, 4)
    
    def _create_camera_view(self):
        """카메라 뷰"""
        x, y, w, h = MAIN_CAMERA_VIEW
        self.camera_view_frame = QFrame(self)
        self.camera_view_frame.setGeometry(x, y, w, h)
        self.camera_view_frame.setStyleSheet("""
            QFrame {
                background-color: #2a2a2a;
                border: 2px solid #555;
                border-radius: 4px;
            }
        """)
        
        layout = QVBoxLayout(self.camera_view_frame)
        layout.setContentsMargins(0, 5, 0, 5)
        layout.setSpacing(5)
        
        self.camera_title = QLabel("카메라 선택 대기 중...")
        self.camera_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #999;")
        self.camera_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_title.setFixedHeight(20)
        layout.addWidget(self.camera_title)
        
        self.camera_view_label = QLabel("캠 버튼을 눌러 카메라를 선택하세요")
        self.camera_view_label.setFixedSize(420, 270)
        self.camera_view_label.setStyleSheet(
            "background-color: black; border: 1px solid #444; border-radius: 4px; color: #666;"
        )
        self.camera_view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_view_label)

        # 현재 연결된 카메라 컨트롤러 참조 (frame_updated 연결/해제 관리)
        self._current_camera_controller = None
        self._camera_frame_slot = None
    
    def show_camera_view(self, robot):
        """카메라 뷰 표시"""
        robot_name = robot.get("name", "로봇")
        self.camera_title.setText(f"{robot_name} - 카메라 뷰")
        self.camera_view_label.setText("카메라 스트리밍 대기 중...")

        # 윈도우 레벨에서 CommunicationTab의 CameraController를 가져와 프레임 업데이트를 연결
        try:
            win = self.window()
            cam_ctrl = None
            if hasattr(win, 'communication_tab'):
                # robot dict에는 id가 있어야 함
                robot_id = robot.get('id')
                cam_ctrl = win.communication_tab.get_camera_controller(robot_id)

            # 이전 컨트롤러와 연결되어 있으면 해제
            if hasattr(self, '_current_camera_controller') and self._current_camera_controller:
                try:
                    if self._camera_frame_slot and self._current_camera_controller:
                        self._current_camera_controller.frame_updated.disconnect(self._camera_frame_slot)
                except Exception:
                    pass
                self._current_camera_controller = None
                self._camera_frame_slot = None

            if cam_ctrl is None:
                self.camera_view_label.setText("카메라가 연결되어 있지 않습니다")
                return

            # 슬롯 생성 및 연결
            def _on_frame(frame):
                try:
                    # OpenCV BGR -> RGB
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb.shape
                    bytes_per_line = ch * w
                    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    pix = QPixmap.fromImage(qimg).scaled(
                        self.camera_view_label.width(),
                        self.camera_view_label.height(),
                        Qt.AspectRatioMode.KeepAspectRatio
                    )
                    self.camera_view_label.setPixmap(pix)
                except Exception:
                    pass

            # 연결 저장 및 시그널 연결
            self._camera_frame_slot = _on_frame
            cam_ctrl.frame_updated.connect(self._camera_frame_slot)
            self._current_camera_controller = cam_ctrl

            # 상태 텍스트 제거
            self.camera_view_label.setText("")
        except Exception:
            # 안전하게 실패
            self.camera_view_label.setText("카메라 표시 실패")
