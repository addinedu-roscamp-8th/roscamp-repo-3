"""
AMR(자율주행 로봇) 대시보드 위젯
"""
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QGridLayout, QScrollArea, QLineEdit, QComboBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class AMRDashboardWidget(QWidget):
    """AMR 제어 대시보드"""
    
    def __init__(self, robot_name, robot_key, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_key = robot_key
        self.main_font = QFont("Arial", 11, QFont.Weight.Bold)
        
        # 컨트롤러
        self.controller = None
        
        # UI 위젯 참조
        self.status_label = None
        self.battery_label = None
        self.connection_state_label = None
        self.position_labels = {}  # x, y, theta
        self.velocity_labels = {}  # linear, angular
        
        self._setup_ui()
    
    def set_controller(self, controller):
        """컨트롤러 설정"""
        self.controller = controller
        # TODO: AMR Controller Signal 연결
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        content_layout = QVBoxLayout(scroll_content)
        
        # 1. 시스템 상태
        status_group = self._create_status_section()
        content_layout.addWidget(status_group)
        
        # 2. 위치 정보
        position_group = self._create_position_section()
        content_layout.addWidget(position_group)
        
        # 3. 내비게이션 제어
        nav_group = self._create_navigation_section()
        content_layout.addWidget(nav_group)
        
        # 4. 수동 제어
        manual_group = self._create_manual_control_section()
        content_layout.addWidget(manual_group)
        
        content_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
    
    def _create_status_section(self):
        """시스템 상태 섹션"""
        group = QGroupBox("🤖 시스템 상태")
        group.setFont(self.main_font)
        layout = QGridLayout()

        conn_title = QLabel("통신:")
        layout.addWidget(conn_title, 0, 0)
        self.connection_state_label = QLabel()
        self.connection_state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._set_connection_status_text("Unknown", "#616161")
        layout.addWidget(self.connection_state_label, 0, 1)
        
        # 상태
        layout.addWidget(QLabel("로봇 상태:"), 1, 0)
        self.status_label = QLabel("대기 중")
        self.status_label.setStyleSheet("color: #FFA500; font-weight: bold;")
        layout.addWidget(self.status_label, 1, 1)
        
        # 배터리
        layout.addWidget(QLabel("배터리:"), 2, 0)
        self.battery_label = QLabel("0%")
        self.battery_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        layout.addWidget(self.battery_label, 2, 1)
        
        # 모드
        layout.addWidget(QLabel("주행 모드:"), 3, 0)
        mode_combo = QComboBox()
        mode_combo.addItems(["자율주행", "수동조작", "일시정지"])
        layout.addWidget(mode_combo, 3, 1)
        
        group.setLayout(layout)
        return group
    
    def _create_position_section(self):
        """위치 정보 섹션"""
        group = QGroupBox("📍 위치 정보")
        group.setFont(self.main_font)
        layout = QGridLayout()
        
        labels = ["X (m)", "Y (m)", "Theta (deg)"]
        for i, label in enumerate(labels):
            layout.addWidget(QLabel(label), i, 0)
            value_label = QLabel("0.00")
            value_label.setStyleSheet("color: #2196F3; font-weight: bold; font-size: 14px;")
            layout.addWidget(value_label, i, 1)
            self.position_labels[label] = value_label
        
        # 속도
        layout.addWidget(QLabel("선속도 (m/s)"), 3, 0)
        linear_label = QLabel("0.00")
        linear_label.setStyleSheet("color: #9C27B0; font-weight: bold;")
        layout.addWidget(linear_label, 3, 1)
        self.velocity_labels['linear'] = linear_label
        
        layout.addWidget(QLabel("각속도 (rad/s)"), 4, 0)
        angular_label = QLabel("0.00")
        angular_label.setStyleSheet("color: #9C27B0; font-weight: bold;")
        layout.addWidget(angular_label, 4, 1)
        self.velocity_labels['angular'] = angular_label
        
        group.setLayout(layout)
        return group
    
    def _create_navigation_section(self):
        """내비게이션 제어 섹션"""
        group = QGroupBox("🗺️ 내비게이션")
        group.setFont(self.main_font)
        layout = QVBoxLayout()
        
        # 목표 위치 설정
        goal_layout = QGridLayout()
        goal_layout.addWidget(QLabel("목표 X:"), 0, 0)
        self.goal_x_input = QLineEdit("0.0")
        self.goal_x_input.setFixedWidth(100)
        goal_layout.addWidget(self.goal_x_input, 0, 1)
        
        goal_layout.addWidget(QLabel("목표 Y:"), 1, 0)
        self.goal_y_input = QLineEdit("0.0")
        self.goal_y_input.setFixedWidth(100)
        goal_layout.addWidget(self.goal_y_input, 1, 1)
        
        goal_layout.addWidget(QLabel("목표 Theta:"), 2, 0)
        self.goal_theta_input = QLineEdit("0.0")
        self.goal_theta_input.setFixedWidth(100)
        goal_layout.addWidget(self.goal_theta_input, 2, 1)
        
        layout.addLayout(goal_layout)
        
        # 버튼
        btn_layout = QHBoxLayout()
        
        btn_go = QPushButton("🚀 목표로 이동")
        btn_go.setFixedHeight(40)
        btn_go.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        btn_go.clicked.connect(self._send_goal)
        btn_layout.addWidget(btn_go)
        
        btn_stop = QPushButton("⏸ 정지")
        btn_stop.setFixedHeight(40)
        btn_stop.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        btn_stop.clicked.connect(self._stop_navigation)
        btn_layout.addWidget(btn_stop)
        
        layout.addLayout(btn_layout)
        
        # 사전 정의된 위치
        preset_layout = QHBoxLayout()
        for name in ["홈", "충전소", "작업장A", "작업장B"]:
            btn = QPushButton(name)
            btn.setFixedHeight(35)
            btn.clicked.connect(lambda ch, n=name: self._go_to_preset(n))
            preset_layout.addWidget(btn)
        
        layout.addLayout(preset_layout)
        
        group.setLayout(layout)
        return group
    
    def _create_manual_control_section(self):
        """수동 제어 섹션"""
        group = QGroupBox("🎮 수동 제어")
        group.setFont(self.main_font)
        layout = QVBoxLayout()
        
        # 방향 버튼
        direction_grid = QGridLayout()
        direction_grid.setSpacing(10)
        
        btn_forward = QPushButton("⬆ 전진")
        btn_forward.setFixedSize(100, 50)
        btn_forward.pressed.connect(lambda: self._manual_move('forward'))
        btn_forward.released.connect(self._manual_stop)
        direction_grid.addWidget(btn_forward, 0, 1)
        
        btn_left = QPushButton("⬅ 좌회전")
        btn_left.setFixedSize(100, 50)
        btn_left.pressed.connect(lambda: self._manual_move('left'))
        btn_left.released.connect(self._manual_stop)
        direction_grid.addWidget(btn_left, 1, 0)
        
        btn_stop_manual = QPushButton("⏹ 정지")
        btn_stop_manual.setFixedSize(100, 50)
        btn_stop_manual.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold;")
        btn_stop_manual.clicked.connect(self._manual_stop)
        direction_grid.addWidget(btn_stop_manual, 1, 1)
        
        btn_right = QPushButton("➡ 우회전")
        btn_right.setFixedSize(100, 50)
        btn_right.pressed.connect(lambda: self._manual_move('right'))
        btn_right.released.connect(self._manual_stop)
        direction_grid.addWidget(btn_right, 1, 2)
        
        btn_backward = QPushButton("⬇ 후진")
        btn_backward.setFixedSize(100, 50)
        btn_backward.pressed.connect(lambda: self._manual_move('backward'))
        btn_backward.released.connect(self._manual_stop)
        direction_grid.addWidget(btn_backward, 2, 1)
        
        layout.addLayout(direction_grid)
        
        # 속도 설정
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("속도:"))
        self.speed_input = QLineEdit("0.5")
        self.speed_input.setFixedWidth(80)
        speed_layout.addWidget(self.speed_input)
        speed_layout.addWidget(QLabel("m/s"))
        speed_layout.addStretch()
        layout.addLayout(speed_layout)
        
        group.setLayout(layout)
        return group
    
    # ===== 제어 메서드 =====
    
    def _send_goal(self):
        """목표 위치로 이동"""
        if self.controller:
            x = float(self.goal_x_input.text())
            y = float(self.goal_y_input.text())
            theta = float(self.goal_theta_input.text())
            print(f"🚀 목표 설정: X={x}, Y={y}, Theta={theta}")
            # TODO: controller.send_goal(x, y, theta)
    
    def _stop_navigation(self):
        """내비게이션 정지"""
        if self.controller:
            print("⏸ 내비게이션 정지")
            # TODO: controller.cancel_goal()
    
    def _go_to_preset(self, preset_name):
        """사전 정의 위치로 이동"""
        presets = {
            "홈": (0.0, 0.0, 0.0),
            "충전소": (5.0, 2.0, 90.0),
            "작업장A": (10.0, 5.0, 0.0),
            "작업장B": (15.0, 8.0, 180.0),
        }
        
        if preset_name in presets:
            x, y, theta = presets[preset_name]
            self.goal_x_input.setText(str(x))
            self.goal_y_input.setText(str(y))
            self.goal_theta_input.setText(str(theta))
            self._send_goal()
    
    def _manual_move(self, direction):
        """수동 이동"""
        if self.controller:
            speed = float(self.speed_input.text())
            print(f"🎮 수동 이동: {direction}, 속도={speed}")
            # TODO: controller.send_velocity(direction, speed)
    
    def _manual_stop(self):
        """수동 정지"""
        if self.controller:
            print("⏹ 수동 정지")
            # TODO: controller.stop()
    
    # ===== 업데이트 메서드 =====
    
    def update_position(self, x, y, theta):
        """위치 업데이트"""
        self.position_labels["X (m)"].setText(f"{x:.2f}")
        self.position_labels["Y (m)"].setText(f"{y:.2f}")
        self.position_labels["Theta (deg)"].setText(f"{theta:.2f}")
    
    def update_velocity(self, linear, angular):
        """속도 업데이트"""
        self.velocity_labels['linear'].setText(f"{linear:.2f}")
        self.velocity_labels['angular'].setText(f"{angular:.2f}")
    
    def update_battery(self, percentage):
        """배터리 업데이트"""
        self.battery_label.setText(f"{percentage}%")
        
        # 배터리 잔량에 따른 색상 변경
        if percentage > 50:
            color = "#4CAF50"  # 녹색
        elif percentage > 20:
            color = "#FFA500"  # 주황
        else:
            color = "#f44336"  # 빨강
        
        self.battery_label.setStyleSheet(f"color: {color}; font-weight: bold;")
    
    def update_status(self, status):
        """상태 업데이트"""
        self.status_label.setText(status)
        
        # 상태에 따른 색상
        status_colors = {
            "주행 중": "#4CAF50",
            "대기 중": "#FFA500",
            "오류": "#f44336",
            "충전 중": "#2196F3",
        }
        
        color = status_colors.get(status, "#757575")
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def set_connection_state(self, ros_connected=None, ping_connected=None, camera_connected=None):
        """시스템 상태 섹션 연결 상태 표시 갱신"""
        if not self.connection_state_label:
            return

        if ros_connected is True:
            text = "ROS Online"
            color = "#1e7e34"
        elif ping_connected is True:
            text = "Ping Online"
            color = "#ef6c00"
        elif ros_connected is False or ping_connected is False:
            text = "Offline"
            color = "#b71c1c"
        else:
            text = "Unknown"
            color = "#616161"

        if camera_connected is True:
            text = f"{text} | CAM"
        elif camera_connected is False:
            text = f"{text} | No CAM"

        self._set_connection_status_text(text, color)

    def _set_connection_status_text(self, text, color):
        self.connection_state_label.setText(f"●  {text}")
        self.connection_state_label.setStyleSheet(
            f"font-size: 14px; color: {color}; font-weight: bold;"
        )
