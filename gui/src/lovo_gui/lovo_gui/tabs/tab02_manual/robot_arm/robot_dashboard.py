"""
로봇팔 대시보드 위젯
"""
import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QGridLayout, QScrollArea, QLineEdit, QCheckBox, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal, QUrl
from PyQt6.QtGui import QFont, QImage, QPixmap
from pathlib import Path
from .pose_memory_manager import PoseMemoryManager
from .sdk_control import SDKControl
from .moveit_control import MoveItControl




# --- 공통 스타일 상수 ---
MEMORY_SLOT_COUNT = 8  # 메모리 슬롯 개수 (P1~P5, 필요시 8로 변경)

# --- UI Widths (px) ---
AXIS_LABEL_WIDTH = 60      # 축 라벨 너비 (Axis column)
JOG_ZONE_WIDTH = 0       # Jog 버튼 영역 너비 (그리드의 플레이스홀더 너비)
TARGET_TO_P_WIDTH = 70     # Target, Actual, Error, P1..Pn 공통 너비
ABS_INPUT_WIDTH = 70       # 절대 좌표/각도 입력 너비
DELTA_INPUT_WIDTH = 40     # Jog delta 입력 너비
MEM_BUTTON_WIDTH = 75      # 메모리 버튼 너비
MOVE_BTN_WIDTH = 120       # 큰 Move 버튼 너비

# --- UI Heights (px) ---
INPUT_HEIGHT = 32          # 일반 입력창 높이
BUTTON_HEIGHT = 32         # 일반 버튼 높이
MOVE_BTN_HEIGHT = 45       # 큰 Move 버튼 높이

# --- 기타 ---
DATA_COL_WIDTH = TARGET_TO_P_WIDTH   # alias for backward compat: data column width
UI_SPACING = 5        # 레이아웃 간격 (setSpacing 값)


class RobotDashboardWidget(QWidget):
    """로봇팔 제어 대시보드 위젯"""
    work_log_signal = pyqtSignal(str)  # 작업 로그 메시지

    def __init__(self, robot_name, robot_key, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_key = robot_key

        # 메모리 (슬롯 개수 변수화)
        self.pose_memory = {i: [0.0]*6 for i in range(1, MEMORY_SLOT_COUNT+1)}
        self.pose_mem_labels = {i: [None]*6 for i in range(1, MEMORY_SLOT_COUNT+1)}

        # Target, Actual, Error, Delta, Absolute 입력 필드
        self.pose_target_inputs = [None] * 6  # Target 입력 필드 (읽기 전용)
        self.pose_actual_labels = [None] * 6
        self.pose_error_labels = [None] * 6
        self.pose_delta_inputs = [None] * 6
        self.absolute_coord_inputs = [None] * 6  # 절대 좌표 이동 입력 필드
        self.absolute_angle_inputs = [None] * 6  # 엔코더 각도 출력 필드

        # 기본 폰트
        self.main_font = QFont("Arial", 9)

        # MoveIt 사용 여부
        self.use_moveit = False

        # Pose memory manager 초기화
        self.pose_memory_manager = PoseMemoryManager(slot_count=MEMORY_SLOT_COUNT)
        # 연결된 GUI 로그로 메시지를 보낼 수 있도록 로거를 설정
        try:
            self.pose_memory_manager.set_logger(self.work_log_signal.emit)
        except Exception:
            pass

        # UI 초기화
        self._setup_ui()
    
    def set_controller(self, controller):
        """컨트롤러 설정 및 Signal 연결"""
        self.controller = controller
        
        # 제어 클래스 초기화
        self.sdk_control = SDKControl(controller)
        self.moveit_control = MoveItControl(controller)
        
        # RobotArmController Signal 연결
        controller.coords_updated.connect(self.update_coords_display)
        controller.pose_updated.connect(self.update_pose_display)
        controller.angles_updated.connect(self.update_angles_display)
    
    def _setup_ui(self):
        """UI 구성"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        main_layout.setSpacing(UI_SPACING)
        
        # 제어 영역 (스크롤)
        control_widget = self._create_control_section()
        main_layout.addWidget(control_widget)
    
    def _create_control_section(self):
        """제어 섹션"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        # Ensure scroll area has a reasonable minimum size so inner widgets remain visible
        scroll.setMinimumWidth(820)
        scroll.setMinimumHeight(520)
        scroll_content = QWidget()
        # Ensure scroll content expands inside the scroll area
        scroll_content.setMinimumWidth(800)
        scroll_content.setMinimumHeight(520)
        scroll_content.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        robot_layout = QVBoxLayout(scroll_content)
        robot_layout.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        robot_layout.setSpacing(UI_SPACING)
        
        # 1. System Control
        sys_group = self._create_system_control()
        # Make system group visible with a reasonable minimum width
        sys_group.setMinimumWidth(600)
        sys_group.setMinimumHeight(80)
        sys_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        robot_layout.addWidget(sys_group)
        
        # 2. 좌표 컨트롤러
        cart_group = self._create_cartesian_controller()
        # Ensure cartesian controller expands and has minimum usable size
        cart_group.setMinimumWidth(760)
        cart_group.setMinimumHeight(420)
        cart_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        robot_layout.addWidget(cart_group)
        
        robot_layout.addStretch()
        
        scroll.setWidget(scroll_content)
        return scroll
    
    def _setup_grid_alignment(self, layout):
        """그리드 정렬 설정"""
        layout.setColumnMinimumWidth(0, AXIS_LABEL_WIDTH)
        # Target column
        layout.setColumnMinimumWidth(1, TARGET_TO_P_WIDTH)
        # Actual / Error columns
        layout.setColumnMinimumWidth(2, DATA_COL_WIDTH)
        layout.setColumnMinimumWidth(3, DATA_COL_WIDTH)
        # P1..Pn columns start at index 4
        for col in range(4, 4 + MEMORY_SLOT_COUNT):
            layout.setColumnMinimumWidth(col, DATA_COL_WIDTH)
        # ...existing code...
    
    def _create_label(self, text, color, width):
        """라벨 생성"""
        lbl = QLabel(text)
        lbl.setFont(self.main_font)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(
            f"border: 2px solid {color}; background-color: white; color: black; border-radius: 3px;"
        )
        # 고정 너비 적용 (너비 파라미터는 픽셀)
        if width and isinstance(width, int) and width > 0:
            lbl.setFixedWidth(width)
        return lbl
    
    def _create_system_control(self):
        """시스템 컨트롤"""
        sys_group = QGroupBox("⚙️ System Control")
        sys_group.setFixedHeight(90)

        sys_h_layout = QHBoxLayout()
        controls = [
            ("✓ Servo ON", self._servo_on),
            ("✗ Servo OFF", self._servo_off),
            ("🏠 HOME", self._go_home),
            ("✊ GRIP", self._grip),
            ("🖐️ UNGRIP", self._ungrip)
        ]
        for text, func in controls:
            btn = QPushButton(text)
            btn.setFixedSize(105, 40)
            btn.setFont(QFont("Arial", 9, QFont.Weight.Bold))
            btn.clicked.connect(func)
            sys_h_layout.addWidget(btn)
        sys_h_layout.addStretch()
        sys_group.setLayout(sys_h_layout)
        return sys_group
    

    def _create_cartesian_controller(self):
        """좌표 컨트롤러"""
        cart_group = QGroupBox()
        main_layout = QVBoxLayout()
        
        # 헤더 라인: 타이틀 + MoveIt 체크박스
        header_layout = QHBoxLayout()
        
        title_label = QLabel("🎯 좌표 컨트롤러 (Cartesian Controller)")
        title_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
        # MoveIt 토글 버튼 (체크박스 대신 버튼 사용)
        self.moveit_toggle_btn = QPushButton("MoveIt: OFF")
        self.moveit_toggle_btn.setFixedSize(120, 35)
        self.moveit_toggle_btn.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.moveit_toggle_btn.setCheckable(True)
        self.moveit_toggle_btn.setChecked(self.use_moveit)
        self._update_moveit_button_style()
        self.moveit_toggle_btn.toggled.connect(self._on_moveit_toggled)
        header_layout.addWidget(self.moveit_toggle_btn)
        
        main_layout.addLayout(header_layout)
        
        # 그리드 레이아웃
        c_grid = QGridLayout()
        c_grid.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        c_grid.setSpacing(UI_SPACING)
        self._setup_grid_alignment(c_grid)
        
        # 헤더
        # Create header labels and keep references for P columns so we can
        # update names loaded from CSV (Label column)
        base_headers = ["Axis", "Target", "Actual", "Error"]
        self.p_header_labels = {}
        for col, text in enumerate(base_headers):
            hdr = QLabel(text)
            hdr.setFont(self.main_font)
            hdr.setAlignment(Qt.AlignmentFlag.AlignCenter)
            c_grid.addWidget(hdr, 0, col, Qt.AlignmentFlag.AlignCenter)

        # Create P headers with default names (P1..Pn) and store QLabel refs
        for idx in range(1, MEMORY_SLOT_COUNT + 1):
            pcol = 3 + idx  # columns: 0..3 used, P1 starts at 4 (3+1)
            hdr = QLabel(f"P{idx}")
            hdr.setFont(self.main_font)
            hdr.setAlignment(Qt.AlignmentFlag.AlignCenter)
            c_grid.addWidget(hdr, 0, pcol, Qt.AlignmentFlag.AlignCenter)
            self.p_header_labels[idx] = hdr

        axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]
        for i in range(6):
            row = i + 1

            # 축 라벨
            axis_lbl = QLabel(axes[i])
            axis_lbl.setFixedWidth(AXIS_LABEL_WIDTH)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_lbl.setStyleSheet(
                "border: 2px solid #666; background-color: #E0E0E0; color: black; border-radius: 3px; padding: 4px;"
            )
            c_grid.addWidget(axis_lbl, row, 0)

            # Target 입력 필드 (읽기 전용)
            target_input = QLineEdit("0.0")
            target_input.setFixedWidth(TARGET_TO_P_WIDTH)
            target_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            target_input.setReadOnly(True)  # 읽기 전용
            target_input.setStyleSheet(
                "background-color: #F5F5F5; color: #555; "
                "border: 2px solid #BDBDBD; border-radius: 3px; padding: 2px;"
            )
            self.pose_target_inputs[i] = target_input
            c_grid.addWidget(target_input, row, 1)

            actual_lbl = self._create_label("0.0", "#757575", DATA_COL_WIDTH)
            self.pose_actual_labels[i] = actual_lbl
            c_grid.addWidget(actual_lbl, row, 2)

            # Error
            err_lbl = self._create_label("0.0", "#f44336", DATA_COL_WIDTH)
            self.pose_error_labels[i] = err_lbl
            c_grid.addWidget(err_lbl, row, 3)

            # 메모리 Pos1~N
            for m_idx in range(1, MEMORY_SLOT_COUNT+1):
                mem_lbl = self._create_label("---", "#555", DATA_COL_WIDTH)
                self.pose_mem_labels[m_idx][i] = mem_lbl
                c_grid.addWidget(mem_lbl, row, m_idx + 3)

        # 좌표 메모리 저장/이동 버튼 라인
        for m_idx in range(1, MEMORY_SLOT_COUNT+1):
            btn_vbox = QVBoxLayout()
            btn_vbox.setSpacing(UI_SPACING)
            btn_vbox.setContentsMargins(0, 0, 0, 0)

            ps_btn = QPushButton("저장")
            ps_btn.setFixedSize(MEM_BUTTON_WIDTH - 5, BUTTON_HEIGHT)
            ps_btn.setStyleSheet("font-size: 10px; background-color: #1976D2;")
            ps_btn.clicked.connect(lambda ch, m=m_idx: self._save_pose_memory(m))

            pm_btn = QPushButton("이동")
            pm_btn.setFixedSize(MEM_BUTTON_WIDTH - 5, BUTTON_HEIGHT)
            pm_btn.setStyleSheet("font-size: 10px; background-color: #2E7D32;")
            pm_btn.clicked.connect(lambda ch, m=m_idx: self._move_pose_memory(m))

            btn_vbox.addWidget(ps_btn)
            btn_vbox.addWidget(pm_btn)
            c_grid.addLayout(btn_vbox, 7, m_idx + 3)
        
        main_layout.addLayout(c_grid)
        
        # ==================== 절대 좌표/각도 이동 섹션 ====================
        # 수평 레이아웃으로 절대 좌표와 절대 각도를 나란히 배치
        abs_control_layout = QHBoxLayout()
        abs_control_layout.setSpacing(UI_SPACING)

        # ==================== Jog Zone (왼쪽) ====================
        jog_group = QGroupBox("🕹️ Jog Zone")
        jog_group.setMinimumWidth(JOG_ZONE_WIDTH)
        jog_group.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        jog_layout = QVBoxLayout()
        jog_layout.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        jog_layout.setSpacing(UI_SPACING)
        jog_name_lbl = QLabel(f"{self.robot_name} Jog Zone")
        jog_name_lbl.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        jog_name_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        jog_layout.addWidget(jog_name_lbl)
        # Add a visible label for the Jog controls relocated here
        jog_header_lbl = QLabel("Jog (+/-)")
        jog_header_lbl.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        jog_header_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        jog_header_lbl.setStyleSheet("color: #333; padding-bottom: 4px;")
        jog_layout.addWidget(jog_header_lbl)

        # 각 축별로 Jog(-) [delta edit] Jog(+) 컨트롤을 이 영역에 배치
        for j in range(6):
            row_h = QHBoxLayout()
            row_h.setSpacing(UI_SPACING)
            row_h.setContentsMargins(0, 0, 0, 0)

            jbtn_m = QPushButton("-")
            jbtn_m.setFixedSize(32, 32)
            jbtn_m.clicked.connect(lambda ch, idx=j: self._cart_jog(idx, -1))

            delta_input = QLineEdit("5.0")
            delta_input.setFixedSize(DELTA_INPUT_WIDTH, INPUT_HEIGHT)
            delta_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.pose_delta_inputs[j] = delta_input

            jbtn_p = QPushButton("+")
            jbtn_p.setFixedSize(32, 32)
            jbtn_p.clicked.connect(lambda ch, idx=j: self._cart_jog(idx, 1))

            row_h.addWidget(jbtn_m)
            row_h.addWidget(delta_input)
            row_h.addWidget(jbtn_p)
            row_h.addStretch()
            jog_layout.addLayout(row_h)

        jog_layout.addStretch()
        jog_group.setLayout(jog_layout)
        abs_control_layout.addWidget(jog_group)

        # ==================== 절대 좌표 이동 섹션 ====================
        abs_coord_group = QGroupBox("📍 절대 좌표 이동 (Absolute Coordinate Movement)")
        abs_coord_layout = QVBoxLayout()
        abs_coord_layout.setSpacing(UI_SPACING)
        abs_coord_layout.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        
        # 라벨과 입력칸을 가로로 정렬
        coord_axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]
        
        for i in range(6):
            # 각 좌표별 수평 레이아웃
            coord_h_layout = QHBoxLayout()
            coord_h_layout.setSpacing(UI_SPACING)
            coord_h_layout.setContentsMargins(UI_SPACING, 0, UI_SPACING, 0)
            
            # 라벨 (좌표컨트롤러와 동일한 스타일)
            axis_lbl = QLabel(coord_axes[i])
            axis_lbl.setFixedWidth(AXIS_LABEL_WIDTH)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_lbl.setStyleSheet(
                "border: 2px solid #666; background-color: #E0E0E0; color: black; border-radius: 3px; padding: 4px;"
            )
            coord_h_layout.addWidget(axis_lbl)
            
            # 입력 필드 (좌표컨트롤러의 Target과 동일한 스타일)
            input_field = QLineEdit("0.0")
            input_field.setFixedWidth(ABS_INPUT_WIDTH)
            input_field.setAlignment(Qt.AlignmentFlag.AlignCenter)
            input_field.setStyleSheet(
                "background-color: #E3F2FD; color: black; "
                "border: 2px solid #2196F3; border-radius: 3px; padding: 2px;"
            )
            self.absolute_coord_inputs[i] = input_field
            coord_h_layout.addWidget(input_field)
            
            coord_h_layout.addStretch()
            abs_coord_layout.addLayout(coord_h_layout)
        
        abs_coord_group.setLayout(abs_coord_layout)
        abs_control_layout.addWidget(abs_coord_group)
        
        # ==================== 현재 엔코더 각도 표시 섹션 ====================
        abs_angle_group = QGroupBox("🔧 현재 엔코더 각도 (Current Encoder Angles)")
        abs_angle_layout = QVBoxLayout()
        abs_angle_layout.setSpacing(UI_SPACING)
        abs_angle_layout.setContentsMargins(UI_SPACING, UI_SPACING, UI_SPACING, UI_SPACING)
        
        # 라벨과 출력칸을 가로로 정렬
        angle_axes = ["J1(°)", "J2(°)", "J3(°)", "J4(°)", "J5(°)", "J6(°)"]
        
        for i in range(6):
            # 각 각도별 수평 레이아웃
            angle_h_layout = QHBoxLayout()
            angle_h_layout.setSpacing(UI_SPACING)
            angle_h_layout.setContentsMargins(UI_SPACING, 0, UI_SPACING, 0)
            
            # 라벨
            axis_lbl = QLabel(angle_axes[i])
            axis_lbl.setFixedWidth(AXIS_LABEL_WIDTH)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_lbl.setStyleSheet(
                "border: 2px solid #666; background-color: #E0E0E0; color: black; border-radius: 3px; padding: 4px;"
            )
            angle_h_layout.addWidget(axis_lbl)
            
            # 출력 필드 (읽기 전용)
            input_field = QLineEdit("0.0")
            input_field.setFixedWidth(ABS_INPUT_WIDTH)
            input_field.setAlignment(Qt.AlignmentFlag.AlignCenter)
            input_field.setReadOnly(True)
            input_field.setStyleSheet(
                "background-color: #F5F5F5; color: #333; "
                "border: 2px solid #BDBDBD; border-radius: 3px; padding: 2px;"
            )
            self.absolute_angle_inputs[i] = input_field
            angle_h_layout.addWidget(input_field)
            
            angle_h_layout.addStretch()
            abs_angle_layout.addLayout(angle_h_layout)
        
        abs_angle_group.setLayout(abs_angle_layout)
        abs_control_layout.addWidget(abs_angle_group)
        
        main_layout.addLayout(abs_control_layout)
        
        # Move 버튼 추가
        move_btn_layout = QHBoxLayout()
        move_btn_layout.addStretch()
        
        move_btn = QPushButton("🎯 이동")
        move_btn.setFixedSize(120, 45)
        move_btn.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        move_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; "
            "border-radius: 5px; border: 2px solid #45a049; } "
            "QPushButton:hover { background-color: #45a049; } "
            "QPushButton:pressed { background-color: #3d8b40; }"
        )
        move_btn.clicked.connect(self._move_to_target)
        move_btn_layout.addWidget(move_btn)
        move_btn_layout.addStretch()
        
        main_layout.addLayout(move_btn_layout)
        cart_group.setLayout(main_layout)
        return cart_group
    
    # ==================== 데이터 업데이트 메서드 ====================
    
    def update_coords_display(self, coords):
        """좌표 데이터 업데이트 (엔코더값 = Actual)"""
        for i in range(6):
            if self.pose_actual_labels[i] and len(coords) > i:
                # Actual: 엔코더값 표시 (안정성 강화)
                actual_value = coords[i] if isinstance(coords[i], (int, float)) else 0.0
                self.pose_actual_labels[i].setText(f"{actual_value:.1f}")
                
                # 오차 계산: Target과 Actual의 차이
                try:
                    target = float(self.pose_target_inputs[i].text())
                    error = target - actual_value
                    self.pose_error_labels[i].setText(f"{error:.1f}")
                    
                    # 오차 범위에 따른 색상 변경
                    color = "#C8E6C9" if abs(error) < 1.0 else "#FFCDD2"
                    self.pose_error_labels[i].setStyleSheet(
                        f"background-color: {color}; color: black; "
                        f"border: 1px solid #f44336; border-radius: 3px;"
                    )
                except ValueError:
                    pass
    
    def update_pose_display(self, pose):
        """포즈 데이터 업데이트 (current_pose)"""
        # pose_memory 저장용
        pass
    
    def update_angles_display(self, angles):
        """엔코더 각도 데이터 업데이트"""
        for i in range(6):
            if self.absolute_angle_inputs[i] and len(angles) > i:
                angle_value = angles[i] if isinstance(angles[i], (int, float)) else 0.0
                self.absolute_angle_inputs[i].setText(f"{angle_value:.2f}")
    
    # ==================== 제어 메서드 ====================
    
    def _on_moveit_changed(self, state):
        """MoveIt 사용 여부 변경"""
        self.use_moveit = (state == Qt.CheckState.Checked.value)
        mode = "MoveIt" if self.use_moveit else "Direct"
        self.work_log_signal.emit(f"🎯 {self.robot_name} 좌표 제어 모드: {mode}")
    
    def _update_moveit_button_style(self):
        """MoveIt 토글 버튼 스타일 업데이트"""
        if self.moveit_toggle_btn.isChecked():
            self.moveit_toggle_btn.setText("MoveIt: ON")
            self.moveit_toggle_btn.setStyleSheet(
                "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; "
                "border: 2px solid #2E7D32; border-radius: 5px; }"
                "QPushButton:hover { background-color: #45a049; }"
            )
        else:
            self.moveit_toggle_btn.setText("MoveIt: OFF")
            self.moveit_toggle_btn.setStyleSheet(
                "QPushButton { background-color: #BDBDBD; color: white; font-weight: bold; "
                "border: 2px solid #757575; border-radius: 5px; }"
                "QPushButton:hover { background-color: #A0A0A0; }"
            )
    
    def _on_moveit_toggled(self, checked):
        """MoveIt 토글 버튼 클릭 이벤트"""
        self.use_moveit = checked
        self._update_moveit_button_style()
        mode = "MoveIt" if checked else "Direct"
        self.work_log_signal.emit(f"🎯 {self.robot_name} 좌표 제어 모드: {mode}")
    
    def _cart_jog(self, axis, direction):
        """좌표 Jog"""
        if not self.controller:
            return
        
        try:
            delta = float(self.pose_delta_inputs[axis].text())
            current = list(self.controller.current_coords)
            current[axis] += (direction * delta)
            
            # 제어 클래스 선택
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.move_jog_cartesian(axis, delta * direction)
            
            # Target 필드 업데이트
            self.pose_target_inputs[axis].setText(f"{current[axis]:.1f}")
        except:
            pass
    
    def _servo_on(self):
        if self.controller:
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.set_servo(True)
            self.work_log_signal.emit(f"✅ {self.robot_name} Servo ON - 명령 전송됨")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _servo_off(self):
        if self.controller:
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.set_servo(False)
            self.work_log_signal.emit(f"❌ {self.robot_name} Servo OFF - 명령 전송됨")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _go_home(self):
        if self.controller:
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.go_home()
            self.work_log_signal.emit(f"🏠 {self.robot_name} HOME 위치로 이동")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _grip(self):
        if self.controller:
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.set_gripper(1)
            self.work_log_signal.emit(f"✊ {self.robot_name} GRIP")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _ungrip(self):
        if self.controller:
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.set_gripper(0)
            self.work_log_signal.emit(f"🖐️ {self.robot_name} UNGRIP")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _move_to_target(self):
        """절대 좌표로 이동"""
        if not self.controller:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
            return
        
        try:
            # 절대 좌표 입력값 읽기
            target_coords = []
            for i in range(6):
                value = float(self.absolute_coord_inputs[i].text())
                target_coords.append(value)
            
            # 제어 클래스 선택
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.move_absolute_cartesian(target_coords)
            
            # Target 필드도 업데이트 (읽기 전용이지만 프로그램에서는 변경 가능)
            for i in range(6):
                self.pose_target_inputs[i].setText(f"{target_coords[i]:.1f}")
            
            coords_str = f"[{', '.join([f'{v:.1f}' for v in target_coords])}]"
            self.work_log_signal.emit(f"🎯 {self.robot_name} 절대 좌표로 이동: {coords_str}")
            
        except ValueError as e:
            self.work_log_signal.emit(f"❌ {self.robot_name} 잘못된 좌표 입력값입니다!")
        except Exception as e:
            self.work_log_signal.emit(f"❌ {self.robot_name} 이동 실패: {str(e)}")

    def _save_pose_memory(self, slot):
        """좌표 메모리 저장"""
        if self.controller:
            self.pose_memory[slot] = list(self.controller.current_coords)
            for i in range(6):
                self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i]:.1f}")
            # 파일에 저장
            self.pose_memory_manager.save(
                self.robot_name, 
                slot, 
                self.pose_memory[slot]
            )
            self.work_log_signal.emit(f"💾 {self.robot_name} P{slot} 저장 완료")
    
    def load_pose_memory(self):
        """저장된 좌표 메모리 로드"""
        saved_memory, labels = self.pose_memory_manager.load_with_labels(self.robot_name)
        for slot in range(1, MEMORY_SLOT_COUNT+1):
            slot_str = str(slot)
            if slot_str in saved_memory:
                self.pose_memory[slot] = saved_memory[slot_str]
                # UI 업데이트
                if all(self.pose_mem_labels[slot]):
                    for i in range(6):
                        self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i]:.1f}")

            # Update header label for this P slot if available
            if hasattr(self, 'p_header_labels') and slot in labels:
                try:
                    self.p_header_labels[slot].setText(labels[slot])
                except Exception:
                    pass

        self.work_log_signal.emit(f"📂 {self.robot_name} 좌표 메모리 로드 완료")
    
    def _move_pose_memory(self, slot):
        """좌표 메모리 이동"""
        if self.controller:
            # 제어 클래스 선택
            control = self.moveit_control if self.use_moveit else self.sdk_control
            control.move_absolute_cartesian(self.pose_memory[slot])
            coords_str = f"[{', '.join([f'{v:.1f}' for v in self.pose_memory[slot]])}]"
            self.work_log_signal.emit(f"🎯 {self.robot_name} P{slot} 위치로 이동: {coords_str}")
            # Target 값을 P1~N의 저장된 좌표로 변경
            for i in range(6):
                self.pose_target_inputs[i].setText(f"{self.pose_memory[slot][i]:.1f}")
    
    def _create_pose_memory_buttons(self):
        """좌표 메모리 저장/이동 버튼"""
        pose_mem_btn_layout = QHBoxLayout()
        for m in range(1, MEMORY_SLOT_COUNT+1):
            btn_v_layout = QVBoxLayout()
            btn_v_layout.setSpacing(2)

            save_btn = QPushButton("저장")
            save_btn.setFixedSize(55, 22)
            save_btn.setStyleSheet("font-size: 9px; background-color: #1976D2;")

            move_btn = QPushButton("이동")
            move_btn.setFixedSize(55, 22)
            move_btn.setStyleSheet("font-size: 9px; background-color: #2E7D32;")

            btn_v_layout.addWidget(save_btn)
            btn_v_layout.addWidget(move_btn)
            pose_mem_btn_layout.addLayout(btn_v_layout)
        pose_mem_btn_layout.addStretch()
        return pose_mem_btn_layout

