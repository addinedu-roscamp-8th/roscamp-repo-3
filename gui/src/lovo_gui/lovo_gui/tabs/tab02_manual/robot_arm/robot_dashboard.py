"""
로봇팔 대시보드 위젯
"""
import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QGridLayout, QScrollArea, QLineEdit, QCheckBox, QSizePolicy, QDoubleSpinBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QUrl
from PyQt6.QtGui import QFont, QImage, QPixmap
from pathlib import Path
from .pose_memory_manager import PoseMemoryManager
from .sdk_control import SDKControl




# --- 공통 스타일 상수 ---
MEMORY_SLOT_COUNT = 8  # 메모리 슬롯 개수 (P1~P5, 필요시 8로 변경)

# --- UI Widths (px) ---
AXIS_LABEL_WIDTH = 60      # 축 라벨 너비 (Axis column)
TARGET_TO_P_WIDTH = 70     # Target, Actual, Error, P1..Pn 공통 너비
ABS_INPUT_WIDTH = 70       # 각도 출력 입력 너비
MEM_BUTTON_WIDTH = 75      # 메모리 버튼 너비

# --- UI Heights (px) ---
INPUT_HEIGHT = 32          # 일반 입력창 높이
BUTTON_HEIGHT = 32         # 일반 버튼 높이

# --- 기타 ---
DATA_COL_WIDTH = TARGET_TO_P_WIDTH   # alias for backward compat: data column width
UI_SPACING = 5        # 레이아웃 간격 (setSpacing 값)

# --- 단위 변환 (UI 표시용: 들어오는 값은 이미 cm, 표시만 'cm'으로 변경) ---
LENGTH_UNIT = "cm"
# No scaling: incoming numbers are already in cm and will be used as-is
SCALE_MM_TO_DISPLAY = 1.0
SCALE_DISPLAY_TO_MM = 1.0


class RobotDashboardWidget(QWidget):
    """로봇팔 제어 대시보드 위젯"""
    work_log_signal = pyqtSignal(str)  # 작업 로그 메시지

    def __init__(self, robot_name, robot_key, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_key = robot_key
        self.controller = None
        self.connection_state_label = None

        # 메모리 (슬롯 개수 변수화)
        self.pose_memory = {i: [0.0]*6 for i in range(1, MEMORY_SLOT_COUNT+1)}
        self.pose_mem_labels = {i: [None]*6 for i in range(1, MEMORY_SLOT_COUNT+1)}

        # Target, Actual, Error 입력 필드
        self.pose_target_inputs = [None] * 6  # Target 입력 필드 (읽기 전용)
        self.pose_actual_labels = [None] * 6
        self.pose_error_labels = [None] * 6
        self.absolute_angle_inputs = [None] * 6  # 엔코더 각도 출력 필드

        # 기본 폰트
        self.main_font = QFont("Arial", 9)

        # MoveIt removed — always use SDK control

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
        # MoveItControl removed; always use SDKControl
        
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
        
        # 1. System Control + Connection Status (same row)
        top_row_layout = QHBoxLayout()
        top_row_layout.setSpacing(UI_SPACING)

        sys_group = self._create_system_control()
        # Make system group visible with a reasonable minimum width
        sys_group.setMinimumWidth(600)
        sys_group.setMinimumHeight(80)
        sys_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        top_row_layout.addWidget(sys_group, 1)

        conn_group = self._create_connection_status_group()
        conn_group.setMinimumHeight(80)
        conn_group.setFixedWidth(300)
        conn_group.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        top_row_layout.addWidget(conn_group, 0)

        robot_layout.addLayout(top_row_layout)
        
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

    def _create_connection_status_group(self):
        """System Control 옆 통신 상태 그룹"""
        conn_group = QGroupBox("통신 상태")
        conn_group.setFixedHeight(90)
        conn_group.setFixedWidth(300)

        layout = QVBoxLayout()
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)

        self.connection_state_label = QLabel()
        self.connection_state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._set_connection_status_text("Unknown", "#616161")

        layout.addStretch()
        layout.addWidget(self.connection_state_label)
        layout.addStretch()
        conn_group.setLayout(layout)
        return conn_group

    def set_connection_state(self, ros_connected=None, ping_connected=None, camera_connected=None):
        """시스템 컨트롤 영역 연결 상태 표시 갱신"""
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
    

    def _create_cartesian_controller(self):
        """좌표 컨트롤러"""
        cart_group = QGroupBox()
        main_layout = QVBoxLayout()
        
        # 헤더 라인: 타이틀
        header_layout = QHBoxLayout()
        
        title_label = QLabel("🎯 좌표 컨트롤러 (Cartesian Controller)")
        title_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
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

        axes = [f"X({LENGTH_UNIT})", f"Y({LENGTH_UNIT})", f"Z({LENGTH_UNIT})", "R(°)", "P(°)", "Y(°)"]
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

        # ==================== Order Command UI ====================
        order_group = QGroupBox("📦 Order Command")
        order_group.setStyleSheet("""
            QGroupBox { 
                font-size: 11px; 
                font-weight: bold;
                padding-top: 10px; 
                margin-top: 5px; 
            }
        """)
        order_layout = QHBoxLayout()
        order_layout.setContentsMargins(5, 5, 5, 5)
        
        # 라벨
        order_layout.addWidget(QLabel("Value:"))
        
        # 실수 입력 (SpinBox)
        self.order_input = QDoubleSpinBox()
        self.order_input.setRange(-9999.0, 9999.0)
        self.order_input.setDecimals(4)
        self.order_input.setSingleStep(0.0001)
        self.order_input.setValue(0.0)
        self.order_input.setFixedWidth(100)
        self.order_input.setFixedHeight(30)
        self.order_input.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #555;
                background-color: #2d2d2d;
                color: white;
                padding: 2px;
            }
        """)
        order_layout.addWidget(self.order_input)
        
        # 전송 버튼
        self.btn_send_order = QPushButton("📤 Send Order")
        self.btn_send_order.setFixedSize(100, 30)
        self.btn_send_order.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3e8e41;
            }
        """)
        self.btn_send_order.clicked.connect(self._send_order_command)
        order_layout.addWidget(self.btn_send_order)
        
        order_layout.addStretch()
        order_group.setLayout(order_layout)
        main_layout.addWidget(order_group)
        # ==========================================================
        
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
        main_layout.addWidget(abs_angle_group)
        cart_group.setLayout(main_layout)
        return cart_group
    
    # ==================== 데이터 업데이트 메서드 ====================
    
    def update_coords_display(self, coords):
        """좌표 데이터 업데이트 (엔코더값 = Actual)"""
        for i in range(6):
            if self.pose_actual_labels[i] and len(coords) > i:
                # Actual: 엔코더값(mm) -> UI 표시 단위로 변환 (cm)
                actual_value_mm = coords[i] if isinstance(coords[i], (int, float)) else 0.0
                actual_display = actual_value_mm * SCALE_MM_TO_DISPLAY
                self.pose_actual_labels[i].setText(f"{actual_display:.2f}")

                # 오차 계산: Target(UI단위=cm)과 Actual(UI단위=cm)의 차이
                try:
                    target_display = float(self.pose_target_inputs[i].text())
                    error_display = target_display - actual_display
                    self.pose_error_labels[i].setText(f"{error_display:.2f}")

                    # 오차 범위에 따른 색상 변경 (단위: cm)
                    color = "#C8E6C9" if abs(error_display) < 1.0 else "#FFCDD2"
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
            try:
                if self.absolute_angle_inputs[i] and len(angles) > i:
                    angle_value = float(angles[i])
                    self.absolute_angle_inputs[i].setText(f"{angle_value:.2f}")
            except Exception:
                # ignore individual conversion errors to keep UI responsive
                continue
    
    # ==================== 제어 메서드 ====================
    
    def _servo_on(self):
        if self.controller:
            try:
                self.controller.send_servo(True)
                self.work_log_signal.emit(f"✅ {self.robot_name} Servo ON - 명령 전송됨")
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} Servo ON 실패: {e}")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _servo_off(self):
        if self.controller:
            try:
                self.controller.send_servo(False)
                self.work_log_signal.emit(f"❌ {self.robot_name} Servo OFF - 명령 전송됨")
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} Servo OFF 실패: {e}")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _go_home(self):
        if self.controller:
            try:
                # Use controller to publish home angles
                self.controller.publish_angles([0.0] * 6)
                self.work_log_signal.emit(f"🏠 {self.robot_name} HOME 위치로 이동")
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} HOME 명령 실패: {e}")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _grip(self):
        if self.controller:
            try:
                self.controller.send_gripper_command(0)  # Send GRIP command
                self.work_log_signal.emit(f"✊ {self.robot_name} GRIP")
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} GRIP 실패: {e}")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _ungrip(self):
        if self.controller:
            try:
                self.controller.send_gripper_command(100)  # Send UNGRIP command
                self.work_log_signal.emit(f"🖐️ {self.robot_name} UNGRIP")
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} UNGRIP 실패: {e}")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _save_pose_memory(self, slot):
        """좌표 메모리 저장"""
        if self.controller:
            self.pose_memory[slot] = list(self.controller.current_coords)
            for i in range(6):
                # 저장된 값(self.pose_memory)는 mm 단위. UI에는 cm로 표시
                self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i] * SCALE_MM_TO_DISPLAY:.2f}")
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
                        self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i] * SCALE_MM_TO_DISPLAY:.2f}")

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
            # Publish saved pose via controller (values stored are in incoming unit)
            try:
                self.controller.publish_goal_pose(self.pose_memory[slot])
            except Exception as e:
                self.work_log_signal.emit(f"❌ {self.robot_name} P{slot} 이동 발행 실패: {e}")
                return
            # 표시용 문자열 (incoming unit)
            coords_str = f"[{', '.join([f'{(v * SCALE_MM_TO_DISPLAY):.2f}' for v in self.pose_memory[slot]])}]"
            self.work_log_signal.emit(f"🎯 {self.robot_name} P{slot} 위치로 이동: {coords_str}")
            # Target 값을 P1~N의 저장된 좌표로 변경
            for i in range(6):
                self.pose_target_inputs[i].setText(f"{self.pose_memory[slot][i] * SCALE_MM_TO_DISPLAY:.2f}")
    
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

    def _send_order_command(self):
        """Order Command 전송 (ROS2 Topic: /packing/order_command)"""
        if not self.controller:
            msg = f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!"
            print(msg)
            self.work_log_signal.emit(msg)
            return

        try:
            value = float(self.order_input.value())
            
            # RobotController에서 publisher 가져오기
            pub = self.controller.get_publisher('order_command')
            
            if pub:
                from std_msgs.msg import Float64
                msg = Float64()
                msg.data = value
                pub.publish(msg)
                
                log_msg = f"📤 Order Command 전송: {value}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
            else:
                log_msg = "⚠️ 'order_command' publisher를 찾을 수 없습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
        except Exception as e:
            log_msg = f"❌ Order Command 전송 실패: {e}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
