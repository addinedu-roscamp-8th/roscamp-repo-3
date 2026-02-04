"""
로봇팔 대시보드 위젯
"""
import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QGridLayout, QScrollArea, QLineEdit, QCheckBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QUrl
from PyQt6.QtGui import QFont, QImage, QPixmap
from pathlib import Path

# --- 공통 스타일 상수 ---
LABEL_WIDTH = 70      # Axis 라벨 (J1, X 등) 너비
JOG_ZONE_WIDTH = 130  # Jog 버튼+입력창 영역 너비
VALUE_WIDTH = 70      # 값 표시창 너비
MEM_WIDTH = 65        # 메모리(Pos1~5) 표시창 너비


class RobotDashboardWidget(QWidget):
    """로봇팔 제어 대시보드 위젯"""
    
    # Signal
    work_log_signal = pyqtSignal(str)  # 작업 로그 메시지
    
    def __init__(self, robot_name, robot_key, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_key = robot_key
        self.main_font = QFont("Arial", 11, QFont.Weight.Bold)
        
        # 컨트롤러
        self.controller = None  # RobotArmController
        
        # 설정 매니저 (P1~P5 저장용)
        self.config_manager = None
        
        # MoveIt 사용 여부
        self.use_moveit = False
        
        # UI 위젯 참조
        self.pose_target_inputs = [None] * 6  # Target 입력 필드 (읽기 전용)
        self.pose_actual_labels = [None] * 6
        self.pose_error_labels = [None] * 6
        self.pose_delta_inputs = [None] * 6
        self.absolute_coord_inputs = [None] * 6  # 절대 좌표 이동 입력 필드
        
        # 메모리
        self.pose_memory = {i: [0.0]*6 for i in range(1, 6)}
        self.pose_mem_labels = {i: [None]*6 for i in range(1, 6)}
        
        self._setup_ui()
    
    def set_controller(self, controller):
        """컨트롤러 설정 및 Signal 연결"""
        self.controller = controller
        self.work_log_signal.emit(f"🔗 {self.robot_name} 컸트롤러 연결됨")
        
        # RobotArmController Signal 연결
        controller.coords_updated.connect(self.update_coords_display)
        controller.pose_updated.connect(self.update_pose_display)
    
    def _setup_ui(self):
        """UI 구성"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 제어 영역 (스크롤)
        control_widget = self._create_control_section()
        main_layout.addWidget(control_widget)
    
    def _create_control_section(self):
        """제어 섹션"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        robot_layout = QVBoxLayout(scroll_content)
        
        # 1. System Control
        sys_group = self._create_system_control()
        robot_layout.addWidget(sys_group)
        
        # 2. 좌표 컨트롤러
        cart_group = self._create_cartesian_controller()
        robot_layout.addWidget(cart_group)
        
        robot_layout.addStretch()
        
        scroll.setWidget(scroll_content)
        return scroll
    
    def _setup_grid_alignment(self, layout):
        """그리드 정렬 설정"""
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setColumnMinimumWidth(0, LABEL_WIDTH)
        layout.setColumnMinimumWidth(1, JOG_ZONE_WIDTH)
        layout.setColumnMinimumWidth(2, VALUE_WIDTH)
        layout.setColumnMinimumWidth(3, VALUE_WIDTH)
        layout.setColumnMinimumWidth(4, 90)
        for i in range(5, 10):
            layout.setColumnMinimumWidth(i, MEM_WIDTH)
    
    def _create_label(self, text, color, width):
        """라벨 생성"""
        lbl = QLabel(text)
        lbl.setFixedSize(width, 32)
        lbl.setFont(self.main_font)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(
            f"border: 2px solid {color}; background-color: white; color: black; border-radius: 3px;"
        )
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
        self._setup_grid_alignment(c_grid)
        
        # 헤더
        c_headers = ["Axis", "Jog (+/-)", "Target", "Actual", "Error", "P1", "P2", "P3", "P4", "P5"]
        for col, text in enumerate(c_headers):
            c_grid.addWidget(QLabel(text), 0, col, Qt.AlignmentFlag.AlignCenter)
        
        axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]
        for i in range(6):
            row = i + 1
            
            # 축 라벨
            axis_lbl = QLabel(axes[i])
            axis_lbl.setFixedWidth(LABEL_WIDTH)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_lbl.setStyleSheet(
                "border: 2px solid #666; background-color: #E0E0E0; color: black; border-radius: 3px; padding: 4px;"
            )
            c_grid.addWidget(axis_lbl, row, 0)
            
            # Jog 컨트롤
            jog_layout = QHBoxLayout()
            jog_layout.setContentsMargins(0, 0, 0, 0)
            jog_layout.setSpacing(4)
            
            jbtn_m = QPushButton("-")
            jbtn_m.setFixedSize(32, 32)
            jbtn_m.clicked.connect(lambda ch, idx=i: self._cart_jog(idx, -1))
            
            jbtn_p = QPushButton("+")
            jbtn_p.setFixedSize(32, 32)
            jbtn_p.clicked.connect(lambda ch, idx=i: self._cart_jog(idx, 1))
            
            delta_input = QLineEdit("5.0")
            delta_input.setFixedSize(40, 32)
            delta_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.pose_delta_inputs[i] = delta_input
            self.pose_target_inputs[i] = None  # 아래에서 설정
            self.pose_actual_labels[i] = None
            self.pose_error_labels[i] = None
            
            jog_layout.addWidget(jbtn_m)
            jog_layout.addWidget(delta_input)
            jog_layout.addWidget(jbtn_p)
            c_grid.addLayout(jog_layout, row, 1)
            
            # Target 입력 필드 (읽기 전용으로 변경)
            target_input = QLineEdit("0.0")
            target_input.setFixedWidth(VALUE_WIDTH)
            target_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            target_input.setReadOnly(True)  # 읽기 전용
            target_input.setStyleSheet(
                "background-color: #F5F5F5; color: #555; "
                "border: 2px solid #BDBDBD; border-radius: 3px; padding: 2px;"
            )
            self.pose_target_inputs[i] = target_input
            c_grid.addWidget(target_input, row, 2)
            
            actual_lbl = self._create_label("0.0", "#757575", VALUE_WIDTH)
            self.pose_actual_labels[i] = actual_lbl
            c_grid.addWidget(actual_lbl, row, 3)
            
            # Error
            err_lbl = self._create_label("0.0", "#f44336", 90)
            self.pose_error_labels[i] = err_lbl
            c_grid.addWidget(err_lbl, row, 4)
            
            # 메모리 Pos1~5
            for m_idx in range(1, 6):
                mem_lbl = self._create_label("---", "#555", MEM_WIDTH)
                self.pose_mem_labels[m_idx][i] = mem_lbl
                c_grid.addWidget(mem_lbl, row, m_idx + 4)
        
        # 좌표 메모리 저장/이동 버튼 라인
        for m_idx in range(1, 6):
            btn_vbox = QVBoxLayout()
            btn_vbox.setSpacing(4)
            btn_vbox.setContentsMargins(0, 0, 0, 0)
            
            ps_btn = QPushButton("저장")
            ps_btn.setFixedSize(MEM_WIDTH - 5, 28)
            ps_btn.setStyleSheet("font-size: 10px; background-color: #1976D2;")
            ps_btn.clicked.connect(lambda ch, m=m_idx: self._save_pose_memory(m))
            
            pm_btn = QPushButton("이동")
            pm_btn.setFixedSize(MEM_WIDTH - 5, 28)
            pm_btn.setStyleSheet("font-size: 10px; background-color: #2E7D32;")
            pm_btn.clicked.connect(lambda ch, m=m_idx: self._move_pose_memory(m))
            
            btn_vbox.addWidget(ps_btn)
            btn_vbox.addWidget(pm_btn)
            c_grid.addLayout(btn_vbox, 7, m_idx + 4)
        
        main_layout.addLayout(c_grid)
        
        # ==================== 절대 좌표 이동 섹션 ====================
        abs_coord_group = QGroupBox("📍 절대 좌표 이동 (Absolute Coordinate Movement)")
        abs_coord_layout = QVBoxLayout()
        abs_coord_layout.setSpacing(10)
        abs_coord_layout.setContentsMargins(10, 10, 10, 10)
        
        # 라벨과 입력칸을 가로로 정렬
        coord_axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]
        
        for i in range(6):
            # 각 좌표별 수평 레이아웃
            coord_h_layout = QHBoxLayout()
            coord_h_layout.setSpacing(10)
            coord_h_layout.setContentsMargins(5, 0, 5, 0)
            
            # 라벨 (좌표컨트롤러와 동일한 스타일)
            axis_lbl = QLabel(coord_axes[i])
            axis_lbl.setFixedWidth(LABEL_WIDTH)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            axis_lbl.setStyleSheet(
                "border: 2px solid #666; background-color: #E0E0E0; color: black; border-radius: 3px; padding: 4px;"
            )
            coord_h_layout.addWidget(axis_lbl)
            
            # 입력 필드 (좌표컨트롤러의 Target과 동일한 스타일)
            input_field = QLineEdit("0.0")
            input_field.setFixedWidth(VALUE_WIDTH)
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
        main_layout.addWidget(abs_coord_group)
        
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
            
            if self.use_moveit:
                # MoveIt 사용
                print(f"🤖 MoveIt으로 이동: {current}")
                # TODO: MoveIt 제어 로직 구현
                # self.controller.move_with_moveit(current)
            else:
                # 직접 제어
                self.controller.publish_coords(current)
            
            # Target 필드 업데이트
            self.pose_target_inputs[axis].setText(f"{current[axis]:.1f}")
        except:
            pass
    
    def _servo_on(self):
        if self.controller:
            self.controller.send_servo(True)
            self.work_log_signal.emit(f"✅ {self.robot_name} Servo ON - 명령 전송됨")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _servo_off(self):
        if self.controller:
            self.controller.send_servo(False)
            self.work_log_signal.emit(f"❌ {self.robot_name} Servo OFF - 명령 전송됨")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _go_home(self):
        if self.controller:
            self.controller.go_home()
            self.work_log_signal.emit(f"🏠 {self.robot_name} HOME 위치로 이동")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _grip(self):
        if self.controller:
            self.controller.send_gripper(1)
            self.work_log_signal.emit(f"✊ {self.robot_name} GRIP")
        else:
            self.work_log_signal.emit(f"⚠️ {self.robot_name} 컨트롤러가 연결되지 않았습니다!")
    
    def _ungrip(self):
        if self.controller:
            self.controller.send_gripper(0)
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
            
            # 로봇에 명령 전송
            self.controller.publish_coords(target_coords)
            
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
            if self.config_manager:
                self.config_manager.save_pose_memory(
                    self.robot_name, 
                    slot, 
                    self.pose_memory[slot]
                )
                self.work_log_signal.emit(f"💾 {self.robot_name} P{slot} 저장 완료")
            else:
                self.work_log_signal.emit(f"⚠️ {self.robot_name} P{slot} 메모리에만 저장됨 (파일 저장 불가)")
    
    def load_pose_memory(self):
        """저장된 좌표 메모리 로드"""
        if self.config_manager:
            saved_memory = self.config_manager.load_all_pose_memory(self.robot_name)
            for slot in range(1, 6):
                slot_str = str(slot)
                if slot_str in saved_memory:
                    self.pose_memory[slot] = saved_memory[slot_str]
                    # UI 업데이트
                    if all(self.pose_mem_labels[slot]):
                        for i in range(6):
                            self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i]:.1f}")
            
            self.work_log_signal.emit(f"📂 {self.robot_name} 좌표 메모리 로드 완료")
    
    def _move_pose_memory(self, slot):
        """좌표 메모리 이동"""
        if self.controller:
            if self.use_moveit:
                # MoveIt 사용
                print(f"🤖 MoveIt으로 이동: Pos{slot}")
                # TODO: MoveIt 제어 로직 구현
                # self.controller.move_with_moveit(self.pose_memory[slot])
            else:
                # 직접 제어
                self.controller.publish_coords(self.pose_memory[slot])
                coords_str = f"[{', '.join([f'{v:.1f}' for v in self.pose_memory[slot]])}]"
                self.work_log_signal.emit(f"🎯 {self.robot_name} P{slot} 위치로 이동: {coords_str}")
            
            # Target 값을 P1~P5의 저장된 좌표로 변경
            for i in range(6):
                self.pose_target_inputs[i].setText(f"{self.pose_memory[slot][i]:.1f}")
    
    def _create_pose_memory_buttons(self):
        """좌표 메모리 저장/이동 버튼"""
        pose_mem_btn_layout = QHBoxLayout()
        for m in range(1, 6):
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

