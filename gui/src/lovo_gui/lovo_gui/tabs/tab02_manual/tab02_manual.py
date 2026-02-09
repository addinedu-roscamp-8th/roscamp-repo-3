"""
Manual 탭
"""
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QTextEdit, QGroupBox
from .robot_arm import RobotDashboardWidget
from .camera import CameraWidget
from .amr import AMRDashboardWidget


class ManualTab(QWidget):
    """Manual 탭 - 로봇별 제어"""
    
    def __init__(self, robot_settings, parent=None):
        super().__init__(parent)
        self.robot_settings = robot_settings
        self.robot_tab_widgets = {}
        self.dashboard_widgets = {}  # robot_id: RobotDashboardWidget or AMRDashboardWidget
        self.camera_widgets = {}  # robot_id: CameraWidget
        self.work_log_widgets = {}  # robot_id: QTextEdit
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.manual_tabs = QTabWidget()
        self.manual_tabs.setTabPosition(QTabWidget.TabPosition.North)
        self.manual_tabs.setStyleSheet("QTabBar::tab { min-width: 150px; min-height: 40px; font-size: 16px; }")
        
        # 로봇별 탭 생성
        robots = self.robot_settings.get_robots()
        for idx, robot in enumerate(robots):
            name = robot.get("name", f"로봇 {idx+1}")
            robot_id = robot.get("id", f"robot{idx+1}")
            
            # 탭 레이아웃: 왼쪽 카메라 + 오른쪽 대시보드
            tab_widget = QWidget()
            tab_layout = QHBoxLayout(tab_widget)
            tab_layout.setContentsMargins(0, 0, 0, 0)
            tab_layout.setSpacing(5)
            
            # 왼쪽: 카메라 + 작업 로그 영역
            left_widget = QWidget()
            left_layout = QVBoxLayout(left_widget)
            left_layout.setContentsMargins(10, 10, 10, 10)
            left_layout.setSpacing(10)
            
            # 카메라 위젯
            camera_widget = CameraWidget(name)
            self.camera_widgets[robot_id] = camera_widget
            left_layout.addWidget(camera_widget, stretch=3)
            
            # 작업 로그 위젯 (GroupBox로 감싸기)
            work_log_group = QGroupBox(f"📋 {name} 작업 로그")
            work_log_layout = QVBoxLayout()
            
            work_log = QTextEdit()
            work_log.setReadOnly(True)
            work_log.setPlaceholderText("작업 로그가 여기에 표시됩니다...")
            work_log.setMinimumHeight(120)
            work_log.setStyleSheet("background-color: black; color: white; border: 2px solid #555; border-radius: 4px;")
            self.work_log_widgets[robot_id] = work_log
            
            work_log_layout.addWidget(work_log)
            work_log_group.setLayout(work_log_layout)
            left_layout.addWidget(work_log_group, stretch=1)
            
            tab_layout.addWidget(left_widget, stretch=1)
            
            # 오른쪽: 대시보드 (로봇팔 또는 AMR)
            if idx < 2:
                # 1, 2번: 로봇팔 대시보드
                dashboard = RobotDashboardWidget(name, robot_id)
                dashboard.load_pose_memory()  # 저장된 메모리 로드
                self.dashboard_widgets[robot_id] = dashboard
                tab_layout.addWidget(dashboard, stretch=2)
            else:
                # 3, 4번: AMR 대시보드
                dashboard = AMRDashboardWidget(name, robot_id)
                self.dashboard_widgets[robot_id] = dashboard
                tab_layout.addWidget(dashboard, stretch=2)
            
            tab_index = self.manual_tabs.addTab(tab_widget, name)
            self.robot_tab_widgets[robot_id] = tab_index
        
        layout.addWidget(self.manual_tabs)
    
    def connect_controllers(self, communication_tab):
        """Communication 탭에서 컨트롤러 연결"""
        for robot_id, dashboard in self.dashboard_widgets.items():
            # 로봇 컨트롤러 연결
            controller = communication_tab.get_robot_controller(robot_id)
            if controller:
                dashboard.set_controller(controller)
                print(f"✅ {robot_id} 컨트롤러 연결됨")
            
            # 작업 로그 signal 연결 (컨트롤러 없어도 연결)
            if hasattr(dashboard, 'work_log_signal'):
                dashboard.work_log_signal.connect(
                    lambda msg, r_id=robot_id: self.add_work_log(r_id, msg)
                )
                print(f"✅ {robot_id} 작업 로그 signal 연결됨")
            
            # 카메라 컨트롤러 연결
            camera_controller = communication_tab.get_camera_controller(robot_id)
            if camera_controller and robot_id in self.camera_widgets:
                self.camera_widgets[robot_id].set_camera_controller(camera_controller)
                
                # 카메라 위젯의 작업 로그 signal 연결
                if hasattr(self.camera_widgets[robot_id], 'work_log_signal'):
                    self.camera_widgets[robot_id].work_log_signal.connect(
                        lambda msg, r_id=robot_id: self.add_work_log(r_id, msg)
                    )

                # 로봇 대시보드 참조 설정 (좌표 저장용)
                if hasattr(self.camera_widgets[robot_id], 'set_robot_dashboard'):
                    self.camera_widgets[robot_id].set_robot_dashboard(dashboard)
    
    def update_tab_name(self, robot_id, new_name):
        """탭 이름 업데이트"""
        tab_index = self.robot_tab_widgets.get(robot_id)
        if tab_index is not None:
            self.manual_tabs.setTabText(tab_index, new_name)
    
    def add_work_log(self, robot_id, message):
        """작업 로그 추가"""
        from datetime import datetime
        
        if robot_id in self.work_log_widgets:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log_message = f"[{timestamp}] {message}"
            self.work_log_widgets[robot_id].append(log_message)
            # 자동 스크롤
            self.work_log_widgets[robot_id].verticalScrollBar().setValue(
                self.work_log_widgets[robot_id].verticalScrollBar().maximum()
            )
    
    def clear_work_log(self, robot_id):
        """작업 로그 초기화"""
        if robot_id in self.work_log_widgets:
            self.work_log_widgets[robot_id].clear()
