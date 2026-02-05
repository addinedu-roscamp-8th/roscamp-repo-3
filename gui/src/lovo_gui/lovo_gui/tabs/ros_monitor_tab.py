"""
ROS 통신 모니터링 탭
"""
import json
from pathlib import Path
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QLabel, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt, QTimer


class RosMonitorTab(QWidget):
    """ROS 통신 모니터링 탭"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.robots = []
        self.robot_controllers = {}
        self.frames_dict = {}  # {robot_id: frame}
        self.topic_values = {}  # {robot_id: {topic_name: value}}
        self._load_robot_config()
        self._setup_ui()
        self._setup_status_update_timer()
    
    def _load_robot_config(self):
        """robotname.json 파일 로드"""
        try:
            config_path = Path(__file__).parent.parent / "config" / "robotname.json"
            with open(config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.robots = data.get('robots', [])
        except Exception as e:
            print(f"⚠️ robotname.json 로드 실패: {e}")
            self.robots = []
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 그리드 레이아웃 (3열 x 2행 = 6칸)
        grid = QGridLayout()
        grid.setSpacing(10)
        
        # 로봇 분류
        robot_arms = []
        amrs = []
        
        for robot in self.robots:
            robot_name = robot.get('name', '')
            if '로봇팔' in robot_name:
                robot_arms.append(robot)
            else:
                amrs.append(robot)
        
        # 6개의 구역 생성
        # 1, 2번: 로봇팔
        self.section_1 = self._create_section(
            robot_arms[0] if len(robot_arms) > 0 else None
        )
        if len(robot_arms) > 0:
            self.frames_dict[robot_arms[0].get('id')] = self.section_1
        
        self.section_2 = self._create_section(
            robot_arms[1] if len(robot_arms) > 1 else None
        )
        if len(robot_arms) > 1:
            self.frames_dict[robot_arms[1].get('id')] = self.section_2
        
        # 3번: 빈 공간
        self.section_3 = self._create_section(None)
        
        # 4, 5, 6번: AMR
        self.section_4 = self._create_section(
            amrs[0] if len(amrs) > 0 else None
        )
        if len(amrs) > 0:
            self.frames_dict[amrs[0].get('id')] = self.section_4
        
        self.section_5 = self._create_section(
            amrs[1] if len(amrs) > 1 else None
        )
        if len(amrs) > 1:
            self.frames_dict[amrs[1].get('id')] = self.section_5
        
        self.section_6 = self._create_section(
            amrs[2] if len(amrs) > 2 else None
        )
        if len(amrs) > 2:
            self.frames_dict[amrs[2].get('id')] = self.section_6
        
        # 그리드에 배치 (row, col)
        grid.addWidget(self.section_1, 0, 0)  # 1행 1열
        grid.addWidget(self.section_2, 0, 1)  # 1행 2열
        grid.addWidget(self.section_3, 0, 2)  # 1행 3열
        grid.addWidget(self.section_4, 1, 0)  # 2행 1열
        grid.addWidget(self.section_5, 1, 1)  # 2행 2열
        grid.addWidget(self.section_6, 1, 2)  # 2행 3열
        
        layout.addLayout(grid)
    
    def _setup_status_update_timer(self):
        """상태 업데이트 타이머 설정"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_all_status)
        self.update_timer.start(500)  # 500ms마다 업데이트
    
    def _update_all_status(self):
        """모든 로봇의 상태 업데이트"""
        for robot_id, frame in self.frames_dict.items():
            if robot_id in self.robot_controllers:
                controller = self.robot_controllers[robot_id]
                is_connected = controller.robot_connected
                self._update_frame_status(frame, is_connected)
    
    def _update_frame_status(self, frame, is_connected):
        """프레임의 연결 상태 업데이트"""
        # 상태 텍스트와 색상 결정
        status_text = "연결중" if is_connected else "연결 끊김"
        status_color = "#4CAF50" if is_connected else "#f44336"
        
        # 상태 표시등 색상 업데이트
        frame.status_indicator.setStyleSheet(f"""
            font-size: 14px;
            color: {status_color};
            font-weight: bold;
        """)
        
        # 정보 라벨의 상태 텍스트 업데이트
        robot_name = frame.robot_data.get('name', 'Unknown')
        robot_ip = frame.robot_data.get('ip', 'N/A')
        robot_domain = frame.robot_data.get('domain', 'N/A')
        frame.info_label.setText(f"{robot_name} | IP: {robot_ip} | DOMAIN ID: {robot_domain} | {status_text} ")
        
        # 프레임 상태 저장
        frame.is_connected = is_connected
    
    def set_robot_controllers(self, controllers):
        """로봇 컨트롤러 설정"""
        self.robot_controllers = controllers
        
        # 컨트롤러 시그널 연결
        for robot_id, controller in controllers.items():
            if robot_id not in self.topic_values:
                self.topic_values[robot_id] = {}
            
            # 시그널 연결하여 토픽 값 업데이트
            controller.angles_updated.connect(lambda val, rid=robot_id: self._update_topic_value(rid, 'current_angles', val))
            controller.coords_updated.connect(lambda val, rid=robot_id: self._update_topic_value(rid, 'current_coords', val))
            controller.pose_updated.connect(lambda val, rid=robot_id: self._update_topic_value(rid, 'current_pose', val))
    
    def _update_topic_value(self, robot_id, topic_name, value):
        """토픽 값 업데이트"""
        if robot_id not in self.topic_values:
            self.topic_values[robot_id] = {}
        
        # 값을 문자열로 변환 (리스트는 간략하게 표시)
        if isinstance(value, list):
            if len(value) <= 6:
                value_str = ', '.join([f'{v:.2f}' if isinstance(v, float) else str(v) for v in value])
            else:
                value_str = f'[{len(value)} values]'
        else:
            value_str = str(value)
        
        self.topic_values[robot_id][topic_name] = value_str
        
        # 해당 프레임의 테이블 업데이트
        if robot_id in self.frames_dict:
            self._update_topic_table(robot_id)
    
    def _update_topic_table(self, robot_id):
        """토픽 테이블 업데이트"""
        frame = self.frames_dict.get(robot_id)
        if not frame or not hasattr(frame, 'topic_table'):
            return
        
        table = frame.topic_table
        values = self.topic_values.get(robot_id, {})
        
        # 테이블의 각 행에서 토픽 이름을 찾아 값 업데이트
        for row in range(table.rowCount()):
            topic_item = table.item(row, 0)
            if topic_item:
                topic_name = topic_item.text().split('/')[-1]  # /robot2/current_angles -> current_angles
                if topic_name in values:
                    value_item = table.item(row, 2)
                    if value_item:
                        value_item.setText(values[topic_name])
    
    def _create_section(self, robot_data):
        """구역 프레임 생성"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border: 2px solid #ccc;
                border-radius: 5px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)
        
        if robot_data is None:
            # 빈 구역
            label = QLabel("예비")
            label.setStyleSheet("font-size: 14px; font-weight: bold; color: #999;")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(label)
            layout.addStretch()
            return frame
        
        # 상태 표시 (기본값: 연결 끊김)
        is_connected = False  # 나중에 실제 연결 상태로 업데이트 가능
        status_text = "연결중" if is_connected else "연결 끊김"
        status_color = "#4CAF50" if is_connected else "#f44336"  # 초록색 또는 빨간색
        
        # 상단: 로봇 정보 한 줄 (이름, IP, DOMAIN, 상태 모두)
        info_label = QLabel(
            f"{robot_data.get('name', 'Unknown')} | IP: {robot_data.get('ip', 'N/A')} | DOMAIN ID: {robot_data.get('domain', 'N/A')} | {status_text} "
        )
        info_label.setStyleSheet("font-size: 12px; font-weight: bold; color: #333;")
        info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # 상태 표시등 (점) 추가
        status_container = QWidget()
        status_layout = QHBoxLayout(status_container)
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setSpacing(0)
        status_layout.addStretch()
        
        # 정보 라벨을 먼저 배치
        status_layout.addWidget(info_label, 1)
        
        # 상태 점 추가
        indicator = QLabel("●")
        indicator.setObjectName("status_indicator")
        indicator.setStyleSheet(f"""
            font-size: 14px;
            color: {status_color};
            font-weight: bold;
        """)
        status_layout.addWidget(indicator, 0, Qt.AlignmentFlag.AlignCenter)
        status_layout.addStretch()
        
        layout.addWidget(status_container)
        
        # 하단: 토픽 모니터링 테이블
        table = QTableWidget()
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["토픽", "타입", "값"])
        table.setStyleSheet("""
            QTableWidget {
                font-size: 12px;
                border: none;
                background-color: white;
                color: black;
            }
            QTableWidget::item {
                padding: 2px;
                color: black;
            }
            QHeaderView::section {
                background-color: #e8e8e8;
                padding: 2px;
                border: none;
                font-size: 12px;
                font-weight: bold;
                color: black;
            }
        """)
        
        # 헤더 설정
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        
        # 샘플 토픽 추가 (로봇별로 다른 토픽)
        robot_name = robot_data.get('name', '')
        robot_domain = robot_data.get('domain', 0)
        
        if '로봇팔' in robot_name:
            # domain 기반으로 robot1, robot2 결정 (61=robot1, 60=robot2)
            display_id = 'robot1' if robot_domain == 61 else 'robot2'
            prefix = f"/{display_id}"
            topics = [
                (f'{prefix}/current_angles', 'Float64MultiArray', '0'),
                (f'{prefix}/current_coords', 'Float64MultiArray', '0'),
                (f'{prefix}/current_pose', 'Pose', '0'),
                (f'{prefix}/goal_pose', 'Pose', '0'),
                (f'{prefix}/gripper_control', 'Int32', '0'),
                (f'{prefix}/servo_status', 'Bool', '0'),
                (f'{prefix}/target_angles', 'Float64MultiArray', '0'),
                (f'{prefix}/target_coords', 'Float64MultiArray', '0'),
            ]
        else:
            topics = [
                ('/odom', 'Odometry', '0'),
                ('/cmd_vel', 'Twist', '0'),
                ('/status', 'Status', '0'),
            ]
        
        table.setRowCount(len(topics))
        for row, (topic, msg_type, _) in enumerate(topics):
            table.setItem(row, 0, QTableWidgetItem(topic))
            table.setItem(row, 1, QTableWidgetItem(msg_type))
            table.setItem(row, 2, QTableWidgetItem('-'))
        
        layout.addWidget(table)
        
        # 저장: 나중에 상태 업데이트를 위한 참조
        frame.robot_data = robot_data
        frame.status_indicator = indicator
        frame.is_connected = is_connected
        frame.info_label = info_label
        frame.topic_table = table  # 테이블 참조 저장
        
        return frame
