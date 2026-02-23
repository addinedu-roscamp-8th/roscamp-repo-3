"""
Communication 탭
"""
import threading
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit,
    QTableWidget, QTableWidgetItem, QHeaderView, QLineEdit
)
from PyQt6.QtCore import Qt
from lovo_gui.constants import COMM_TABLE_WIDTH, COMM_TABLE_COL_WIDTHS, STYLE_TABLE
from lovo_gui.core import RobotArmController
from lovo_gui.tabs.tab02_manual import CameraController


class CommunicationTab(QWidget):
    """Communication 탭 - 로봇 연결 관리 및 통신 로그"""
    
    def __init__(self, robot_settings, comm_manager, parent=None):
        super().__init__(parent)
        self.robot_settings = robot_settings
        self.comm_manager = comm_manager
        self.conn_table = None
        self.comm_log_viewer = None
        self.server_status_label = None
        self.robot_row_by_id = {}       # {robot_id: row_index}
        self.robot_status_by_id = {}    # {robot_id: QLabel}
        
        # 로봇 컨트롤러들
        self.robot_controllers = {}  # {robot_id: RobotArmController}
        self.camera_controllers = {}  # {robot_id: CameraController}
        self.controller_threads = []
        self.controller_contexts = {}  # {robot_id: Context}
        self.controller_executors = {}  # {robot_id: Executor}
        
        # Domain Bridge 프로세스
        self.domain_bridge_process = None
        
        self._setup_ui()
        self._bind_connection_state_store()
        self._init_robot_controllers()
    
    def _init_robot_controllers(self):
        """로봇 컨트롤러 초기화"""
        robots = self.robot_settings.get_robots()
        
        for idx, robot in enumerate(robots):
            robot_id = robot.get("id")
            robot_name = robot.get("name")
            robot_domain = robot.get("domain")
            robot_ip = robot.get("ip", "127.0.0.1")
            
            # 로봇팔 도메인 (60, 61)만 RobotArmController 생성
            if robot_domain in [60, 61]:  # 상차(60), 하차(61) 모두 활성화
                # 모든 컨트롤러를 설정 파일의 server_domain에서 실행
                if 'main_context' not in self.controller_contexts:
                    server_domain = self.robot_settings.get_server_domain()
                    context = rclpy.Context()
                    context.init(domain_id=server_domain)
                    self.controller_contexts['main_context'] = context
                    print(f"[DEBUG Communication Tab] ROS Context 초기화: domain_id={server_domain} (from config)")
                else:
                    context = self.controller_contexts['main_context']
                
                # RobotArmController 생성 - robot_domain 전달 (토픽 이름 구분용)
                controller = RobotArmController(robot_name, robot_domain, context=context)
                self.robot_controllers[robot_id] = controller
                print(f"[DEBUG Communication Tab] RobotArmController 생성: robot_id={robot_id}, robot_name={robot_name}, robot_domain={robot_domain}")
                
                # Signal 연결
                controller.connection_changed.connect(
                    lambda connected, r_id=robot_id: self._on_robot_connection_changed(r_id, connected)
                )
                
                # 모든 노드를 하나의 Executor에 추가
                if 'main_executor' not in self.controller_executors:
                    executor = SingleThreadedExecutor(context=context)
                    self.controller_executors['main_executor'] = executor
                    
                    # ROS2 spin 스레드 시작 (한 번만)
                    thread = threading.Thread(
                        target=executor.spin,
                        daemon=True
                    )
                    thread.start()
                    self.controller_threads.append(thread)
                
                self.controller_executors['main_executor'].add_node(controller)
                self.comm_manager.log(f"{robot_name} ROS2 컨트롤러 초기화 완료 (Domain 70, Robot Domain: {robot_domain})")
            
            # 모든 로봇에 CameraController 생성 (UDP 포트: 9510, 9520, 9530, 9540, 9550)
            camera_port = 9510 + (idx * 10)
            # Pass the robot controller reference if it exists so CameraController
            # can publish to ROS via the RobotArmController
            camera_controller = CameraController(
                robot_ip, camera_port,
                robot_controller=self.robot_controllers.get(robot_id)
            )
            camera_controller.connection_changed.connect(
                lambda connected, r_id=robot_id: self._on_camera_connection_changed(r_id, connected)
            )
            self.camera_controllers[robot_id] = camera_controller
            
            self.comm_manager.log(f"{robot_name} 카메라 컨트롤러 초기화 완료 (UDP Port: {camera_port})")
        
        # Domain Bridge는 수동으로 실행하세요
        self.comm_manager.log("💡 Domain Bridge를 수동으로 실행해주세요:")
        config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        config_file = os.path.abspath(os.path.join(config_dir, 'domain_bridge_config.yaml'))
        self.comm_manager.log(f"   ros2 run domain_bridge domain_bridge {config_file}")
        # self._start_domain_bridge()  # 자동 시작 비활성화
    
    def _on_robot_connection_changed(self, robot_id, connected):
        """로봇 연결 상태 변경 시"""
        if hasattr(self.comm_manager, "state_store"):
            self.comm_manager.state_store.set_robot_ros(robot_id, connected)

        # 테이블에서 해당 로봇 찾아서 상태 업데이트
        robots = self.robot_settings.get_robots()
        for idx, robot in enumerate(robots):
            if robot.get("id") == robot_id:
                row = idx + 1  # 서버 다음
                status_widget = self.conn_table.cellWidget(row, 1)
                if status_widget:
                    if connected:
                        self._set_status_label(status_widget, "Online (ROS)", "green")
                        self.comm_manager.log(f"✅ {robot.get('name')} 연결됨")
                    else:
                        self._set_status_label(status_widget, "Offline", "red")
                        self.comm_manager.log(f"❌ {robot.get('name')} 연결 끊김")
                break

    def _on_camera_connection_changed(self, robot_id, connected):
        """카메라 연결 상태 변경 시 공통 상태 업데이트"""
        if hasattr(self.comm_manager, "state_store"):
            self.comm_manager.state_store.set_robot_camera(robot_id, connected)
    
    def get_robot_controller(self, robot_id):
        """로봇 컨트롤러 반환"""
        return self.robot_controllers.get(robot_id)
    
    def get_camera_controller(self, robot_id):
        """카메라 컨트롤러 반환"""
        return self.camera_controllers.get(robot_id)
    
    def _setup_ui(self):
        """UI 구성"""
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(10)
        
        # 왼쪽: 연결 상태 테이블
        left_widget = self._create_connection_table()
        main_layout.addWidget(left_widget)
        
        # 오른쪽: 통신 로그
        right_widget = self._create_log_viewer()
        main_layout.addWidget(right_widget)
        
        # 통신 매니저에 로그 뷰어 설정
        self.comm_manager.set_log_viewer(self.comm_log_viewer)
        
        # 샘플 로그
        self.comm_manager.log("시스템 시작")
        self.comm_manager.log("로봇 연결 대기 중...")

    def _create_section_title(self, text: str):
        """공통 섹션 타이틀 라벨"""
        title = QLabel(text)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
        title.setStyleSheet("""
            background-color: #3f3f3f;
            color: #f2f2f2;
            font-size: 13px;
            font-weight: bold;
            border: 1px solid #555;
        """)
        return title

    def _bind_connection_state_store(self):
        """공통 상태 스토어 시그널 구독"""
        if not hasattr(self.comm_manager, "state_store"):
            return
        self.comm_manager.state_store.server_state_changed.connect(self._on_server_state_changed)
        self.comm_manager.state_store.robot_state_changed.connect(self._on_robot_state_changed)

    def _on_server_state_changed(self, reachable):
        if not self.server_status_label:
            return
        if reachable is True:
            self._set_status_label(self.server_status_label, "Online", "green")
        elif reachable is False:
            self._set_status_label(self.server_status_label, "Offline", "red")
        else:
            self._set_status_label(self.server_status_label, "Unknown", "gray")

    def _on_robot_state_changed(self, robot_id, state):
        label = self.robot_status_by_id.get(robot_id)
        if not label:
            return

        ros_connected = state.get("ros_connected")
        ping_connected = state.get("ping_connected")
        camera_connected = state.get("camera_connected")

        if ros_connected is True:
            text = "🟢 Online (ROS)"
            style = "color: green; font-weight: bold;"
        elif ping_connected is True:
            text = "🟡 Reachable (Ping)"
            style = "color: #d17b00; font-weight: bold;"
        elif ros_connected is False or ping_connected is False:
            text = "🔴 Offline"
            style = "color: red; font-weight: bold;"
        else:
            text = "⚪ Unknown"
            style = "color: gray; font-weight: bold;"

        if camera_connected is True:
            text = f"{text} | CAM"
        elif camera_connected is False:
            text = f"{text} | No CAM"

        color = "gray"
        if "green" in style:
            color = "green"
        elif "#d17b00" in style:
            color = "#d17b00"
        elif "red" in style:
            color = "red"
        self._set_status_label(label, text.replace("🟢 ", "").replace("🟡 ", "").replace("🔴 ", "").replace("⚪ ", ""), color)

    def _set_status_label(self, label, text, color):
        label.setText(f"●  {text}")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet(f"color: {color}; font-weight: bold;")
    
    def _create_connection_table(self):
        """연결 상태 테이블"""
        left_widget = QWidget()
        left_widget.setFixedWidth(COMM_TABLE_WIDTH)
        layout = QVBoxLayout(left_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        layout.addWidget(self._create_section_title("통신 연결 상태"))
        
        # 테이블 생성
        self.conn_table = QTableWidget()
        self.conn_table.setColumnCount(5)
        self.conn_table.setHorizontalHeaderLabels(["이름", "상태", "도메인 ID", "IP 주소", "연결"])
        self.conn_table.setStyleSheet(STYLE_TABLE)
        
        # 셀 변경 시 저장 이벤트 연결
        self.conn_table.itemChanged.connect(self._on_table_cell_changed)
        
        # 헤더 크기 조정
        header = self.conn_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Fixed)
        self.conn_table.setColumnWidth(1, COMM_TABLE_COL_WIDTHS['status'])
        self.conn_table.setColumnWidth(2, COMM_TABLE_COL_WIDTHS['domain'])
        self.conn_table.setColumnWidth(4, COMM_TABLE_COL_WIDTHS['connect'])
        
        # 서버 + 로봇 수만큼 행 생성
        robots = self.robot_settings.get_robots()
        self.conn_table.setRowCount(len(robots) + 1)
        
        # 서버 추가
        self._add_server_row()
        
        # 로봇 추가
        for idx, robot in enumerate(robots):
            self._add_robot_row(idx + 1, idx, robot)
        
        layout.addWidget(self.conn_table)
        return left_widget
    
    def _add_server_row(self):
        """서버 행 추가"""
        server_domain = self.robot_settings.get_server_domain()
        
        self.conn_table.setItem(0, 0, QTableWidgetItem("서버"))
        
        server_status = QLabel("●  Offline")
        server_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.conn_table.setCellWidget(0, 1, server_status)
        self.server_status_label = server_status
        
        self.conn_table.setItem(0, 2, QTableWidgetItem(str(server_domain)))
        
        server_ip = QLineEdit("192.168.0.70")
        server_ip.setStyleSheet("color: black; border: none; background: transparent;")
        server_ip.editingFinished.connect(lambda: self._save_server_ip(server_ip.text()))
        self.conn_table.setCellWidget(0, 3, server_ip)
        
        server_connect = QPushButton("Connect")
        server_connect.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
        server_connect.clicked.connect(
            lambda: self.comm_manager.check_connection(server_ip.text(), server_status, "서버")
        )
        self.conn_table.setCellWidget(0, 4, server_connect)
    
    def _add_robot_row(self, row, idx, robot):
        """로봇 행 추가"""
        name = robot.get("name", f"로봇 {idx+1}")
        ip = robot.get("ip", "")
        domain = robot.get("domain", "N/A")
        robot_id = robot.get("id", f"robot{idx+1}")
        
        # 이름
        self.conn_table.setItem(row, 0, QTableWidgetItem(name))
        
        # 상태
        status_label = QLabel("●  Offline")
        status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.conn_table.setCellWidget(row, 1, status_label)
        self.robot_row_by_id[robot_id] = row
        self.robot_status_by_id[robot_id] = status_label
        
        # 도메인 ID
        self.conn_table.setItem(row, 2, QTableWidgetItem(str(domain)))
        
        # IP 주소
        ip_input = QLineEdit(ip)
        ip_input.setStyleSheet("color: black; border: none; background: transparent;")
        ip_input.editingFinished.connect(lambda r=row, inp=ip_input: self._save_robot_ip(r, inp.text()))
        self.conn_table.setCellWidget(row, 3, ip_input)
        
        # Connect 버튼
        connect_btn = QPushButton("Connect")
        connect_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
        connect_btn.clicked.connect(
            lambda: self.comm_manager.check_connection(
                ip_input.text(), status_label, name, robot_id=robot_id
            )
        )
        self.conn_table.setCellWidget(row, 4, connect_btn)
    
    def _create_log_viewer(self):
        """통신 로그 뷰어"""
        right_widget = QWidget()
        layout = QVBoxLayout(right_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)
        
        # 타이틀
        layout.addWidget(self._create_section_title("통신 로그"))
        
        # 로그 뷰어
        self.comm_log_viewer = QTextEdit()
        self.comm_log_viewer.setReadOnly(True)
        self.comm_log_viewer.setStyleSheet("""
            background-color: #1e1e1e;
            color: #00ff00;
            font-family: Consolas;
            font-size: 11px;
            border: 1px solid #555;
        """)
        layout.addWidget(self.comm_log_viewer)
        
        # 버튼
        btn_layout = QHBoxLayout()
        
        clear_btn = QPushButton("🗑️ 로그 지우기")
        clear_btn.clicked.connect(lambda: self.comm_log_viewer.clear())
        clear_btn.setStyleSheet("background-color: #dc3545; color: white; padding: 5px; font-weight: bold;")
        
        save_btn = QPushButton("💾 로그 저장")
        save_btn.setStyleSheet("background-color: #28a745; color: white; padding: 5px; font-weight: bold;")
        
        btn_layout.addWidget(clear_btn)
        btn_layout.addWidget(save_btn)
        btn_layout.addStretch()
        
        layout.addLayout(btn_layout)
        
        return right_widget
    
    def _save_server_ip(self, ip):
        """서버 IP 저장"""
        # TODO: 서버 IP를 config에 저장할 필요가 있다면 추가
        self.comm_manager.log(f"✏️ 서버 IP 변경: {ip}")
    
    def _save_robot_ip(self, row, ip):
        """로봇 IP 저장"""
        robot_idx = row - 1  # 서버 제외
        robots = self.robot_settings.get_robots()
        
        if robot_idx >= 0 and robot_idx < len(robots):
            robots[robot_idx]["ip"] = ip
            self.robot_settings.save()
            self.comm_manager.log(f"✏️ {robots[robot_idx]['name']} IP 변경: {ip}")
    
    def _on_table_cell_changed(self, item):
        """테이블 셀 변경 시 JSON 저장"""
        row = item.row()
        col = item.column()
        
        # 서버 행(0)은 제외
        if row == 0:
            return
        
        robot_idx = row - 1  # 로봇 인덱스 (서버 제외)
        robots = self.robot_settings.get_robots()
        
        if robot_idx >= len(robots):
            return
        
        # 편집 가능한 열: 0(이름), 2(도메인)
        if col == 0:  # 이름
            robots[robot_idx]["name"] = item.text()
            self.robot_settings.save()
            self.comm_manager.log(f"✏️ {item.text()} 이름 변경됨")
            
        elif col == 2:  # 도메인 ID
            try:
                domain = int(item.text())
                robots[robot_idx]["domain"] = domain
                self.robot_settings.save()
                self.comm_manager.log(f"✏️ {robots[robot_idx]['name']} 도메인 변경: {domain}")
            except ValueError:
                self.comm_manager.log(f"⚠️ 도메인 ID는 숫자여야 합니다")
                item.setText(str(robots[robot_idx]["domain"]))
    
    def _start_domain_bridge(self):
        """Domain Bridge 프로세스 시작"""
        import subprocess
        import os
        
        # 설정 파일 경로
        config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        config_file = os.path.join(config_dir, 'domain_bridge_config.yaml')
        
        if not os.path.exists(config_file):
            self.comm_manager.log(f"⚠️ Domain Bridge 설정 파일을 찾을 수 없습니다: {config_file}")
            return
        
        try:
            # domain_bridge 실행
            self.domain_bridge_process = subprocess.Popen(
                ['ros2', 'run', 'domain_bridge', 'domain_bridge', config_file],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.comm_manager.log(f"🌉 Domain Bridge 시작됨 (PID: {self.domain_bridge_process.pid})")
        except Exception as e:
            self.comm_manager.log(f"❌ Domain Bridge 시작 실패: {e}")
            self.comm_manager.log(f"💡 'ros2 run domain_bridge domain_bridge' 명령을 수동으로 실행해주세요")
    
    def __del__(self):
        """소멸자 - Domain Bridge 프로세스 종료"""
        if self.domain_bridge_process:
            self.domain_bridge_process.terminate()
            self.comm_manager.log("🌉 Domain Bridge 종료됨")
