import sys
import json
import os
import csv
import subprocess
from datetime import datetime
from PyQt6.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QTextEdit, QTabWidget, QLabel, QFrame, QLineEdit,
    QCalendarWidget, QSizePolicy, QGridLayout, QComboBox, QGroupBox, QScrollArea, QDialog,
    QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont

class CameraDialog(QDialog):
    """카메라 뷰 다이얼로그"""
    def __init__(self, robot_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"{robot_name} - 카메라 뷰")
        self.setFixedSize(680, 580)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 카메라 뷰
        cam_view = QLabel("카메라 대기 중...")
        cam_view.setFixedSize(640, 480)
        cam_view.setStyleSheet("background-color: black; border: 2px solid #555; border-radius: 4px;")
        cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(cam_view)
        
        # 컨트롤 버튼
        btn_layout = QHBoxLayout()
        for text in ["🔌 CONNECT", "❌ DISCONNECT", "📸 CAPTURE", "✖ 닫기"]:
            btn = QPushButton(text)
            btn.setFixedHeight(40)
            if text == "✖ 닫기":
                btn.clicked.connect(self.close)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #dc3545;
                        color: white;
                        font-weight: bold;
                    }
                    QPushButton:hover {
                        background-color: #c82333;
                    }
                """)
            btn_layout.addWidget(btn)
        
        layout.addLayout(btn_layout)

class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # 1. 설정 파일 경로 설정
        self.config_path = "robotname.json"
        self.load_config()

        self.setWindowTitle("Lovo 제어 시스템")
        self.resize(1920, 1080)
        
        # 타이틀바 제거 (프레임리스 윈도우)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        
        # 메뉴바 제거
        self.menuBar().hide()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 수평 레이아웃 (왼쪽: 탭, 오른쪽: 사이드바)
        horizontal_layout = QHBoxLayout(central_widget)
        horizontal_layout.setContentsMargins(0, 0, 0, 0)
        horizontal_layout.setSpacing(0)
        
        # 왼쪽 영역 (탭 위젯)
        left_widget = QWidget()
        main_layout = QVBoxLayout(left_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        horizontal_layout.addWidget(left_widget, 1)
        
        # 우측 사이드바
        sidebar = QWidget()
        sidebar.setFixedWidth(150)
        sidebar.setStyleSheet("background-color: #2d2d2d;")
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(10, 00, 10, 20)
        sidebar_layout.setSpacing(15)
        
        # 사이드바 버튼들
        Sidebar_Height = 80
        btn_run = QPushButton("운전")
        btn_run.setFixedHeight(Sidebar_Height)
        btn_run.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
            QPushButton:pressed {
                background-color: #1e7e34;
            }
        """)
        
        btn_stop = QPushButton("정지")
        btn_stop.setFixedHeight(Sidebar_Height)
        btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
            QPushButton:pressed {
                background-color: #bd2130;
            }
        """)
        
        btn_reset = QPushButton("초기화")
        btn_reset.setFixedHeight(Sidebar_Height)
        btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #ffc107;
                color: #333;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #e0a800;
            }
            QPushButton:pressed {
                background-color: #d39e00;
            }
        """)
        
        btn_exit = QPushButton("종료")
        btn_exit.setFixedHeight(Sidebar_Height)
        btn_exit.setStyleSheet("""
            QPushButton {
                background-color: #6c757d;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #5a6268;
            }
            QPushButton:pressed {
                background-color: #545b62;
            }
        """)
        btn_exit.clicked.connect(QApplication.quit)
        
        sidebar_layout.addWidget(btn_run)
        sidebar_layout.addWidget(btn_stop)
        sidebar_layout.addWidget(btn_reset)
        sidebar_layout.addStretch()
        sidebar_layout.addWidget(btn_exit)
        
        horizontal_layout.addWidget(sidebar)

        # 하단 메인 탭 위젯 설정
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.South)
        self.tabs.setStyleSheet("QTabBar::tab { min-height: 80px; min-width: 200px; font-size: 16px; }")

        # 1. Main 탭 (절대 좌표 방식)
        self.tab_main = QWidget()
        
        # 좌상단: 시스템 맵 (0, 0, 1000, 500)
        system_map = QFrame(self.tab_main)
        system_map.setGeometry(0, 0, 1300, 650)
        system_map.setStyleSheet("""
            QFrame {
                background-color: #e8e8e8;
                border: none;
            }
        """)
        system_map_layout = QVBoxLayout(system_map)
        system_map_layout.addWidget(QLabel("시스템 맵", alignment=Qt.AlignmentFlag.AlignCenter))
        
        # 우상단: 주문 로그 (1000, 0, 620, 500)
        order_log_frame = QFrame(self.tab_main)
        order_log_frame.setGeometry(1300, 0, 460, 650)
        order_log_frame.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border: none;
            }
        """)
        order_log_layout = QVBoxLayout(order_log_frame)
        order_log_layout.setContentsMargins(5, 5, 5, 5)
        
        order_log_title = QLabel("주문 로그")
        order_log_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
        order_log_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        order_log_layout.addWidget(order_log_title)
        
        self.order_log_viewer = QTextEdit()
        self.order_log_viewer.setReadOnly(True)
        self.order_log_viewer.setStyleSheet("""
            background-color: white;
            color: #333;
            font-family: Consolas;
            font-size: 11px;
            border: 1px solid #ccc;
        """)
        order_log_layout.addWidget(self.order_log_viewer)
        
        # 좌하단: 로봇 상태 그리드 (0, 500, 1000, 280)
        grid_container = QWidget(self.tab_main)
        grid_container.setGeometry(0, 650, 1300, 300)
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
        robots = self.config.get("robots", [])
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
            
            # 현재 상태 (운송 로봇은 좌표 표시)
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
        
        # 우하단: 카메라 뷰 (1000, 500, 620, 280)
        self.camera_view_frame = QFrame(self.tab_main)
        self.camera_view_frame.setGeometry(1300, 650, 460, 320)
        self.camera_view_frame.setStyleSheet("""
            QFrame {
                background-color: #2a2a2a;
                border: 2px solid #555;
                border-radius: 4px;
            }
        """)
        camera_view_layout = QVBoxLayout(self.camera_view_frame)
        camera_view_layout.setContentsMargins(0, 5, 0, 5)
        camera_view_layout.setSpacing(5)
        
        self.camera_title = QLabel("카메라 선택 대기 중...")
        self.camera_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #999;")
        self.camera_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_title.setFixedHeight(20)
        camera_view_layout.addWidget(self.camera_title)
        
        self.camera_view_label = QLabel("캠 버튼을 눌러 카메라를 선택하세요")
        self.camera_view_label.setFixedSize(420, 270)
        self.camera_view_label.setStyleSheet("background-color: black; border: 1px solid #444; border-radius: 4px; color: #666;")
        self.camera_view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        camera_view_layout.addWidget(self.camera_view_label)
        
        self.tabs.addTab(self.tab_main, "Main")

        # 2. Manual 탭 (상단에 설정된 로봇 이름들 표시)
        self.tab_manual = QWidget()
        manual_layout = QVBoxLayout(self.tab_manual)
        self.manual_tabs = QTabWidget()
        self.manual_tabs.setTabPosition(QTabWidget.TabPosition.North)
        self.manual_tabs.setStyleSheet("QTabBar::tab { min-width: 150px; min-height: 40px; font-size: 16px; }")

        # 설정 파일에서 로봇 이름들을 가져와서 탭 생성
        self.robot_tab_widgets = {} # 실시간 수정을 위해 딕셔너리에 저장
        robots = self.config.get("robots", [])
        
        for idx, robot in enumerate(robots):
            name = robot.get("name", f"로봇 {idx+1}")
            robot_id = robot.get("id", f"robot{idx+1}")
            
            # 로봇팔(처음 2개)만 전체 대시보드, 나머지는 비전 전용
            if idx < 2:
                tab = self.create_robot_dashboard_tab(name, robot_id)
            else:
                tab = self.create_vision_only_tab(name, robot_id)
            
            index = self.manual_tabs.addTab(tab, name)
            self.robot_tab_widgets[robot_id] = index # 로봇 ID별 탭 인덱스 저장

        manual_layout.addWidget(self.manual_tabs)
        self.tabs.addTab(self.tab_manual, "Manual")

        # 3. Monitoring 탭
        self.tab_monitoring = QWidget()
        monitoring_layout = QVBoxLayout(self.tab_monitoring)
        monitoring_layout.setContentsMargins(20, 20, 20, 20)
        monitoring_layout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        
        # 모니터링 맵 (1600x800)
        monitoring_map = QFrame()
        monitoring_map.setFixedSize(1600, 800)
        monitoring_map.setStyleSheet("""
            QFrame {
                background-color: #e8e8e8;
                border: 2px solid #999;
                border-radius: 4px;
            }
        """)
        map_layout = QVBoxLayout(monitoring_map)
        map_label = QLabel("모니터링 맵", alignment=Qt.AlignmentFlag.AlignCenter)
        map_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #555;")
        map_layout.addWidget(map_label)
        
        monitoring_layout.addWidget(monitoring_map)
        monitoring_layout.addStretch()
        
        self.tabs.addTab(self.tab_monitoring, "Monitoring")

        # 4. Communication (통신) 탭
        self.tab_comm = QWidget()
        self.setup_comm_tab()
        self.tabs.addTab(self.tab_comm, "Communication")

        # 5. Log 탭
        self.setup_log_tab()
        self.tabs.addTab(self.tab_log, "Log")

        main_layout.addWidget(self.tabs)

    def load_config(self):
        """robotname.json 파일을 읽어옵니다. 없으면 기본값을 생성합니다."""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    self.config = json.load(f)
            except:
                self.set_default_config()
        else:
            self.set_default_config()

    def set_default_config(self):
        self.config = {
            "server_domain": 70,
            "robots": [
                {"name": "상차 로봇팔", "domain": 61, "id": "jecobot_126b", "ip": "192.168.0.61"},
                {"name": "하차 로봇팔", "domain": 60, "id": "jecobot_aab4", "ip": "192.168.0.60"},
                {"name": "운송 로봇 1", "domain": 52, "id": "d9ec", "ip": "192.168.0.10"},
                {"name": "운송 로봇 2", "domain": 51, "id": "20f0", "ip": "192.168.0.48"},
                {"name": "청소 로봇", "domain": 50, "id": "dfc6", "ip": "192.168.0.44"}
            ]
        }
        self.save_config()

    def save_config(self):
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(self.config, f, indent=4, ensure_ascii=False)

    def setup_comm_tab(self):
        """통신 탭 UI 구성 - 표 형식"""
        main_h_layout = QHBoxLayout(self.tab_comm)
        main_h_layout.setContentsMargins(0, 0, 0, 0)
        main_h_layout.setSpacing(10)
        
        # 왼쪽: 연결 상태 테이블
        left_widget = QWidget()
        left_widget.setFixedWidth(700)
        layout = QVBoxLayout(left_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # 테이블 생성
        self.conn_table = QTableWidget()
        self.conn_table.setColumnCount(5)
        self.conn_table.setHorizontalHeaderLabels(["이름", "상태", "도메인 ID", "IP 주소", "연결"])
        
        # 테이블 스타일 설정
        self.conn_table.setStyleSheet("""
            QTableWidget {
                background-color: white;
                gridline-color: #d0d0d0;
                color: black;
            }
            QHeaderView::section {
                background-color: #4a90e2;
                color: white;
                padding: 5px;
                border: 1px solid #357abd;
                font-weight: bold;
            }
            QTableWidget::item {
                padding: 5px;
            }
        """)
        
        # 헤더 크기 조정
        header = self.conn_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)  # 이름
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)     # 상태
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)     # 도메인 ID
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)   # IP
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Fixed)     # 연결
        self.conn_table.setColumnWidth(1, 100)
        self.conn_table.setColumnWidth(2, 100)
        self.conn_table.setColumnWidth(4, 100)
        
        # 서버 + 로봇 수만큼 행 생성
        robots = self.config.get("robots", [])
        self.conn_table.setRowCount(len(robots) + 1)  # +1 for server
        
        # 서버 추가 (첫 번째 행)
        server_domain = self.config.get("server_domain", 70)
        self.conn_table.setItem(0, 0, QTableWidgetItem("서버"))
        
        server_status = QLabel("🔴 Offline")
        server_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.conn_table.setCellWidget(0, 1, server_status)
        
        self.conn_table.setItem(0, 2, QTableWidgetItem(str(server_domain)))
        
        server_ip = QLineEdit("192.168.0.70")
        server_ip.setStyleSheet("color: black; border: none; background: transparent;")
        self.conn_table.setCellWidget(0, 3, server_ip)
        
        server_connect = QPushButton("Connect")
        server_connect.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
        server_connect.clicked.connect(lambda: self.check_robot_connection(-1, server_ip.text(), server_status, "서버"))
        self.conn_table.setCellWidget(0, 4, server_connect)
        
        # 로봇 추가
        for idx, robot in enumerate(robots):
            row = idx + 1  # 서버 다음부터
            name = robot.get("name", f"로봇 {idx+1}")
            ip = robot.get("ip", "")
            domain = robot.get("domain", "N/A")
            
            # 이름
            self.conn_table.setItem(row, 0, QTableWidgetItem(name))
            
            # 상태
            status_label = QLabel("🔴 Offline")
            status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.conn_table.setCellWidget(row, 1, status_label)
            
            # 도메인 ID
            self.conn_table.setItem(row, 2, QTableWidgetItem(str(domain)))
            
            # IP 주소
            ip_input = QLineEdit(ip)
            ip_input.setStyleSheet("color: black; border: none; background: transparent;")
            self.conn_table.setCellWidget(row, 3, ip_input)
            
            # Connect 버튼
            connect_btn = QPushButton("Connect")
            connect_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
            connect_btn.clicked.connect(lambda ch, i=idx, ip_w=ip_input, st_lbl=status_label, nm=name: self.check_robot_connection(i, ip_w.text(), st_lbl, nm))
            self.conn_table.setCellWidget(row, 4, connect_btn)
        
        layout.addWidget(self.conn_table)
        main_h_layout.addWidget(left_widget)
        
        # 오른쪽: 통신 로그
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        log_title = QLabel("📡 통신 로그")
        log_title.setStyleSheet("font-size: 16px; font-weight: bold; color: #333;")
        right_layout.addWidget(log_title)
        
        self.comm_log_viewer = QTextEdit()
        self.comm_log_viewer.setReadOnly(True)
        self.comm_log_viewer.setStyleSheet("""
            background-color: #1e1e1e;
            color: #00ff00;
            font-family: Consolas;
            font-size: 11px;
            border: 1px solid #555;
        """)
        right_layout.addWidget(self.comm_log_viewer)
        
        # 로그 제어 버튼
        log_btn_layout = QHBoxLayout()
        clear_btn = QPushButton("🗑️ 로그 지우기")
        clear_btn.clicked.connect(lambda: self.comm_log_viewer.clear())
        clear_btn.setStyleSheet("background-color: #dc3545; color: white; padding: 5px; font-weight: bold;")
        
        save_btn = QPushButton("💾 로그 저장")
        save_btn.setStyleSheet("background-color: #28a745; color: white; padding: 5px; font-weight: bold;")
        
        log_btn_layout.addWidget(clear_btn)
        log_btn_layout.addWidget(save_btn)
        log_btn_layout.addStretch()
        right_layout.addLayout(log_btn_layout)
        
        main_h_layout.addWidget(right_widget)
        
        # 샘플 로그 추가
        self.comm_log_viewer.append("[2026-01-23 17:32:15] 시스템 시작")
        self.comm_log_viewer.append("[2026-01-23 17:32:16] 로봇 연결 대기 중...")

    def check_robot_connection(self, robot_idx, ip_address, status_label, robot_name):
        """Ping으로 로봇 연결 상태 확인"""
        if not ip_address:
            self.log_communication(f"⚠️ {robot_name}: IP 주소가 입력되지 않았습니다")
            return
        
        self.log_communication(f"🔍 {robot_name} ({ip_address}) 연결 확인 중...")
        
        # Linux ping 명령어: -c 1 (1번 ping), -W 1 (1초 타임아웃)
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )
            
            if result.returncode == 0:
                status_label.setText("🟢 Online")
                status_label.setStyleSheet("color: green; font-weight: bold;")
                self.log_communication(f"✅ {robot_name} ({ip_address}) 연결 성공")
            else:
                status_label.setText("🔴 Offline")
                status_label.setStyleSheet("color: red; font-weight: bold;")
                self.log_communication(f"❌ {robot_name} ({ip_address}) 연결 실패")
        except subprocess.TimeoutExpired:
            status_label.setText("🔴 Timeout")
            status_label.setStyleSheet("color: orange; font-weight: bold;")
            self.log_communication(f"⏱️ {robot_name} ({ip_address}) 연결 시간 초과")
        except Exception as e:
            status_label.setText("🔴 Error")
            status_label.setStyleSheet("color: red; font-weight: bold;")
            self.log_communication(f"⚠️ {robot_name} ({ip_address}) 오류: {str(e)}")
    
    def log_communication(self, message):
        """통신 로그에 메시지 추가"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.comm_log_viewer.append(log_entry)

    def update_robot_name(self, robot_index, new_name):
        """이름을 변경하고 파일 저장 및 Manual 탭 이름을 즉시 갱신합니다."""
        # 1. 데이터 업데이트 및 저장
        if robot_index < len(self.config.get("robots", [])):
            robot_id = self.config["robots"][robot_index].get("id", f"robot{robot_index+1}")
            self.config["robots"][robot_index]["name"] = new_name
            self.save_config()
            
            # 2. Manual 탭의 텍스트 즉시 변경
            tab_index = self.robot_tab_widgets.get(robot_id)
            if tab_index is not None:
                self.manual_tabs.setTabText(tab_index, new_name)
                self.log_viewer.append(f"✅ [{robot_id}]의 이름이 '{new_name}'으로 변경되었습니다.")

    def setup_log_tab(self):
        self.tab_log = QWidget()
        log_tab_layout = QHBoxLayout(self.tab_log)
        left_panel = QVBoxLayout()
        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(350)

        self.calendar = QCalendarWidget()
        self.calendar.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.calendar.setFixedWidth(330)
        self.calendar.setGridVisible(True)
        left_panel.addWidget(self.calendar)

        for text in ["자재", "모니터링", "알람"]:
            btn = QPushButton(text)
            btn.setFixedWidth(260)
            btn.setFixedHeight(40)
            left_panel.addWidget(btn)
        left_panel.addStretch()

        self.log_viewer = QTextEdit()
        self.log_viewer.setReadOnly(True)
        self.log_viewer.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas;")

        log_tab_layout.addWidget(left_widget)
        log_tab_layout.addWidget(self.log_viewer)
        self.calendar.selectionChanged.connect(self.filter_log_by_date)

    def filter_log_by_date(self):
        selected_date = self.calendar.selectedDate().toString("yyyy-MM-dd")
        self.log_viewer.append(f"📅 [{selected_date}] 날짜 선택됨")
    
    def show_camera_view(self, robot):
        """메인 화면의 카메라 뷰 영역에 선택한 로봇의 카메라 표시"""
        robot_name = robot.get("name", "로봇")
        self.camera_title.setText(f"{robot_name} - 카메라 뷰")
        self.camera_view_label.setText("카메라 스트리밍 대기 중...")

    # --- 비전 전용 탭 생성 메서드 (운송/청소 로봇용) ---
    def create_vision_only_tab(self, robot_name, robot_key):
        """비전 시스템만 있는 탭 (바퀴 제어 로봇용)"""
        tab = QWidget()
        main_layout = QVBoxLayout(tab)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(10)
        
        # 카메라 비전 그룹
        vision_group = QGroupBox("📷 Camera Vision")
        vision_layout = QVBoxLayout()
        
        cam_view = QLabel("카메라 대기 중...")
        cam_view.setFixedSize(640, 480)
        cam_view.setStyleSheet("background-color: black; border: 2px solid #555; border-radius: 4px;")
        cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        cam_ctrl_layout = QHBoxLayout()
        for text, func in [("🔌 CONNECT", lambda: None), ("❌ DISCONNECT", lambda: None), ("📸 CAPTURE", lambda: None)]:
            btn = QPushButton(text)
            btn.setFixedSize(120, 40)
            btn.clicked.connect(func)
            cam_ctrl_layout.addWidget(btn)
        cam_ctrl_layout.addStretch()
        
        vision_layout.addWidget(cam_view)
        vision_layout.addLayout(cam_ctrl_layout)
        vision_group.setLayout(vision_layout)
        
        main_layout.addWidget(vision_group, alignment=Qt.AlignmentFlag.AlignTop)
        main_layout.addStretch()
        
        # 안내 메시지
        info_label = QLabel("💡 바퀴 제어 시스템은 추후 추가 예정입니다.")
        info_label.setStyleSheet("color: #888; font-size: 12px; font-style: italic;")
        main_layout.addWidget(info_label, alignment=Qt.AlignmentFlag.AlignCenter)
        
        return tab
    
    # --- 로봇 대시보드 탭 생성 메서드 ---
    def create_robot_dashboard_tab(self, robot_name, robot_key):
        """로봇팔 제어 대시보드 탭 생성"""
        tab = QWidget()
        main_h_layout = QHBoxLayout(tab)
        main_h_layout.setContentsMargins(0, 0, 0, 0)
        main_h_layout.setSpacing(10)
        
        # --- 왼쪽: 카메라 비전 ---
        left_layout = QVBoxLayout()
        
        # 카메라 비전 그룹
        vision_group = QGroupBox("📷 Camera Vision")
        vision_group.setFixedWidth(700)
        vision_layout = QVBoxLayout()
        
        cam_view = QLabel("카메라 대기 중...")
        cam_view.setFixedSize(640, 480)
        cam_view.setStyleSheet("background-color: black; border: 2px solid #555; border-radius: 4px;")
        cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        cam_ctrl_layout = QHBoxLayout()
        for text, func in [("🔌 CONNECT", lambda: None), ("❌ DISCONNECT", lambda: None), ("📸 CAPTURE", lambda: None)]:
            btn = QPushButton(text)
            btn.clicked.connect(func)
            cam_ctrl_layout.addWidget(btn)
        
        vision_layout.addWidget(cam_view)
        vision_layout.addLayout(cam_ctrl_layout)
        vision_group.setLayout(vision_layout)
        
        left_layout.addWidget(vision_group)
        left_layout.addStretch()
        
        main_h_layout.addLayout(left_layout)
        
        # --- 오른쪽: 제어 영역 (스크롤) ---
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        right_layout = QVBoxLayout(scroll_content)
        
        # 시스템 컨트롤
        sys_group = QGroupBox("⚙️ System Control")
        sys_group.setFixedHeight(100)
        sys_h_layout = QHBoxLayout()
        
        for text, func in [("✓ Servo ON", self.send_servo), ("✗ Servo OFF", self.send_servo), 
                           ("🏠 HOME", self.go_home), ("✊ GRIP", self.control_gripper), ("🖐️ UNGRIP", self.control_gripper)]:
            btn = QPushButton(text)
            btn.setFixedSize(100, 40)
            btn.setFont(QFont("Arial", 9, QFont.Weight.Bold))
            btn.clicked.connect(func)
            sys_h_layout.addWidget(btn)
        sys_h_layout.addStretch()
        sys_group.setLayout(sys_h_layout)
        right_layout.addWidget(sys_group)
        
        # 각도 컨트롤러
        jog_group = QGroupBox("🔧 각도 컨트롤러")
        grid = QGridLayout()
        grid.setSpacing(6)
        
        # 헤더
        headers = ["축", "Jog", "목표", "현재", "오차", "Pos1", "Pos2", "Pos3", "Pos4", "Pos5"]
        for col, text in enumerate(headers):
            grid.addWidget(QLabel(text), 0, col, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # 관절별 컨트롤
        for i in range(6):
            row = i + 1
            
            # 축 라벨
            axis_lbl = QLabel(f"J{i+1}")
            axis_lbl.setFixedWidth(40)
            axis_lbl.setStyleSheet("background-color: #E0E0E0; color: black; border-radius: 3px;")
            grid.addWidget(axis_lbl, row, 0)
            
            # Jog 컨트롤
            jog_h = QHBoxLayout()
            jog_h.setSpacing(2)
            btn_m = QPushButton("-")
            btn_m.setFixedSize(30, 28)
            btn_p = QPushButton("+")
            btn_p.setFixedSize(30, 28)
            jog_h.addWidget(btn_m)
            jog_h.addWidget(btn_p)
            grid.addLayout(jog_h, row, 1)
            
            # 목표값
            target_lbl = QLabel("0.0")
            target_lbl.setFixedSize(60, 28)
            target_lbl.setStyleSheet("background-color: white; border: 1px solid #2196F3; font-size: 10px;")
            grid.addWidget(target_lbl, row, 2)
            
            # 현재값
            current_lbl = QLabel("0.0")
            current_lbl.setFixedSize(60, 28)
            current_lbl.setStyleSheet("background-color: white; border: 1px solid #757575; font-size: 10px;")
            grid.addWidget(current_lbl, row, 3)
            
            # 오차
            error_lbl = QLabel("0.0")
            error_lbl.setFixedSize(60, 28)
            error_lbl.setStyleSheet("background-color: #C8E6C9; border: 1px solid #f44336; font-size: 10px;")
            grid.addWidget(error_lbl, row, 4)
            
            # 메모리 Pos1~5
            for m in range(1, 6):
                mem_lbl = QLabel("---")
                mem_lbl.setFixedSize(55, 28)
                mem_lbl.setStyleSheet("background-color: #555; color: white; border: 1px solid #999; font-size: 9px;")
                mem_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                grid.addWidget(mem_lbl, row, 4 + m)
        
        jog_group.setLayout(grid)
        right_layout.addWidget(jog_group)
        
        # 메모리 저장/이동 버튼 라인
        mem_btn_layout = QHBoxLayout()
        for m in range(1, 6):
            btn_v_layout = QVBoxLayout()
            btn_v_layout.setSpacing(2)
            
            save_btn = QPushButton("저장")
            save_btn.setFixedSize(55, 22)
            save_btn.setStyleSheet("font-size: 9px; background-color: #1976D2;")
            save_btn.clicked.connect(lambda ch, slot=m: self.save_memory(slot))
            
            move_btn = QPushButton("이동")
            move_btn.setFixedSize(55, 22)
            move_btn.setStyleSheet("font-size: 9px; background-color: #2E7D32;")
            move_btn.clicked.connect(lambda ch, slot=m: self.move_memory(slot))
            
            btn_v_layout.addWidget(save_btn)
            btn_v_layout.addWidget(move_btn)
            mem_btn_layout.addLayout(btn_v_layout)
        
        mem_btn_layout.addStretch()
        right_layout.addLayout(mem_btn_layout)
        
        # 좌표 컨트롤러
        cart_group = QGroupBox("🎯 좌표 컨트롤러")
        c_grid = QGridLayout()
        c_grid.setSpacing(6)
        
        c_headers = ["축", "Jog", "목표", "현재", "오차", "Pos1", "Pos2", "Pos3", "Pos4", "Pos5"]
        for col, text in enumerate(c_headers):
            c_grid.addWidget(QLabel(text), 0, col, alignment=Qt.AlignmentFlag.AlignCenter)
        
        axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]
        for i in range(6):
            row = i + 1
            
            axis_lbl = QLabel(axes[i])
            axis_lbl.setFixedWidth(60)
            axis_lbl.setStyleSheet("background-color: #E0E0E0; color: black; border-radius: 3px;")
            c_grid.addWidget(axis_lbl, row, 0)
            
            # Jog
            jog_h = QHBoxLayout()
            jog_h.setSpacing(2)
            btn_m = QPushButton("-")
            btn_m.setFixedSize(30, 28)
            btn_p = QPushButton("+")
            btn_p.setFixedSize(30, 28)
            jog_h.addWidget(btn_m)
            jog_h.addWidget(btn_p)
            c_grid.addLayout(jog_h, row, 1)
            
            target_lbl = QLabel("0.0")
            target_lbl.setFixedSize(60, 28)
            target_lbl.setStyleSheet("background-color: white; border: 1px solid #2196F3; font-size: 10px;")
            c_grid.addWidget(target_lbl, row, 2)
            
            current_lbl = QLabel("0.0")
            current_lbl.setFixedSize(60, 28)
            current_lbl.setStyleSheet("background-color: white; border: 1px solid #757575; font-size: 10px;")
            c_grid.addWidget(current_lbl, row, 3)
            
            error_lbl = QLabel("0.0")
            error_lbl.setFixedSize(60, 28)
            error_lbl.setStyleSheet("background-color: #C8E6C9; border: 1px solid #f44336; font-size: 10px;")
            c_grid.addWidget(error_lbl, row, 4)
            
            # 메모리 Pos1~5
            for m in range(1, 6):
                mem_lbl = QLabel("---")
                mem_lbl.setFixedSize(55, 28)
                mem_lbl.setStyleSheet("background-color: #555; color: white; border: 1px solid #999; font-size: 9px;")
                mem_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                c_grid.addWidget(mem_lbl, row, 4 + m)
        
        cart_group.setLayout(c_grid)
        right_layout.addWidget(cart_group)
        
        # 좌표 메모리 저장/이동 버튼 라인
        pose_mem_btn_layout = QHBoxLayout()
        for m in range(1, 6):
            btn_v_layout = QVBoxLayout()
            btn_v_layout.setSpacing(2)
            
            save_btn = QPushButton("저장")
            save_btn.setFixedSize(55, 22)
            save_btn.setStyleSheet("font-size: 9px; background-color: #1976D2;")
            save_btn.clicked.connect(lambda ch, slot=m: self.save_pose_memory(slot))
            
            move_btn = QPushButton("이동")
            move_btn.setFixedSize(55, 22)
            move_btn.setStyleSheet("font-size: 9px; background-color: #2E7D32;")
            move_btn.clicked.connect(lambda ch, slot=m: self.move_pose_memory(slot))
            
            btn_v_layout.addWidget(save_btn)
            btn_v_layout.addWidget(move_btn)
            pose_mem_btn_layout.addLayout(btn_v_layout)
        
        pose_mem_btn_layout.addStretch()
        
        cart_group.setLayout(c_grid)
        right_layout.addWidget(cart_group)
        right_layout.addLayout(pose_mem_btn_layout)
        right_layout.addStretch()
        
        scroll.setWidget(scroll_content)
        main_h_layout.addWidget(scroll)
        
        return tab
    
    # --- 더미 제어 메서드 ---
    def send_servo(self):
        self.lbl_status.setText("Status: Servo 컨트롤")
    
    def go_home(self):
        self.lbl_status.setText("Status: HOME 위치로 이동")
    
    def control_gripper(self):
        self.lbl_status.setText("Status: 그리퍼 컨트롤")
    
    def save_memory(self, slot):
        """메모리 슬롯에 현재 관절각 저장"""
        self.lbl_status.setText(f"Status: Slot {slot} 저장됨")
    
    def move_memory(self, slot):
        """메모리 슬롯의 위치로 이동"""
        self.lbl_status.setText(f"Status: Slot {slot} 위치로 이동 중")
    
    def save_pose_memory(self, slot):
        """메모리 슬롯에 현재 좌표 저장"""
        self.lbl_status.setText(f"Status: Pose Slot {slot} 저장됨")
    
    def move_pose_memory(self, slot):
        """메모리 슬롯의 좌표로 이동"""
        self.lbl_status.setText(f"Status: Pose Slot {slot}로 이동 중")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    sys.exit(app.exec())