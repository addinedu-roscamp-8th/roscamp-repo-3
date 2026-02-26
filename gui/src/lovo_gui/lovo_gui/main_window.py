"""
Lovo 제어 시스템 메인 윈도우
"""
import sys
from PyQt6.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTabWidget, QLabel, QSpinBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QCloseEvent, QScreen

from lovo_gui.tabs.tab05_communication import RobotSettings, CommunicationManager
from lovo_gui.constants import (
    WINDOW_WIDTH, WINDOW_HEIGHT, SIDEBAR_WIDTH, SIDEBAR_BUTTON_HEIGHT,
    STYLE_BUTTON_GREEN, STYLE_BUTTON_RED, STYLE_BUTTON_YELLOW, STYLE_BUTTON_ORANGE, STYLE_BUTTON_GRAY,
    COLOR_DARK_BG, TAB_HEIGHT, TAB_WIDTH
)
from lovo_gui.tabs.tab01_main import MainTab
from lovo_gui.tabs.tab02_manual import ManualTab
from lovo_gui.tabs.tab04_ros_monitor import RosMonitorTab
from lovo_gui.tabs.tab05_communication import CommunicationTab
from lovo_gui.tabs.tab06_log import LogTab
from lovo_gui.dialogs import FireAlertDialog

#######

class MyMainWindow(QMainWindow):
    """메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        
        # 설정 및 통신 매니저 초기화
        self.robot_settings = RobotSettings("config/robotname.json")
        self.comm_manager = CommunicationManager()
        
        # 화재 다이얼로그 (필요 시 표시)
        self.fire_dialog = None
        
        self.setWindowTitle("Lovo 제어 시스템")
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.menuBar().hide()
        
        # 화면 가용 영역에 맞게 크기 조정
        screen = QApplication.primaryScreen()
        if screen:
            available_geometry = screen.availableGeometry()
            self.resize(available_geometry.width(), available_geometry.height())
            self.move(available_geometry.x(), available_geometry.y())
        else:
            self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃 (수평: 탭 + 사이드바)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 왼쪽: 탭 영역
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(left_widget, 1)
        
        # 탭 위젯 생성
        self.tabs = self._create_tabs()
        left_layout.addWidget(self.tabs)

        # 서버 상태 동기화 구독
        self.comm_manager.server_enabled_changed.connect(self._on_server_state_sync)

        # 오른쪽: 사이드바
        sidebar = self._create_sidebar()
        main_layout.addWidget(sidebar)
    
    def _create_sidebar(self):
        """사이드바 생성"""
        sidebar = QWidget()
        sidebar.setFixedWidth(SIDEBAR_WIDTH)
        sidebar.setStyleSheet(f"background-color: {COLOR_DARK_BG};")
        
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(10, 0, 10, 20)
        layout.setSpacing(15)
        
        # 버튼들
        btn_run = QPushButton("운전")
        btn_run.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        btn_run.setStyleSheet(STYLE_BUTTON_GREEN)
        
        btn_stop = QPushButton("정지")
        btn_stop.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        btn_stop.setStyleSheet(STYLE_BUTTON_RED)
        
        btn_reset = QPushButton("초기화")
        btn_reset.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        btn_reset.setStyleSheet(STYLE_BUTTON_YELLOW)

        self.btn_topview_camera = QPushButton()
        self.btn_topview_camera.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        self.btn_topview_camera.clicked.connect(self._toggle_topview_camera)
        self._sync_topview_camera_button()

        if hasattr(self, "main_tab") and hasattr(self.main_tab, "topview_camera_state_changed"):
            self.main_tab.topview_camera_state_changed.connect(self._sync_topview_camera_button)

        # 서버 연결 토글 버튼 (인스턴스 속성으로 만들어 상태 변경 가능하게 함)
        if getattr(self.comm_manager, 'server_enabled', False):
            server_text = "서버 연결 해제"
            server_style = STYLE_BUTTON_GRAY
        else:
            server_text = "서버 연결"
            server_style = STYLE_BUTTON_ORANGE

        self.btn_server_disconnect = QPushButton(server_text)
        self.btn_server_disconnect.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        self.btn_server_disconnect.setStyleSheet(server_style)
        self.btn_server_disconnect.clicked.connect(self._server_disconnect)
        
        btn_exit = QPushButton("종료")
        btn_exit.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        btn_exit.setStyleSheet(STYLE_BUTTON_GRAY)
        btn_exit.clicked.connect(QApplication.quit)
        
        layout.addWidget(btn_run)
        layout.addWidget(btn_stop)
        layout.addWidget(btn_reset)
        layout.addWidget(self.btn_topview_camera)
        layout.addWidget(self.btn_server_disconnect)
        layout.addStretch()
        layout.addWidget(btn_exit)
        
        return sidebar
    
    def _create_domain_widget(self):
        """탭 바 코너용 도메인 ID 위젯 생성"""
        domain_widget = QWidget()
        domain_widget.setStyleSheet(f"background-color: {COLOR_DARK_BG};")
        
        layout = QHBoxLayout(domain_widget)
        layout.setContentsMargins(20, 5, 10, 5)
        layout.setSpacing(8)
        
        # 도메인 ID 라벨
        domain_label = QLabel("🌐 Domain:")
        domain_label.setStyleSheet("color: white; font-size: 12px; font-weight: bold;")
        layout.addWidget(domain_label)
        
        # 도메인 ID 입력 (SpinBox)
        self.domain_spinbox = QSpinBox()
        self.domain_spinbox.setRange(0, 101)
        self.domain_spinbox.setValue(self.robot_settings.get_server_domain())
        self.domain_spinbox.setFixedWidth(70)
        self.domain_spinbox.setFixedHeight(30)
        self.domain_spinbox.setStyleSheet("""
            QSpinBox {
                background-color: #2d2d2d;
                color: white;
                border: 2px solid #555;
                border-radius: 3px;
                padding: 3px;
                font-size: 12px;
            }
            QSpinBox::up-button, QSpinBox::down-button {
                background-color: #444;
                border: 1px solid #666;
            }
        """)
        layout.addWidget(self.domain_spinbox)
        
        # 적용 버튼
        btn_apply = QPushButton("적용")
        btn_apply.setFixedSize(60, 30)
        btn_apply.setStyleSheet("""
            QPushButton {
                background-color: #1976D2;
                color: white;
                border: none;
                border-radius: 3px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1565C0;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        btn_apply.clicked.connect(self._apply_domain_id)
        layout.addWidget(btn_apply)
        
        return domain_widget
    
    def _apply_domain_id(self):
        """도메인 ID 적용"""
        new_domain = self.domain_spinbox.value()
        current_domain = self.robot_settings.get_server_domain()
        
        if new_domain == current_domain:
            print(f"[UI] 도메인 ID가 동일합니다: {new_domain}")
            return
        
        # 설정 파일에 저장
        self.robot_settings.set_server_domain(new_domain)
        
        print(f"[UI] 도메인 ID 변경: {current_domain} → {new_domain}")
        print(f"[UI] ⚠️ Communication 탭에서 재연결이 필요합니다!")
        
        # 로그 탭에 메시지 추가 (있다면)
        if hasattr(self, 'log_tab'):
            self.log_tab.add_log(f"[시스템] 도메인 ID 변경: {current_domain} → {new_domain} (재연결 필요)")
    
    def _create_tabs(self):
        """탭 위젯 생성"""
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.TabPosition.South)
        tabs.setStyleSheet(
            f"QTabBar::tab {{ min-height: {TAB_HEIGHT}px; min-width: {TAB_WIDTH}px; font-size: 16px; }}"
        )
        
        # 탭 바 우측에 도메인 ID 위젯 추가
        domain_widget = self._create_domain_widget()
        tabs.setCornerWidget(domain_widget, Qt.Corner.BottomRightCorner)
        
        # Main 탭
        self.main_tab = MainTab(self.robot_settings, self.comm_manager)
        tabs.addTab(self.main_tab, "Main")
        
        # Manual 탭
        self.manual_tab = ManualTab(self.robot_settings)
        tabs.addTab(self.manual_tab, "Manual")
        
        # Communication 탭
        self.communication_tab = CommunicationTab(self.robot_settings, self.comm_manager)
        tabs.addTab(self.communication_tab, "Communication")
        
        # ROS Monitor 탭
        self.ros_monitor_tab = RosMonitorTab(self.comm_manager)
        tabs.addTab(self.ros_monitor_tab, "ROS Monitor")
        
        # Log 탭
        self.log_tab = LogTab(self.comm_manager)
        tabs.addTab(self.log_tab, "Log")
        
        # Manual 탭에 컨트롤러 연결
        self.manual_tab.connect_controllers(self.communication_tab)
        self.manual_tab.bind_connection_state_store(self.comm_manager)
        
        # Main 탭에 로봇 컨트롤러 연결 (robot_state 시그널용)
        print(f"[DEBUG MainWindow] Main 탭에 robot_controllers 연결 중...")
        print(f"[DEBUG MainWindow] robot_controllers: {list(self.communication_tab.robot_controllers.keys())}")
        self.main_tab.connect_robot_controllers(self.communication_tab.robot_controllers)
        
        # robot1의 robot_state 시그널 연결 (화재 감지)
        if 'jekobot_126b' in self.communication_tab.robot_controllers:
            robot1_controller = self.communication_tab.robot_controllers['jekobot_126b']
            robot1_controller.robot_state_updated.connect(self._on_robot1_state_changed)
            print(f"[DEBUG MainWindow] robot1 state 시그널 연결 완료")
        
        # ROS Monitor 탭에 로봇 컨트롤러 전달
        self.ros_monitor_tab.set_robot_controllers(self.communication_tab.robot_controllers)
        
        return tabs
    
    def closeEvent(self, event: QCloseEvent):
        """윈도우 종료 시 데이터 저장"""
        try:
            # 로봇 컨트롤러 종료 플래그 설정 (race condition 방지)
            if hasattr(self, 'communication_tab') and hasattr(self.communication_tab, 'robot_controllers'):
                for robot_id, controller in self.communication_tab.robot_controllers.items():
                    if hasattr(controller, '_is_shutting_down'):
                        controller._is_shutting_down = True
            
            # Manual 탭의 모든 로봇팔 대시보드에서 메모리 저장
            if hasattr(self, 'manual_tab') and hasattr(self.manual_tab, 'dashboard_widgets'):
                for robot_id, dashboard in self.manual_tab.dashboard_widgets.items():
                    # RobotDashboardWidget인 경우만 처리
                    if (hasattr(dashboard, 'pose_memory') and 
                        hasattr(dashboard, 'config_manager') and 
                        dashboard.config_manager is not None):
                        # 저장 슬롯은 대시보드 인스턴스에 따라 가변적일 수 있으므로
                        # 대시보드의 pose_memory 키를 사용하여 모든 슬롯을 저장합니다.
                        for slot_key in list(dashboard.pose_memory.keys()):
                            try:
                                slot = int(slot_key)
                            except Exception:
                                continue
                            if slot in dashboard.pose_memory:
                                dashboard.config_manager.save_pose_memory(
                                    dashboard.robot_name,
                                    slot,
                                    dashboard.pose_memory[slot]
                                )

            # Manual 탭의 카메라 offset 값도 CSV로 저장
            if hasattr(self, 'manual_tab') and hasattr(self.manual_tab, 'camera_widgets'):
                for _robot_id, camera_widget in self.manual_tab.camera_widgets.items():
                    if hasattr(camera_widget, 'save_offsets_to_csv'):
                        camera_widget.save_offsets_to_csv()
            
            # Main 탭의 로컬 카메라 정리
            if hasattr(self, 'main_tab') and hasattr(self.main_tab, 'cleanup'):
                self.main_tab.cleanup()

            print("✅ 모든 좌표 메모리 저장 완료")
        except Exception as e:
            print(f"⚠️ 종료 시 저장 오류: {str(e)}")
        finally:
            event.accept()

    def _server_disconnect(self):
        """토글: 서버 폴링 활성/비활성 및 버튼 텍스트/스타일 업데이트"""
        try:
            # Toggle flag
            current = bool(getattr(self.comm_manager, 'server_enabled', False))
            new_state = not current
            if hasattr(self.comm_manager, 'set_server_enabled'):
                self.comm_manager.set_server_enabled(new_state)
            else:
                self.comm_manager.server_enabled = new_state

            # Update button appearance
            if new_state:
                # 서버 폴링 활성화 -> 버튼은 '해제'로 보이게
                self.btn_server_disconnect.setText("서버 연결 해제")
                self.btn_server_disconnect.setStyleSheet(STYLE_BUTTON_GRAY)
            else:
                # 비활성화 상태 -> 버튼은 '연결'로 보이게
                self.btn_server_disconnect.setText("서버 연결")
                self.btn_server_disconnect.setStyleSheet(STYLE_BUTTON_ORANGE)

            # 로그 출력
            print(f"[UI] server_enabled set to {new_state}")
        except Exception as e:
            print(f"[UI] _server_disconnect error: {e}")

    def _on_server_state_sync(self, enabled):
        """서버 상태 신호에 따라 사이드바 버튼 동기화"""
        if enabled:
            self.btn_server_disconnect.setText("서버 연결 해제")
            self.btn_server_disconnect.setStyleSheet(STYLE_BUTTON_GRAY)
        else:
            self.btn_server_disconnect.setText("서버 연결")
            self.btn_server_disconnect.setStyleSheet(STYLE_BUTTON_ORANGE)

    def _sync_topview_camera_button(self, *_args):
        """탑뷰 카메라 버튼 텍스트/스타일 동기화"""
        connected = bool(
            hasattr(self, "main_tab")
            and hasattr(self.main_tab, "is_topview_camera_connected")
            and self.main_tab.is_topview_camera_connected()
        )
        if connected:
            # 연결되어 있으면 현재 포트에 따라 텍스트 변경
            current_port = self.main_tab.get_current_topview_port() if hasattr(self.main_tab, "get_current_topview_port") else 9630
            if current_port == 9730:
                self.btn_topview_camera.setText("탑뷰 일반영상")
                self.btn_topview_camera.setStyleSheet(STYLE_BUTTON_ORANGE)
            else:
                self.btn_topview_camera.setText("탑뷰 오버레이")
                self.btn_topview_camera.setStyleSheet(STYLE_BUTTON_GRAY)
        else:
            self.btn_topview_camera.setText("탑뷰카메라 연결")
            self.btn_topview_camera.setStyleSheet(STYLE_BUTTON_ORANGE)

    def _toggle_topview_camera(self):
        """탑뷰 카메라 포트 전환 (9630 ↔ 9730)"""
        try:
            if hasattr(self, "main_tab"):
                if self.main_tab.is_topview_camera_connected():
                    # 연결되어 있으면 포트 전환
                    if hasattr(self.main_tab, "switch_topview_port"):
                        self.main_tab.switch_topview_port()
                else:
                    # 연결되어 있지 않으면 연결
                    if hasattr(self.main_tab, "connect_topview_camera"):
                        self.main_tab.connect_topview_camera()
            self._sync_topview_camera_button()
        except Exception as e:
            print(f"[UI] _toggle_topview_camera error: {e}")
    
    def _on_robot1_state_changed(self, state: int):
        """robot1의 robot_state 값이 변경될 때 호출"""
        print(f"[DEBUG MainWindow] robot1 state changed: {state}")
        
        if state == 5:
            # 화재 상황 발생 (state == 5)
            if self.fire_dialog is None or not self.fire_dialog.isVisible():
                print(f"[DEBUG MainWindow] 화재 다이얼로그 표시 (state={state})")
                self.fire_dialog = FireAlertDialog(self)
                self.fire_dialog.show()
        else:
            # 화재 상황 해제
            if self.fire_dialog is not None and self.fire_dialog.isVisible():
                print(f"[DEBUG MainWindow] 화재 다이얼로그 닫기 (state={state})")
                self.fire_dialog.close()
                self.fire_dialog = None


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
