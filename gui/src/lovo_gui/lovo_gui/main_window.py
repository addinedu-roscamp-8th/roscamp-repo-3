"""
Lovo 제어 시스템 메인 윈도우
"""
import sys
from PyQt6.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTabWidget
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

#######

class MyMainWindow(QMainWindow):
    """메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        
        # 설정 및 통신 매니저 초기화
        self.robot_settings = RobotSettings("config/robotname.json")
        self.comm_manager = CommunicationManager()
        
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
    
    def _create_tabs(self):
        """탭 위젯 생성"""
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.TabPosition.South)
        tabs.setStyleSheet(
            f"QTabBar::tab {{ min-height: {TAB_HEIGHT}px; min-width: {TAB_WIDTH}px; font-size: 16px; }}"
        )
        
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
        self.log_tab = LogTab()
        tabs.addTab(self.log_tab, "Log")
        
        # Manual 탭에 컨트롤러 연결
        self.manual_tab.connect_controllers(self.communication_tab)
        self.manual_tab.bind_connection_state_store(self.comm_manager)
        
        # ROS Monitor 탭에 로봇 컨트롤러 전달
        self.ros_monitor_tab.set_robot_controllers(self.communication_tab.robot_controllers)
        
        return tabs
    
    def closeEvent(self, event: QCloseEvent):
        """윈도우 종료 시 데이터 저장"""
        try:
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

    def _sync_topview_camera_button(self, *_args):
        """탑뷰 카메라 버튼 텍스트/스타일 동기화"""
        connected = bool(
            hasattr(self, "main_tab")
            and hasattr(self.main_tab, "is_topview_camera_connected")
            and self.main_tab.is_topview_camera_connected()
        )
        if connected:
            self.btn_topview_camera.setText("탑뷰카메라 해제")
            self.btn_topview_camera.setStyleSheet(STYLE_BUTTON_GRAY)
        else:
            self.btn_topview_camera.setText("탑뷰카메라 연결")
            self.btn_topview_camera.setStyleSheet(STYLE_BUTTON_ORANGE)

    def _toggle_topview_camera(self):
        """탑뷰 카메라 연결/해제 토글"""
        try:
            if hasattr(self, "main_tab") and hasattr(self.main_tab, "toggle_topview_camera"):
                self.main_tab.toggle_topview_camera()
            self._sync_topview_camera_button()
        except Exception as e:
            print(f"[UI] _toggle_topview_camera error: {e}")


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
