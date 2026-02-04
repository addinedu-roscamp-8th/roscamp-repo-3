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

from lovo_gui.config_manager import ConfigManager
from lovo_gui.communication import CommunicationManager
from lovo_gui.constants import (
    WINDOW_WIDTH, WINDOW_HEIGHT, SIDEBAR_WIDTH, SIDEBAR_BUTTON_HEIGHT,
    STYLE_BUTTON_GREEN, STYLE_BUTTON_RED, STYLE_BUTTON_YELLOW, STYLE_BUTTON_GRAY,
    COLOR_DARK_BG, TAB_HEIGHT, TAB_WIDTH
)
from lovo_gui.tabs.main_tab import MainTab
from lovo_gui.tabs.manual_tab import ManualTab
from lovo_gui.tabs.monitoring_tab import MonitoringTab
from lovo_gui.tabs.communication_tab import CommunicationTab
from lovo_gui.tabs.log_tab import LogTab


class MyMainWindow(QMainWindow):
    """메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        
        # 설정 및 통신 매니저 초기화
        self.config_manager = ConfigManager("robotname.json")
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
        
        # 오른쪽: 사이드바
        sidebar = self._create_sidebar()
        main_layout.addWidget(sidebar)
        
        # 탭 위젯 생성
        self.tabs = self._create_tabs()
        left_layout.addWidget(self.tabs)
    
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
        
        btn_exit = QPushButton("종료")
        btn_exit.setFixedHeight(SIDEBAR_BUTTON_HEIGHT)
        btn_exit.setStyleSheet(STYLE_BUTTON_GRAY)
        btn_exit.clicked.connect(QApplication.quit)
        
        layout.addWidget(btn_run)
        layout.addWidget(btn_stop)
        layout.addWidget(btn_reset)
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
        self.main_tab = MainTab(self.config_manager)
        tabs.addTab(self.main_tab, "Main")
        
        # Manual 탭
        self.manual_tab = ManualTab(self.config_manager)
        tabs.addTab(self.manual_tab, "Manual")
        
        # Monitoring 탭
        self.monitoring_tab = MonitoringTab()
        tabs.addTab(self.monitoring_tab, "Monitoring")
        
        # Communication 탭
        self.communication_tab = CommunicationTab(self.config_manager, self.comm_manager)
        tabs.addTab(self.communication_tab, "Communication")
        
        # Log 탭
        self.log_tab = LogTab()
        tabs.addTab(self.log_tab, "Log")
        
        # Manual 탭에 컨트롤러 연결
        self.manual_tab.connect_controllers(self.communication_tab)
        
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
                        for slot in range(1, 6):
                            if slot in dashboard.pose_memory:
                                dashboard.config_manager.save_pose_memory(
                                    dashboard.robot_name,
                                    slot,
                                    dashboard.pose_memory[slot]
                                )
            
            print("✅ 모든 좌표 메모리 저장 완료")
        except Exception as e:
            print(f"⚠️ 종료 시 저장 오류: {str(e)}")
        finally:
            event.accept()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
