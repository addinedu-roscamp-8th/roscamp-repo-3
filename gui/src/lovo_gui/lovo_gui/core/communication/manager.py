"""Communication manager shared across UI tabs."""
import subprocess
from datetime import datetime

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QLabel

from lovo_gui.core.api_client import APIClient
from .state_store import ConnectionStateStore


class CommunicationManager(QObject):
    """로봇 통신 관리"""
    server_enabled_changed = pyqtSignal(bool)

    def __init__(self, log_viewer=None):
        super().__init__()
        self.log_viewer = log_viewer
        self.api_client = APIClient()
        self.state_store = ConnectionStateStore()
        # 서버 폴링 허용 플래그 (초기값: 비활성화)
        self.server_enabled = False

    def check_connection(self, ip_address, status_label, device_name, robot_id=None):
        """연결 상태 확인 (서버: API, 로봇: Ping)"""
        if not ip_address:
            self.log(f"⚠️ {device_name}: IP 주소가 입력되지 않았습니다")
            return False

        self.log(f"🔍 {device_name} ({ip_address}) 연결 확인 중...")

        # 서버인 경우 API 통신 확인
        if device_name == "서버":
            return self._check_server_connection(ip_address, status_label)

        # 그 외(로봇)는 Ping으로 확인
        return self._check_ping(ip_address, status_label, device_name, robot_id=robot_id)

    def _check_server_connection(self, ip_address, status_label):
        """서버 API 연결 확인"""
        url = ip_address
        if ":" not in ip_address:
            url = f"{ip_address}:5000"

        self.api_client.set_base_url(url)

        if self.api_client.check_health():
            self._update_status(status_label, "🟢 Online", "green")
            self.state_store.set_server_reachable(True)
            self.log(f"✅ 서버 ({url}) API 연결 성공")
            return True

        self._update_status(status_label, "🔴 Offline", "red")
        self.state_store.set_server_reachable(False)
        self.log(f"❌ 서버 ({url}) API 연결 실패")
        return False

    def _check_ping(self, ip_address, status_label, device_name, robot_id=None):
        """Ping으로 기기 연결 확인"""
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )

            if result.returncode == 0:
                self._update_status(status_label, "🟢 Online", "green")
                if robot_id:
                    self.state_store.set_robot_ping(robot_id, True)
                self.log(f"✅ {device_name} ({ip_address}) 연결 성공")
                return True

            self._update_status(status_label, "🔴 Offline", "red")
            if robot_id:
                self.state_store.set_robot_ping(robot_id, False)
            self.log(f"❌ {device_name} ({ip_address}) 연결 실패")
            return False

        except subprocess.TimeoutExpired:
            self._update_status(status_label, "🔴 Timeout", "orange")
            if robot_id:
                self.state_store.set_robot_ping(robot_id, False)
            self.log(f"⏱️ {device_name} ({ip_address}) 연결 시간 초과")
            return False

        except Exception as e:
            self._update_status(status_label, "🔴 Error", "red")
            if robot_id:
                self.state_store.set_robot_ping(robot_id, False)
            self.log(f"⚠️ {device_name} ({ip_address}) 오류: {str(e)}")
            return False

    def _update_status(self, label, text, color):
        """상태 라벨 업데이트"""
        if isinstance(label, QLabel):
            label.setText(text)
            label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def log(self, message):
        """통신 로그에 메시지 추가"""
        if self.log_viewer:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log_entry = f"[{timestamp}] {message}"
            self.log_viewer.append(log_entry)
            sb = self.log_viewer.verticalScrollBar()
            sb.setValue(sb.maximum())

    def set_log_viewer(self, log_viewer):
        """로그 뷰어 설정"""
        self.log_viewer = log_viewer

    def set_server_enabled(self, enabled: bool):
        """서버 폴링 허용 여부 설정"""
        self.server_enabled = bool(enabled)
        self.server_enabled_changed.emit(self.server_enabled)
