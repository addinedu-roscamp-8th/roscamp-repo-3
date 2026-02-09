"""
통신 관련 기능
"""
import subprocess
from datetime import datetime
from PyQt6.QtWidgets import QLabel


class CommunicationManager:
    """로봇 통신 관리"""
    
    def __init__(self, log_viewer=None):
        self.log_viewer = log_viewer
    
    def check_connection(self, ip_address, status_label, device_name):
        """Ping으로 연결 상태 확인"""
        if not ip_address:
            self.log(f"⚠️ {device_name}: IP 주소가 입력되지 않았습니다")
            return False
        
        self.log(f"🔍 {device_name} ({ip_address}) 연결 확인 중...")
        
        # Linux ping 명령어: -c 1 (1번 ping), -W 1 (1초 타임아웃)
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )
            
            if result.returncode == 0:
                self._update_status(status_label, "🟢 Online", "green")
                self.log(f"✅ {device_name} ({ip_address}) 연결 성공")
                return True
            else:
                self._update_status(status_label, "🔴 Offline", "red")
                self.log(f"❌ {device_name} ({ip_address}) 연결 실패")
                return False
                
        except subprocess.TimeoutExpired:
            self._update_status(status_label, "🔴 Timeout", "orange")
            self.log(f"⏱️ {device_name} ({ip_address}) 연결 시간 초과")
            return False
            
        except Exception as e:
            self._update_status(status_label, "🔴 Error", "red")
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
    
    def set_log_viewer(self, log_viewer):
        """로그 뷰어 설정"""
        self.log_viewer = log_viewer
