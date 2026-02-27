"""
API Client for Main Server communication
"""
import requests
import json
from PyQt6.QtCore import QObject, pyqtSignal, QThread

class APIClient(QObject):
    """
    Main Server와 통신하는 API 클라이언트
    """
    
#    def __init__(self, base_url="http://10.160.141.77:5000", timeout=2.0):
    def __init__(self, base_url="http://192.168.0.30:5000", timeout=2.0):
        super().__init__()
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        
    def set_base_url(self, url):
        """기본 URL 설정"""
        if not url.startswith("http"):
            url = f"http://{url}"
        self.base_url = url.rstrip('/')

    def check_health(self):
        """서버 상태 확인 (간단한 API 호출)"""
        try:
            # 루트 경로가 없으므로 /api/products 등 가벼운 API 호출
            response = requests.get(f"{self.base_url}/api/products", timeout=self.timeout)
            return response.status_code == 200
        except Exception:
            return False

    def get_products(self):
        """제품 목록 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/products", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching products: {e}")
        return []

    def get_materials(self):
        """자재 목록 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/materials", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching materials: {e}")
        return []

    def get_orders(self):
        """주문 목록 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/orders", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching orders: {e}")
        return []
        
    def get_robots(self):
        """로봇 상태 조회 (통합)"""
        try:
            response = requests.get(f"{self.base_url}/api/robots", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robots: {e}")
        return []

    def get_robot_arms(self):
        """로봇팔 상태 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/robots/arms", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robot arms: {e}")
        return []

    def get_robot_pinkies(self):
        """운송 로봇 상태 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/robots/pinkies", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robot pinkies: {e}")
        return []

    def create_order(self, order_data):
        """주문 생성"""
        try:
            headers = {'Content-Type': 'application/json'}
            response = requests.post(
                f"{self.base_url}/api/orders", 
                data=json.dumps(order_data),
                headers=headers,
                timeout=self.timeout
            )
            return response.json() if response.status_code in [200, 201] else None
        except Exception as e:
            print(f"Error creating order: {e}")
        return None

    def get_customers(self):
        """고객 목록 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/customers", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching customers: {e}")
        return []

    def get_charging_stations(self):
        """충전소 목록 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/charging-stations", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching charging stations: {e}")
        return []

    def get_inventory_logs(self):
        """재고 로그 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/inventory-logs", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching inventory logs: {e}")
        return []

    def get_robot_jobs(self):
        """로봇 작업 로그 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/robot-jobs", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robot jobs: {e}")
        return []

    def get_robot_logs(self):
        """로봇 상태 로그 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/robot-logs", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robot logs: {e}")
        return []

    def get_fire_logs(self):
        """화재 감지 로그 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/fire-logs", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching fire logs: {e}")
        return []
    
    # ==================== AI Server 이미지 저장 제어 ====================
    
    def _get_ai_server_url(self):
        """AI 서버 URL 반환 (메인 서버 IP + 포트 8000)"""
        try:
            # "http://192.168.0.30:5000" -> "192.168.0.30"
            ip_with_port = self.base_url.replace("http://", "").replace("https://", "")
            ip_address = ip_with_port.split(":")[0]
            return f"http://{ip_address}:8000"
        except Exception:
            return "http://127.0.0.1:8000"  # 기본값
    
    def enable_ai_image_saving(self):
        """AI 이벤트 이미지 저장 활성화"""
        try:
            ai_url = self._get_ai_server_url()
            response = requests.post(f"{ai_url}/api/image-saving/enable", timeout=self.timeout)
            if response.status_code == 200:
                print(f"✅ AI 이미지 저장 활성화")
                return True
        except Exception as e:
            print(f"❌ AI 이미지 저장 활성화 실패: {e}")
        return False
    
    def disable_ai_image_saving(self):
        """AI 이벤트 이미지 저장 비활성화"""
        try:
            ai_url = self._get_ai_server_url()
            response = requests.post(f"{ai_url}/api/image-saving/disable", timeout=self.timeout)
            if response.status_code == 200:
                print(f"✅ AI 이미지 저장 비활성화")
                return True
        except Exception as e:
            print(f"❌ AI 이미지 저장 비활성화 실패: {e}")
        return False
    
    def get_ai_image_saving_status(self):
        """AI 이벤트 이미지 저장 상태 조회"""
        try:
            ai_url = self._get_ai_server_url()
            response = requests.get(f"{ai_url}/api/image-saving/status", timeout=self.timeout)
            if response.status_code == 200:
                data = response.json()
                return data.get("image_saving_enabled", False)
        except Exception as e:
            print(f"Error fetching AI image saving status: {e}")
        return False

# 전역에서 사용할 수 있는 싱글턴 인스턴스 (필요 시)
api_client = APIClient()
