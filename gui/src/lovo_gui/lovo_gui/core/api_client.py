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
    
    def __init__(self, base_url="http://192.168.0.30:4000", timeout=2.0):
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
        """로봇 상태 조회"""
        try:
            response = requests.get(f"{self.base_url}/api/robots", timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"Error fetching robots: {e}")
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

# 전역에서 사용할 수 있는 싱글턴 인스턴스 (필요 시)
api_client = APIClient()
