"""
로봇 설정 관리 (Communication 탭 소유)
- 읽기: 모든 탭에서 사용 가능
- 쓰기: Communication 탭에서만 사용
"""
import json
import os
from pathlib import Path


class RobotSettings:
    """robotname.json 설정 파일 관리"""
    
    def __init__(self, config_path="config/robotname.json"):
        # ROS 2 패키지 share 디렉토리 경로 해결 추가
        if not os.path.exists(config_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('lovo_gui')
                resolved_path = os.path.join(share_dir, 'config', os.path.basename(config_path))
                if os.path.exists(resolved_path):
                    config_path = resolved_path
            except Exception:
                pass
                
        self.config_path = config_path
        self.config = {}
        self.load()
    
    def load(self):
        """설정 파일 로드"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {self.config_path}")
        
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                self.config = json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(f"설정 파일 형식이 올바르지 않습니다: {e}")
        except Exception as e:
            raise RuntimeError(f"설정 파일 로드 중 오류 발생: {e}")
    
    def save(self):
        """설정 파일 저장 (Communication 탭에서만 사용)"""
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(self.config, f, indent=4, ensure_ascii=False)
    
    def get_robots(self):
        """로봇 리스트 반환 (모든 탭에서 사용 가능)"""
        return self.config.get("robots", [])
    
    def get_server_domain(self):
        """서버 도메인 반환 (모든 탭에서 사용 가능)"""
        return self.config.get("server_domain", 70)
    
    def set_server_domain(self, domain_id):
        """서버 도메인 설정 (UI에서 사용)"""
        self.config["server_domain"] = domain_id
        self.save()
    
    def update_robot_name(self, index, new_name):
        """로봇 이름 업데이트 (Communication 탭에서만 사용)"""
        if index < len(self.config.get("robots", [])):
            self.config["robots"][index]["name"] = new_name
            self.save()
            return True
        return False
    
    def get_robot_by_index(self, index):
        """인덱스로 로봇 정보 가져오기 (모든 탭에서 사용 가능)"""
        robots = self.get_robots()
        if 0 <= index < len(robots):
            return robots[index]
        return None
