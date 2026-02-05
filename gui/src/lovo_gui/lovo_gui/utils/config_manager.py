"""
설정 파일 관리
"""
import json
import os
import csv
from pathlib import Path


class ConfigManager:
    """robotname.json 설정 파일 관리"""
    
    def __init__(self, config_path="config/robotname.json"):
        self.config_path = config_path
        self.config = {}
        # CSV 저장 디렉토리
        self.pose_memory_dir = Path(config_path).parent / "pose_memory"
        self.pose_memory_dir.mkdir(exist_ok=True)
        self.load()
    
    def load(self):
        """설정 파일 로드"""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    self.config = json.load(f)
            except:
                self._set_default()
        else:
            self._set_default()
    
    def _set_default(self):
        """기본 설정 생성"""
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
        self.save()
    
    def save(self):
        """설정 파일 저장"""
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(self.config, f, indent=4, ensure_ascii=False)
    
    def get_robots(self):
        """로봇 리스트 반환"""
        return self.config.get("robots", [])
    
    def get_server_domain(self):
        """서버 도메인 반환"""
        return self.config.get("server_domain", 70)
    
    def update_robot_name(self, index, new_name):
        """로봇 이름 업데이트"""
        if index < len(self.config.get("robots", [])):
            self.config["robots"][index]["name"] = new_name
            self.save()
            return True
        return False
    
    def get_robot_by_index(self, index):
        """인덱스로 로봇 정보 가져오기"""
        robots = self.get_robots()
        if 0 <= index < len(robots):
            return robots[index]
        return None
    
    # ==================== 좌표 메모리 관리 (P1~P5) CSV 형식 ====================
    
    def _get_pose_memory_csv_path(self, robot_name):
        """로봇 이름을 기반으로 CSV 파일 경로 반환"""
        # 파일명: robot1_pose_memory.csv, robot2_pose_memory.csv 등
        robot_id = robot_name.replace(" ", "_")
        return self.pose_memory_dir / f"{robot_id}_pose_memory.csv"
    
    def get_pose_memory(self, robot_name):
        """로봇의 좌표 메모리 반환 (CSV에서 로드)"""
        pose_dict = {str(i): [0.0] * 6 for i in range(1, 6)}
        
        csv_path = self._get_pose_memory_csv_path(robot_name)
        if csv_path.exists():
            try:
                with open(csv_path, 'r', encoding='utf-8') as f:
                    reader = csv.reader(f)
                    next(reader)  # 헤더 스킵
                    for row in reader:
                        if len(row) >= 7:
                            # "P1" -> "1"로 변환
                            slot_str = row[0].replace('P', '') if row[0].startswith('P') else row[0]
                            coords = [float(x) for x in row[1:7]]
                            pose_dict[slot_str] = coords
                print(f"✅ CSV 로드 완료: {csv_path}")
            except Exception as e:
                print(f"❌ CSV 읽기 오류 ({robot_name}): {str(e)}")
        
        return pose_dict
    
    def save_pose_memory(self, robot_name, slot, coords):
        """좌표 메모리 저장 (CSV 형식)"""
        csv_path = self._get_pose_memory_csv_path(robot_name)
        
        # 전체 메모리 로드
        pose_dict = self.get_pose_memory(robot_name)
        pose_dict[str(slot)] = coords
        
        # CSV에 저장
        try:
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 헤더
                writer.writerow(['Slot', 'X(mm)', 'Y(mm)', 'Z(mm)', 'R(°)', 'P(°)', 'Y(°)'])
                # 데이터
                for slot_num in range(1, 6):
                    slot_str = str(slot_num)
                    values = pose_dict.get(slot_str, [0.0]*6)
                    writer.writerow([f'P{slot_num}'] + values)
            
            print(f"✅ CSV 저장 완료: {csv_path}")
        except Exception as e:
            print(f"❌ CSV 저장 오류 ({robot_name}): {str(e)}")
    
    def load_all_pose_memory(self, robot_name):
        """로봇의 모든 좌표 메모리 로드"""
        return self.get_pose_memory(robot_name)
