"""
포즈 메모리 관리 (P1~P5 좌표 저장/로드)
"""
import csv
from pathlib import Path


class PoseMemoryManager:
    """로봇팔 포즈 메모리 CSV 관리"""
    
    def __init__(self, config_dir="config"):
        """
        Args:
            config_dir: 설정 파일 디렉토리 경로
        """
        self.pose_memory_dir = Path(config_dir) / "pose_memory"
        self.pose_memory_dir.mkdir(parents=True, exist_ok=True)
    
    def _get_csv_path(self, robot_name):
        """로봇 이름을 기반으로 CSV 파일 경로 반환"""
        robot_id = robot_name.replace(" ", "_")
        return self.pose_memory_dir / f"{robot_id}_pose_memory.csv"
    
    def load(self, robot_name):
        """로봇의 좌표 메모리 반환 (CSV에서 로드)"""
        pose_dict = {str(i): [0.0] * 6 for i in range(1, 6)}
        
        csv_path = self._get_csv_path(robot_name)
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
    
    def save(self, robot_name, slot, coords):
        """좌표 메모리 저장 (CSV 형식)"""
        csv_path = self._get_csv_path(robot_name)
        
        # 전체 메모리 로드
        pose_dict = self.load(robot_name)
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
