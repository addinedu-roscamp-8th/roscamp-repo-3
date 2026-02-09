"""
포즈 메모리 관리 (P1~P5 좌표 저장/로드)
"""
import csv
from pathlib import Path


class PoseMemoryManager:
    """로봇팔 포즈 메모리 CSV 관리"""

    def __init__(self, config_dir="config", slot_count: int = 8):
        """
        Args:
            config_dir: 설정 파일 디렉토리 경로
            slot_count: 저장 슬롯 개수 (기본 8)
        """
        self.pose_memory_dir = Path(config_dir) / "pose_memory"
        self.pose_memory_dir.mkdir(parents=True, exist_ok=True)
        self.slot_count = int(slot_count)
        # logger: optional callable that accepts a single string argument
        self.logger = None

    def set_logger(self, logger_callable):
        """Set an external logger callable (e.g., widget's work_log_signal.emit)

        The callable should accept a single string argument.
        """
        self.logger = logger_callable

    def _log(self, msg: str):
        if self.logger:
            try:
                self.logger(str(msg))
            except Exception:
                # fallback to print if logger fails
                print(msg)
        else:
            print(msg)
    
    def _get_csv_path(self, robot_name):
        """로봇 이름을 기반으로 CSV 파일 경로 반환"""
        robot_id = robot_name.replace(" ", "_")
        return self.pose_memory_dir / f"{robot_id}_pose_memory.csv"
    
    def load(self, robot_name):
        """로봇의 좌표 메모리 반환 (CSV에서 로드)

        유지보수 편의를 위해 내부적으로 라벨(네이밍)도 함께 저장할 수 있도록
        `load_with_labels`를 별도로 제공합니다. 기존 `load`는 좌표 딕셔너리만 반환합니다.
        """
        pose_dict, _labels = self.load_with_labels(robot_name)
        return pose_dict

    def load_with_labels(self, robot_name):
        """CSV에서 좌표와 슬롯 라벨을 함께 로드합니다.

        Returns:
            (pose_dict, label_dict)
            pose_dict: {"1": [x..y], ...}
            label_dict: {1: "상차P1", ...}
        """
        pose_dict = {str(i): [0.0] * 6 for i in range(1, self.slot_count + 1)}
        label_dict = {i: f"P{i}" for i in range(1, self.slot_count + 1)}

        csv_path = self._get_csv_path(robot_name)
        if csv_path.exists():
            try:
                with open(csv_path, 'r', encoding='utf-8') as f:
                    reader = csv.reader(f)
                    header = next(reader, None)
                    for row in reader:
                        # 기대 형식: Slot, Label, X, Y, Z, R, P, Y
                        if len(row) >= 8:
                            # Slot 컬럼: 'P1' 또는 '1'
                            slot_str = row[0].replace('P', '') if row[0].startswith('P') else row[0]
                            try:
                                slot_num = int(slot_str)
                            except ValueError:
                                continue
                            label = row[1] if row[1] else f"P{slot_num}"
                            try:
                                coords = [float(x) for x in row[2:8]]
                            except Exception:
                                coords = [0.0] * 6
                            pose_dict[str(slot_num)] = coords
                            label_dict[slot_num] = label
                self._log(f"✅ CSV 로드 완료: {csv_path}")
            except Exception as e:
                self._log(f"❌ CSV 읽기 오류 ({robot_name}): {str(e)}")

        return pose_dict, label_dict
    
    def save(self, robot_name, slot, coords):
        """좌표 메모리 저장 (CSV 형식)"""
        csv_path = self._get_csv_path(robot_name)
        # 전체 메모리와 라벨 로드
        pose_dict, label_dict = self.load_with_labels(robot_name)
        pose_dict[str(slot)] = coords

        # CSV에 저장 (Slot, Label, X, Y, Z, R, P, Y)
        try:
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['Slot', 'Label', 'X(mm)', 'Y(mm)', 'Z(mm)', 'R(°)', 'P(°)', 'Y(°)'])
                for slot_num in range(1, self.slot_count + 1):
                    slot_str = str(slot_num)
                    values = pose_dict.get(slot_str, [0.0]*6)
                    label = label_dict.get(slot_num, f"P{slot_num}")
                    writer.writerow([f'P{slot_num}', label] + values)

            self._log(f"✅ CSV 저장 완료: {csv_path}")
        except Exception as e:
            self._log(f"❌ CSV 저장 오류 ({robot_name}): {str(e)}")
