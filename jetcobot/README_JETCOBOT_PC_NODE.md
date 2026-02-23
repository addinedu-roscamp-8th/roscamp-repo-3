# JetCobot PC Node - 실행 가이드

JetCobot의 PC side 제어 노드입니다. ArUco 마커 기반 자동 pick & place 기능을 제공합니다.

## 📋 사전 요구사항 (Prerequisites)

### Python 및 ROS2 버전
- **Python**: 3.10+
- **ROS2**: Jazzy
- **Ubuntu**: 24.04 LTS

### 주요 의존성 버전
```
pymoveit2>=0.3.0
opencv-python>=4.8.0
opencv-contrib-python>=4.8.0
PyYAML>=6.0
numpy>=1.24.0
```

## 🔧 설치 및 실행 가이드

### 1. Virtual Environment 생성

mycobot 가상 환경을 생성합니다 (Python 3.12 사용 시 아래 명령 권장):

```bash
python3.12 -m venv ~/venv/mycobot
```

**다른 Python 버전 사용 시:**
```bash
python3 -m venv ~/venv/mycobot
```

### 2. Virtual Environment 활성화

생성된 mycobot 가상 환경을 활성화합니다:

```bash
source ~/venv/mycobot/bin/activate
```

**확인:**
```bash
which python  # should show path to ~/venv/mycobot/bin/python
python --version  # should show Python 3.12+ (또는 사용 중인 버전)
```

### 3. ROS2 환경 설정

```bash
source /opt/ros/jazzy/setup.bash
```

### 4. 의존성 설치 (rosdep)

workspace의 jetcobot 폴더로 이동하여 rosdep 실행:

```bash
cd /home/addinedu/lovo/roscamp-repo-3/jetcobot

# ROS 패키지 의존성 자동 설치
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

**주의:** 이 명령어는 ROS 패키지의 native dependencies를 자동으로 설치합니다.

### 5. 시스템 의존성 설치

pymoveit2 설치 전 필요한 시스템 패키지들을 설치합니다:

```bash
sudo apt-get update
sudo apt-get install -y python3-dev build-essential
```

**필수 패키지:**
- `python3-dev`: Python 개발 헤더 파일
- `build-essential`: C/C++ 컴파일러 및 빌드 도구

### 6. Python 패키지 설치

**중요:** 이 단계는 **venv가 활성화된 상태**에서 실행해야 합니다 (2단계 참조).

```bash
# venv가 활성화되어 있는지 확인:
# (venv/mycobot) $ 프롬프트가 보여야 함

pip install --upgrade pip
pip install pymoveit2>=0.3.0
pip install opencv-python>=4.8.0
pip install opencv-contrib-python>=4.8.0
pip install PyYAML>=6.0
pip install numpy>=1.24.0
pip install scipy
```

또는 한 번에 설치:

```bash
pip install pymoveit2 opencv-python opencv-contrib-python PyYAML numpy scipy
```

**주의:** pymoveit2 설치 시 "찾을 수 없음" 오류가 발생하면:
1. ROS2 환경이 제대로 sourced되었는지 확인: `echo $ROS_DISTRO` (jazzy 출력 확인)
2. 위의 시스템 의존성 설치 완료 확인
3. rosdep 설치가 완료되었는지 확인
4. **venv가 활성화되어 있는지 확인**: `echo $VIRTUAL_ENV` (~/venv/mycobot 출력 확인)

**대안: rosdep을 통한 설치**

다른 컴퓨터에서 pip로 설치 실패 시 rosdep으로 설치할 수 있습니다:

```bash
# ROS2 환경이 sourced된 상태에서
rosdep install pymoveit2 -y
```

또는 colcon 빌드 시 자동으로 설치되므로, 다음 단계(Workspace 빌드)를 먼저 진행해도 됩니다.

### 7. Workspace 빌드

```bash
cd /home/addinedu/lovo/roscamp-repo-3/jetcobot

# 전체 빌드 (권장)
colcon build

# 또는 특정 패키지만 빌드하려면:
# colcon build --packages-select jetcobot_bringup jetcobot_description jetcobot_moveit_config lovo_interfaces

# 설치된 패키지 sourcing
source install/setup.bash
```

**참고:** `colcon build`는 jetcobot 폴더 내에서 실행해야 합니다. jetcobot이 workspace root입니다.

### 8. 실행

```bash
# 1. Launch 파일로 실행 (MoveIt 및 other components 포함)
ros2 launch jetcobot_bringup jetcobot_pc_node.py

# 또는

# 2. 직접 노드 실행
ros2 run jetcobot_bringup jetcobot_pc_node.py
```

## ⚙️ 설정 파일

### picking_poses.yaml
위치: `jetcobot_bringup/config/picking_poses.yaml`

로봇의 pick/place 위치 정의:
- `poses`: 각 위치의 좌표 (xyz_cm, rpy_deg) 및 offset 정의
- `sub_offsets`: 다른 아이템별 offset (place 위치에서)

예시:
```yaml
poses:
  "1":  # Place 위치
    xyz_cm: [30.0, 20.0, 5.0]
    rpy_deg: [0, 0, 0]
    offset_cm: [0, 0, 0]
    z_lift_cm: 10.0
    offsets:  # pick_id별 sub_offset
      "4": {offset_cm: [3.0, -6.0, 8.0], z_lift_cm: 8.0}
      "5": {offset_cm: [-2.0, 5.0, 8.0], z_lift_cm: 8.0}
```

### camera_calibration.yaml
카메라 내부 파라미터 및 손목-카메라 변환 정의.

## 📊 주요 토픽 및 서비스

### Publishers
- `/PTP_tcp_pose` (Float64MultiArray): TCP 위치/방향 [x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg]

### Subscribers
- `/order_command` (Float64): 주문 명령 (X.YZW 형식: X=place_id, YZW=pick_ids)
- `/PTP_goal_pose` (Float64MultiArray): 목표 위치 [x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg]
- `/PTP_capture_image_with_command/compressed`: 이미지 캡처 명령

### Services
- `/capture_image` (GetSnapshot): 카메라 이미지 캡처

## 🚀 사용 예시

### 1. Receipt-based Pick & Place 자동화

```bash
# 다른 터미널에서:
ros2 topic pub /order_command std_msgs/Float64 "data: 1.456"
# place_id=1, pick_ids=[4,5,6] 순서로 자동 실행
```

### 2. 수동 위치 지정

```bash
ros2 topic pub /PTP_goal_pose std_msgs/Float64MultiArray "data: [30.0, 20.0, 5.0, 0, 0, 0]"
# x=30cm, y=20cm, z=5cm, roll=pitch=yaw=0도로 이동
```

## 🐛 트러블슈팅

### TF 변환 오류
```
TF lookup failed: frame 'gripper_tcp' does not exist
```
→ 로봇 드라이버 및 state_publisher가 정상 실행 중인지 확인

### 카메라 이미지 없음
→ 카메라 서비스 (`/capture_image`)가 실행 중인지 확인

### ArUco 마커 감지 실패
→ 조명 확인, 마커 크기 설정 확인 (picking_poses.yaml의 marker_size_m)

### MoveIt 충돌
```
[move_group-2] STATUS_ABORTED
```
→ 로봇의 현재 위치가 목표 위치에 도달 불가능한 상태인지 확인

## 📝 환경 변수 확인

```bash
# Virtual environment 활성화 확인
echo $VIRTUAL_ENV  # should show ~/venv/mycobot

# ROS2 설정 확인
echo $ROS_DISTRO  # should show "jazzy"

# Python 경로 확인
which python
python -c "import pymoveit2; print(pymoveit2.__version__)"
```

## 📚 참고 자료

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [OpenCV ArUco Documentation](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)

## ✅ 체크리스트

시작 전 다음 항목들을 확인하세요:

- [ ] venv 생성: `python3 -m venv ~/venv/mycobot`
- [ ] venv 활성화: `source ~/venv/mycobot/bin/activate`
- [ ] ROS2 sourced: `source /opt/ros/jazzy/setup.bash`
- [ ] rosdep 설치 완료: `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y`
- [ ] workspace 빌드 완료: `colcon build ...`
- [ ] install/setup.bash sourced: `source install/setup.bash`
- [ ] 로봇 하드웨어 연결
- [ ] 카메라 서비스 실행 중
- [ ] 모든 요구 TF 브로드캐스트 중

준비 완료 후 `ros2 launch jetcobot_bringup jetcobot_pc_node.py` 실행!
