# Pinky Line Follower (IR 센서 기반 라인 트레이서)

이 패키지는 Pinky 로봇의 하단에 장착된 3개의 IR 센서를 활용하여 검은색 라인을 추적하며 자율 주행하는 ROS 2 노드를 제공합니다. PD 제어기를 통해 부드러운 주행을 구현하였으며, 라인을 이탈했을 때의 복구 로직이 포함되어 있습니다.

## 🌟 주요 기능

- **3채널 IR 센서 융합**: 왼쪽, 중앙, 오른쪽 센서의 아날로그 값을 읽어 바닥의 상태(라인/배경)를 판단합니다.
- **PD 제어 (Proportional-Derivative)**: 에러값에 비례 및 미분 제어를 적용하여 급격한 조향 변화를 방지하고 속도감 있는 라인 추적을 가능하게 합니다.
- **라인 이탈 알림 및 복구**: 로봇이 라인을 완전히 벗어난 경우, 마지막으로 감지되었던 방향을 기억하여 그 방향으로 회전하며 다시 라인을 찾는 탐색 모드로 전환됩니다.
- **파라미터 튜닝**: `ros2 param`을 통해 로봇의 주행 속도, 제어 이득(Gain), 센서 임계값 등을 실시간으로 조정할 수 있습니다.

## 🛠 시스템 요구 사항

- **OS**: Ubuntu 22.04 / 24.04
- **ROS 2**: Jazzy (또는 Humble 이상 호환 가능)
- **Dependencies**: 
  - `geometry_msgs`
  - `std_msgs`
  - `pinky_sensor_adc` (IR 센서 값을 발행하는 하드웨어 드라이버)

## 🏗 빌드 방법

```bash
# 워크스페이스로 이동
cd ~/pinky_pro

# 패키지 빌드
colcon build --packages-select pinky_line_follower

# 환경 설정 불러오기
source install/setup.bash
```

## 🚀 실행 및 테스트

### 1. 기본 실행
로봇의 센서 및 모터 드라이버가 이미 실행 중(`bringup_robot.launch.xml`)이어야 합니다.

```bash
ros2 launch pinky_line_follower line_follower.launch.py
```

### 2. 센서 데이터 모니터링
바닥 재질에 따른 IR 수치를 확인하려면 아래 명령어를 사용하세요.
```bash
ros2 topic echo /ir_sensor/range
```

## ⚙️ 설정 파라미터 (Parameters)

| 파라미터명 | 타입 | 기본값 | 설명 |
| :--- | :--- | :--- | :--- |
| `speed` | double | `0.1` | 전진 선속도 (m/s) |
| `kp` | double | `0.8` | 비례 제어 이득 (조향 민감도) |
| `kd` | double | `0.2` | 미분 제어 이득 (주행 안정성) |
| `threshold` | int | `500` | 라인 감지 기준값 (입력 > Threshold = 검은색) |

### 파라미터 실시간 조정 예시
```bash
# 더 빠른 속도로 설정
ros2 param set /line_follower speed 0.15

# 조향을 더 민감하게 설정
ros2 param set /line_follower kp 1.2
```

## 📐 Topic 구조

- **구독 (Subscribed Topics)**
  - `/ir_sensor/range` ([std_msgs/UInt16MultiArray]): IR 센서 데이터 (배열 형태: [L, C, R])
- **발행 (Published Topics)**
  - `/cmd_vel` ([geometry_msgs/Twist]): 로봇의 이동 및 조향 명령

## 💡 성능 최적화 팁

1. **Threshold 설정**: 라인 위(검은색)와 배경(흰색)의 센서 값을 측정하여 그 중간값을 `threshold`로 설정하는 것이 가장 정확합니다.
2. **PD 이득 조정**: 
   - 흔들림이 심하면 `kd` 값을 높여 댐핑을 줍니다.
   - 곡선 주행이 부족하면 `kp` 값을 높입니다.
3. **조명 주의**: 적외선 센서는 주변 조명에 민감하므로 환경이 바뀌면 `threshold`를 다시 확인해야 합니다.
