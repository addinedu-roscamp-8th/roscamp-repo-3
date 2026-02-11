# top_view_fire 실행 가이드

## 개요
이 패키지는 카메라 영상에서 화재 객체를 탐지하고, 다음 두 좌표를 `PointStamped`로 publish 합니다.

- map 기준 화재 좌표: `/top_view/fire_map_point` (`frame_id=map`)
- 로봇 베이스 기준 화재 좌표: `/top_view/fire_base_point` (`frame_id=base_link`)

로봇 베이스 좌표는 아래 변환 행렬(`T_base_from_map`)을 사용해 계산합니다.

- 기본 경로: `/home/addinedu/workspace/fov_slam_transform/config/robot_tf_matrix.yaml`

## 변환 파이프라인
1. 픽셀 좌표 -> map 좌표 (`homography_params.yaml`의 `homography`)
2. map 좌표 -> base_link 좌표 (`robot_tf_matrix.yaml`의 `T_base_from_map`)

## 실행 방법
작업 디렉터리:

```bash
cd /home/addinedu/workspace/roscamp-repo-3/fire_detection
```

빌드:

```bash
colcon build --packages-select top_view_fire
```

환경 설정:

```bash
source install/setup.bash
```

런치 실행:

```bash
ros2 launch top_view_fire top_view_fire.launch.py
```

## 토픽 확인

```bash
ros2 topic echo /top_view/fire_map_point
ros2 topic echo /top_view/fire_base_point
```

## 주요 파일
- 노드: `fire_detection/top_view_fire/fire_map_publisher_node.py`
- 런치: `fire_detection/launch/top_view_fire.launch.py`
- 호모그래피: `fire_detection/config/homography_params.yaml`
- 변환 행렬: `/home/addinedu/workspace/fov_slam_transform/config/robot_tf_matrix.yaml`

## 참고
- 현재 launch 기본 카메라 디바이스는 `/dev/video1` 입니다.
- 모델 기본 경로는 `~/workspace/fire_detetion/results/weights/best.pt` 입니다.
