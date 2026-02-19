import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # 설정 파일들과 스크립트가 위치한 디렉토리 경로 (현재 파일 기준 상대 경로 사용 권장하나, 여기서는 절대경로 수정)
    # 기존: /home/addinedu/Desktop/roscamp-repo-3/server/pinky_control_server
    # 변경: /home/addinedu/Desktop/roscamp-repo-3/server/mainserver/pinky_server
    current_dir = '/home/addinedu/Desktop/roscamp-repo-3/server/mainserver/pinky_server'
    config_dir = '/home/addinedu/Desktop/roscamp-repo-3/server/mainserver/config/bridge_pinky'
    
    launch_actions = [
        # 1. ROS_DOMAIN_ID를 59로 설정 (관제 PC용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
    ]

    # 2. Pinky 통합 도메인 브릿지 실행 (폴더 내의 모든 yaml 파일 실행)
    import glob
    yaml_files = glob.glob(os.path.join(config_dir, '*.yaml'))
    
    for yaml_file in yaml_files:
        filename = os.path.basename(yaml_file)
        launch_actions.append(
            Node(
                package='domain_bridge',
                executable='domain_bridge',
                name=f'pinky_domain_bridge_{filename.replace(".yaml", "")}',
                arguments=[yaml_file],
                output='screen'
            )
        )
    
    launch_actions.append(
        # 3. 로컬 상태 모니터링 실행 (대시보드)
        ExecuteProcess(
            cmd=['python3', os.path.join(current_dir, 'status_monitor.py')],
            output='screen',
            emulate_tty=True
        )
    )

    return LaunchDescription(launch_actions)

