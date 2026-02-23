import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # 1. 절대 경로 설정 (Home 디렉토리 기준)
    base_path = os.path.join(
        os.path.expanduser('~'), 
        'Desktop', 
        'lovo', 
        'serverfinal', 
        'mainserver'
    )
    domain_path = os.path.join(base_path, 'domain')
    
    # 설정 파일 경로: config/bridge_pinky
    config_dir = os.path.join(base_path, 'config', 'bridge_pinky')
    
    launch_actions = [
        # ROS_DOMAIN_ID를 59로 설정 (관제 PC용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
    ]

    # 2. Pinky 통합 도메인 브릿지 실행 (폴더 내의 모든 yaml 파일 실행)
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
    
    # 3. 로컬 상태 모니터링 실행 (대시보드)
    launch_actions.append(
        ExecuteProcess(
            cmd=['python3', os.path.join(base_path, 'status_pinky_monitor.py')],
            output='screen',
            emulate_tty=True
        )
    )

    return LaunchDescription(launch_actions)