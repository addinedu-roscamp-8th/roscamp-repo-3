import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess

def generate_launch_description():
    # 1. 경로 설정
    # ~/Desktop/lovo/serverfinal/mainserver
    base_path = os.path.join(
        os.path.expanduser('~'), 
        'Desktop', 
        'lovo', 
        'serverfinal', 
        'mainserver'
    )


    # 2. 설정 파일 폴더 경로 (mainserver/config/bridge_robotarm)
    config_dir = os.path.join(base_path, 'config', 'bridge_robotarm')

    launch_actions = [
        # ROS_DOMAIN_ID를 59로 설정 (관제 PC와 통신용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
    ]

    # 3. Config 폴더 내의 모든 yaml 파일에 대해 도메인 브릿지 실행
    yaml_files = glob.glob(os.path.join(config_dir, '*.yaml'))
    
    if not yaml_files:
        print(f"⚠️ Warning: No YAML files found in {config_dir}")

    for yaml_file in yaml_files:
        filename = os.path.basename(yaml_file)
        node_name = f'robot_arm_domain_bridge_{filename.replace(".yaml", "")}'
        
        launch_actions.append(
            Node(
                package='domain_bridge',
                executable='domain_bridge',
                name=node_name,
                arguments=[yaml_file],
                output='screen'
            )
        )
    
    # 4. 로컬 상태 모니터링 실행 (대시보드)
    launch_actions.append(
        ExecuteProcess(
            cmd=['python3', os.path.join(base_path, 'status_robotarm_monitor.py')],
            output='screen',
            emulate_tty=True
        )
    )
    
    return LaunchDescription(launch_actions)