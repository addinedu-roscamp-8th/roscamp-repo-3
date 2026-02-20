import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # 현재 파일의 디렉토리 경로 (server3/mainserver)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 설정 파일 경로: server3/mainserver/config/bridge_robotarm
    config_dir = os.path.join(current_dir, 'config', 'bridge_robotarm')
    
    launch_actions = [
        # 1. ROS_DOMAIN_ID를 59로 설정 (관제 PC와 통신용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
    ]

    # 2. Config 폴더 내의 모든 yaml 파일에 대해 도메인 브릿지 실행
    yaml_files = glob.glob(os.path.join(config_dir, '*.yaml'))
    
    if not yaml_files:
        print(f"Warning: No YAML files found in {config_dir}")

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
    
    return LaunchDescription(launch_actions)
