import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # 1. 기본 경로 설정 (mainserver까지)
    # ~/Desktop/roscamp-repo-3/serverfinal/mainserver
    main_server_path = os.path.join(
        os.path.expanduser('~'), 
        'Desktop', 
        'roscamp-repo-3', 
        'serverfinal', 
        'mainserver'
    )

    # 2. 설정 파일 폴더 경로 (mainserver/config/bridge_robotarm)
    config_dir = os.path.join(main_server_path, 'config', 'bridge_robotarm')

    launch_actions = [
        # ROS_DOMAIN_ID를 59로 설정 (관제 PC와 통신용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
    ]

    # 3. Config 폴더 내의 모든 yaml 파일에 대해 도메인 브릿지 실행
    yaml_files = glob.glob(os.path.join(config_dir, '*.yaml'))
    
    if not yaml_files:
        # 경로가 맞는지 터미널에 출력해서 확인용
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
    
    return LaunchDescription(launch_actions)