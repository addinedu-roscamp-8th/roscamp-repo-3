import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # 설정 파일들과 스크립트가 위치한 디렉토리 경로
    config_dir = '/home/addinedu/Desktop/roscamp-repo-3/server/pinky_control_server'
    
    return LaunchDescription([
        # 1. ROS_DOMAIN_ID를 59로 설정 (관제 PC용)
        SetEnvironmentVariable('ROS_DOMAIN_ID', '59'),
        
        # 2. Pinky 1용 도메인 브릿지
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='pinky_1_bridge',
            arguments=[os.path.join(config_dir, 'bridge_pinky1.yaml')],
            output='log' # 터미널 대시보드 보호를 위해 로그 파일로 출력
        ),
        # 3. Pinky 2용 도메인 브릿지
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='pinky_2_bridge',
            arguments=[os.path.join(config_dir, 'bridge_pinky2.yaml')],
            output='log'
        ),
        # 4. Pinky 3용 도메인 브릿지
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='pinky_3_bridge',
            arguments=[os.path.join(config_dir, 'bridge_pinky3.yaml')],
            output='log'
        ),
        
        # 5. 로컬 상태 모니터링 실행 (대시보드)
        ExecuteProcess(
            cmd=['python3', os.path.join(config_dir, 'status_monitor.py')],
            output='screen',
            emulate_tty=True
        ),
    ])
