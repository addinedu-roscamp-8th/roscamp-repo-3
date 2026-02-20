"""
Domain Bridge Launch File
PC(Domain 70)와 Robot(Domain 61) 간 통신 브릿지
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('lovo_gui')
    
    # YAML 설정 파일 경로
    config_file = PathJoinSubstitution([
        pkg_share, 'config', 'domain_bridge_config.yaml'
    ])
    
    # Launch Arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file,
        description='Path to domain bridge configuration file'
    )
    
    # Domain Bridge 실행
    domain_bridge_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'domain_bridge', 'domain_bridge',
            LaunchConfiguration('config')
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        config_arg,
        domain_bridge_node
    ])
