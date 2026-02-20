#!/usr/bin/env python3
"""
Launch MoveIt 2 for the jetcobot robotic arm (NO Gazebo).

- MoveIt (move_group)
- robot_state_publisher (TF 생성)
- RViz (시각화)

이 launch 하나로:
- /joint_states (RPi) →
- TF (/tf, /tf_static) →
- RViz 로봇 모델 표시
가 자동으로 연결됨.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    package_name_moveit_config = "jetcobot_moveit_config"

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_config_package = LaunchConfiguration("rviz_config_package")

    pkg_share_moveit_config_temp = FindPackageShare(
        package=package_name_moveit_config
    )

    # Declare arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name="robot_name",
        default_value="jetcobot",
        description="Name of the robot to use",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Whether to start RViz",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value="move_group.rviz",
        description="RViz configuration file",
    )

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name="rviz_config_package",
        default_value=package_name_moveit_config,
        description="Package containing the RViz configuration file",
    )

    def configure_setup(context):

        robot_name_str = LaunchConfiguration("robot_name").perform(context)

        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(
            package_name_moveit_config
        )

        config_path = os.path.join(
            pkg_share_moveit_config, "config", robot_name_str
        )

        initial_positions_file_path = os.path.join(
            config_path, "initial_positions.yaml"
        )
        joint_limits_file_path = os.path.join(
            config_path, "joint_limits.yaml"
        )
        kinematics_file_path = os.path.join(
            config_path, "kinematics.yaml"
        )
        moveit_controllers_file_path = os.path.join(
            config_path, "moveit_controllers.yaml"
        )
        srdf_model_path = os.path.join(
            config_path, f"{robot_name_str}.srdf"
        )
        pilz_cartesian_limits_file_path = os.path.join(
            config_path, "pilz_cartesian_limits.yaml"
        )

        # ---- MoveIt config ----
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name_str,
                package_name=package_name_moveit_config,
            )
            .trajectory_execution(
                file_path=moveit_controllers_file_path
            )
            .robot_description_semantic(
                file_path=srdf_model_path
            )
            .joint_limits(
                file_path=joint_limits_file_path
            )
            .robot_description_kinematics(
                file_path=kinematics_file_path
            )
            .planning_pipelines(
                pipelines=[
                    "ompl",
                    "pilz_industrial_motion_planner",
                    "stomp",
                ],
                default_planning_pipeline="ompl",
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(
                file_path=pilz_cartesian_limits_file_path
            )
            .to_moveit_configs()
        )

        # ---- robot_state_publisher (TF 생성 핵심) ----
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"use_sim_time": use_sim_time},
            ],
        )

        # ---- move_group ----
        move_group_capabilities = {
            "capabilities": "move_group/ExecuteTaskSolutionCapability"
        }

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
                {"start_state": {"content": initial_positions_file_path}},
                move_group_capabilities,
            ],
        )

        # ---- RViz ----
        rviz_node = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                [
                    FindPackageShare(rviz_config_package),
                    "/rviz/",
                    rviz_config_file,
                ],
            ],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
        )

        rviz_exit_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=EmitEvent(
                    event=Shutdown(reason="rviz exited")
                ),
            ),
        )

        return [
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
            rviz_exit_handler,
        ]

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)

    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld

