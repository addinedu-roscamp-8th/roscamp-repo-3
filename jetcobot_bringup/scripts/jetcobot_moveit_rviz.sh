#!/bin/bash
# MoveIt + RViz only (no Gazebo)

cleanup() {
  echo "Cleaning up..."
  pkill -9 -f "ros2 launch jetcobot_moveit_config move_group.launch.py"
  pkill -9 -f "move_group|rviz2|robot_state_publisher|controller_manager|ros2_control_node"
}

trap 'cleanup' SIGINT SIGTERM

echo "Launching MoveIt + RViz (no Gazebo)..."
ros2 launch jetcobot_moveit_config move_group.launch.py \
  use_sim_time:=false \
  use_rviz:=true

wait

