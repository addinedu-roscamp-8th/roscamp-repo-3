"""
Pick and place state machine for JetCobot
"""
import rclpy
from rclpy.time import Time
from std_msgs.msg import Int32


class PickPlaceStateMachine:
    """
    State machine for pick and place operations
    
    States:
        - IDLE: Waiting for new command
        - PENDING: Command received, ready to start
        - MOVING: Moving to target position
        - WAITING_FOR_EXECUTION: Waiting before gripper action
        - GRIPPER_CLOSING: Actuating gripper
        - LIFTING: Lifting object after grasp
    """
    
    # State constants
    IDLE = "IDLE"
    PENDING = "PENDING"
    MOVING = "MOVING"
    WAITING_FOR_EXECUTION = "WAITING_FOR_EXECUTION"
    GRIPPER_CLOSING = "GRIPPER_CLOSING"
    LIFTING = "LIFTING"
    
    def __init__(self, node, moveit2, gripper_pub, logger):
        """
        Initialize state machine
        
        Args:
            node: ROS2 node
            moveit2: MoveIt2 instance
            gripper_pub: Gripper command publisher
            logger: ROS logger
        """
        self.node = node
        self.moveit2 = moveit2
        self.gripper_pub = gripper_pub
        self.logger = logger
        
        # State variables
        self._state = self.IDLE
        self._state_start_time = None
        self._target_move_cmd = None  # (x, y, z, qx, qy, qz, qw)
        self._gripper_command_value = None
        self._move_goal_sent = False  # Track if goal was already sent
        self._lift_goal_sent = False  # Track if lift goal was already sent
        
        # Timing configuration
        self.execution_timeout = 30.0  # seconds
        self.gripper_wait_time = 0.8   # seconds (reduced from 4.0)
        self.gripper_close_time = 0.8  # seconds (reduced from 2.0)
        self.lift_distance = 0.1       # meters
        self.lift_timeout = 30.0       # seconds
        
    def set_target(self, position, orientation, gripper_command, lift_distance=0.1):
        """
        Set new target for pick/place operation
        
        Args:
            position: (x, y, z) target position in meters
            orientation: (qx, qy, qz, qw) target orientation quaternion
            gripper_command: 0 for close (pickup), 100 for open (place)
            lift_distance: Distance to lift after grasp (meters)
        """
        self._target_move_cmd = (*position, *orientation)
        self._gripper_command_value = gripper_command
        self.lift_distance = lift_distance
        self._state = self.PENDING
        self._state_start_time = self.node.get_clock().now()
        self._move_goal_sent = False  # Reset flag when new target is set
        self._lift_goal_sent = False  # Reset flag when new target is set
        
        action_type = "PICKUP" if gripper_command == 0 else "PLACE"
        self.logger.info(f"📝 Target set to PENDING state ({action_type} mode)")
    
    def process(self):
        """
        Process state machine (call this periodically)
        
        Returns:
            Current state
        """
        current_time = self.node.get_clock().now()
        
        if self._state == self.IDLE:
            return self._state
        
        elif self._state == self.PENDING:
            self._execute_move(current_time)
        
        elif self._state == self.MOVING:
            self._check_move_completion(current_time)
        
        elif self._state == self.WAITING_FOR_EXECUTION:
            self._wait_and_actuate_gripper(current_time)
        
        elif self._state == self.GRIPPER_CLOSING:
            self._wait_and_lift(current_time)
        
        elif self._state == self.LIFTING:
            self._check_lift_completion(current_time)
        
        return self._state
    
    def _execute_move(self, current_time):
        """Execute move to target"""
        # Only send goal once per state entry
        if self._move_goal_sent:
            return
        
        target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw = self._target_move_cmd
        
        self.logger.info("\n========== Moving to Target ==========")
        self.logger.info(f"🚀 Target position: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) m")
        self.logger.info(f"📐 Target quaternion: ({target_qx:.4f}, {target_qy:.4f}, {target_qz:.4f}, {target_qw:.4f})")
        
        # Send move command to MoveIt2
        self.moveit2.move_to_pose(
            position=(target_x, target_y, target_z),
            quat_xyzw=(target_qx, target_qy, target_qz, target_qw),
            frame_id="base_link",
            target_link="gripper_tcp",
            cartesian=False,
        )
        
        self._move_goal_sent = True  # Mark that goal was sent
        self._state = self.MOVING
        self._state_start_time = current_time
        self.logger.info("⏳ Planning and executing...")
    
    def _check_move_completion(self, current_time):
        """Check if move execution is complete"""
        try:
            if hasattr(self.moveit2, '_action_handle') and self.moveit2._action_handle is not None:
                goal_state = self.moveit2._action_handle.goal_state
                if goal_state in [4, 5, 8]:  # Terminal states
                    ok = self.moveit2.get_last_execution_error_code() == 0
                    if ok:
                        self.logger.info("✅ Move to target execution SUCCESS")
                        self._move_goal_sent = False  # Reset flag for next move
                        self._state = self.WAITING_FOR_EXECUTION
                        self._state_start_time = current_time
                    else:
                        err_code = self.moveit2.get_last_execution_error_code()
                        self.logger.error(f"❌ Move to target execution FAILED (error_code={err_code})")
                        self._move_goal_sent = False  # Reset flag
                        self._state = self.IDLE
                else:
                    # Check timeout
                    elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
                    if elapsed > self.execution_timeout:
                        self.logger.error(f"❌ Move execution TIMEOUT ({elapsed:.1f}s)")
                        self._move_goal_sent = False  # Reset flag
                        self._state = self.IDLE
            else:
                # Fallback - wait a bit before considering success
                elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
                if elapsed > 2.0:  # Wait at least 2 seconds before fallback
                    self.logger.warn("⚠️ Action handle not available, assuming move success (fallback)")
                    self._move_goal_sent = False  # Reset flag
                    self._state = self.WAITING_FOR_EXECUTION
                    self._state_start_time = current_time
        except Exception as e:
            self.logger.warn(f"Could not check execution state: {e}, assuming success")
            self._move_goal_sent = False  # Reset flag
            self._state = self.WAITING_FOR_EXECUTION
            self._state_start_time = current_time
    
    def _wait_and_actuate_gripper(self, current_time):
        """Wait before actuating gripper"""
        elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
        if elapsed >= self.gripper_wait_time:
            action_text = "Closing" if self._gripper_command_value == 0 else "Opening"
            self.logger.info(f"🤏 {action_text} gripper...")
            
            gripper_msg = Int32()
            gripper_msg.data = self._gripper_command_value
            self.gripper_pub.publish(gripper_msg)
            
            self._state = self.GRIPPER_CLOSING
            self._state_start_time = current_time
    
    def _wait_and_lift(self, current_time):
        """Wait after gripper closes, then lift"""
        elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
        if elapsed >= self.gripper_close_time:
            # Only send goal once per state entry
            if self._lift_goal_sent:
                return
            
            self.logger.info("✅ Gripper action completed")
            
            # Get target position for lift
            target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw = self._target_move_cmd
            lift_z = target_z + self.lift_distance
            
            self.logger.info(f"🚀 Lifting up by {self.lift_distance*100:.0f}cm to Z: {lift_z:.4f} m")
            
            # Send lift command
            self.moveit2.move_to_pose(
                position=(target_x, target_y, lift_z),
                quat_xyzw=(target_qx, target_qy, target_qz, target_qw),
                frame_id="base_link",
                target_link="gripper_tcp",
                cartesian=False,
            )
            
            self._lift_goal_sent = True  # Mark that goal was sent
            self._state = self.LIFTING
            self._state_start_time = current_time
            self.logger.info("⏳ Planning and executing lift...")
    
    def _check_lift_completion(self, current_time):
        """Check if lift execution is complete"""
        try:
            if hasattr(self.moveit2, '_action_handle') and self.moveit2._action_handle is not None:
                goal_state = self.moveit2._action_handle.goal_state
                if goal_state in [4, 5, 8]:  # Terminal states
                    ok = self.moveit2.get_last_execution_error_code() == 0
                    if ok:
                        self.logger.info("✅ Lift execution SUCCESS")
                    else:
                        err_code = self.moveit2.get_last_execution_error_code()
                        self.logger.error(f"❌ Lift execution FAILED (error_code={err_code})")
                    self.logger.info("=" * 50 + "\n")
                    self._lift_goal_sent = False  # Reset flag
                    self._state = self.IDLE
                else:
                    # Check timeout
                    elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
                    if elapsed > self.lift_timeout:
                        self.logger.error(f"❌ Lift execution TIMEOUT ({elapsed:.1f}s)")
                        self.logger.error("=" * 50 + "\n")
                        self._lift_goal_sent = False  # Reset flag
                        self._state = self.IDLE
            else:
                # Fallback - wait a bit before considering success
                elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
                if elapsed > 2.0:  # Wait at least 2 seconds before fallback
                    self.logger.warn("⚠️ Action handle not available, assuming lift success (fallback)")
                    self.logger.info("=" * 50 + "\n")
                    self._lift_goal_sent = False  # Reset flag
                    self._state = self.IDLE
        except Exception as e:
            self.logger.warn(f"Could not check lift execution state: {e}")
            self.logger.info("=" * 50 + "\n")
            self._lift_goal_sent = False  # Reset flag
            self._state = self.IDLE
    
    def get_state(self):
        """Get current state"""
        return self._state
    
    def is_idle(self):
        """Check if state machine is idle"""
        return self._state == self.IDLE
    
    def is_busy(self):
        """Check if state machine is busy (not idle)"""
        return self._state != self.IDLE
