#!/usr/bin/env python3
"""
JetCobot PC node - Refactored version

This node handles:
- TCP pose publishing from TF
- Goal pose execution via MoveIt2
- ArUco marker detection and pick/place operations
- Gripper and servo control
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float64MultiArray, Float64,Int8
from lovo_interfaces.msg import CaptureImageWithCommand
from tf2_ros import Buffer, TransformListener
from pymoveit2 import MoveIt2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import math
import threading
import os
import yaml
from datetime import datetime

# Import custom modules (using absolute imports)
from jetcobot_bringup.utils.transforms import (
    quat_to_R, euler_from_quaternion, quaternion_from_euler,
    euler_from_rotation_matrix, quat_from_R_matrix
)
from jetcobot_bringup.utils.geometry import make_T, inv_T, Rx
from jetcobot_bringup.vision import ArucoDetector
from jetcobot_bringup.control import PickPlaceStateMachine
from jetcobot_bringup.nodes.sub.auto import PickPlaceHandler

from lovo_interfaces.srv import GetSnapshot


class JetCobotPC(Node):
    def __init__(self):
        super().__init__("jetcobot_PC_node")

        # ====== Load Parameters ======
        self._load_parameters()
        self._load_camera_calibration()
        self._load_pose_configs()
        
        # ====== Initialize Components ======
        self._init_tf()
        self._init_moveit2()
        self._init_vision()
        self._init_publishers()
        self._init_subscribers()
        self._init_state_machine()
        self._init_service_client()
        
        # ====== Timers ======
        self.create_timer(0.1 / self.pose_rate, self.publish_tcp_pose)
        self.create_timer(0.1, self._process_state_machine)
        self.create_timer(1.0, self._publish_robot_status)  # Publish robot status every 1 second
        
        # ====== State Variables ======
        self._last_tcp_pose = None  # (x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg)
        self.robot_state = 1  # 0=INIT, 1=IDLE, 2=BUSY, 3=SUCCESS, 4=ERROR
        os.makedirs(self.capture_save_dir, exist_ok=True)
        
        self.get_logger().info("✅ jetcobot_PC node started successfully")

    def _load_parameters(self):
        """Load ROS parameters"""
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "gripper_tcp")
        self.declare_parameter("pose_publish_rate", 10.0)
        self.declare_parameter("capture_save_dir", os.path.expanduser("~/captured_images"))
        default_picking_poses_yaml = os.path.join(
            get_package_share_directory("jetcobot_bringup"),
            "config",
            "picking_poses.yaml",
        )
        default_packing_poses_yaml = os.path.join(
            get_package_share_directory("jetcobot_bringup"),
            "config",
            "packing_poses.yaml",
        )
        self.declare_parameter("picking_poses_yaml", default_picking_poses_yaml)
        self.declare_parameter("packing_poses_yaml", default_packing_poses_yaml)
        
        self.base_frame = self.get_parameter("base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value
        self.pose_rate = self.get_parameter("pose_publish_rate").value
        self.capture_save_dir = self.get_parameter("capture_save_dir").value
        self.picking_poses_yaml = self.get_parameter("picking_poses_yaml").value
        self.packing_poses_yaml = self.get_parameter("packing_poses_yaml").value

    def _load_camera_calibration(self):
        """Load camera calibration from parameters"""
        # Declare parameters with defaults
        self.declare_parameter("camera_calibration.fx", 985.84531253827311)
        self.declare_parameter("camera_calibration.fy", 989.13989023921351)
        self.declare_parameter("camera_calibration.cx", 351.36443763136799)
        self.declare_parameter("camera_calibration.cy", 247.87768187819151)
        self.declare_parameter("camera_calibration.k1", -0.34697332313728774)
        self.declare_parameter("camera_calibration.k2", -1.1200580055555269)
        self.declare_parameter("camera_calibration.p1", -0.003985139731462751)
        self.declare_parameter("camera_calibration.p2", 0.003163420922100966)
        self.declare_parameter("camera_calibration.k3", 6.245322834389615)
        self.declare_parameter("camera_calibration.cam_to_tcp_dx", 0.0)
        self.declare_parameter("camera_calibration.cam_to_tcp_dy", 0.04)
        self.declare_parameter("camera_calibration.cam_to_tcp_dz", 0.045)
        self.declare_parameter("camera_calibration.cam_tcp_roll_deg", 90.0)
        self.declare_parameter("camera_calibration.marker_size_m", 0.015)
        
        # Load parameters
        fx = self.get_parameter("camera_calibration.fx").value
        fy = self.get_parameter("camera_calibration.fy").value
        cx = self.get_parameter("camera_calibration.cx").value
        cy = self.get_parameter("camera_calibration.cy").value
        k1 = self.get_parameter("camera_calibration.k1").value
        k2 = self.get_parameter("camera_calibration.k2").value
        p1 = self.get_parameter("camera_calibration.p1").value
        p2 = self.get_parameter("camera_calibration.p2").value
        k3 = self.get_parameter("camera_calibration.k3").value
        
        self.CAM_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.CAM_D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
        
        self.CAM_TO_TCP_DX = self.get_parameter("camera_calibration.cam_to_tcp_dx").value
        self.CAM_TO_TCP_DY = self.get_parameter("camera_calibration.cam_to_tcp_dy").value
        self.CAM_TO_TCP_DZ = self.get_parameter("camera_calibration.cam_to_tcp_dz").value
        self.CAM_TCP_ROLL_DEG = self.get_parameter("camera_calibration.cam_tcp_roll_deg").value
        self.MARKER_SIZE_M = self.get_parameter("camera_calibration.marker_size_m").value
        
        self.get_logger().info(f"📷 Camera calibration loaded: fx={fx:.2f}, fy={fy:.2f}")

    def _load_pose_configs(self):
        """Load picking and packing pose configurations from YAML files"""
        self.picking_pose_config = self._load_pose_config_file(
            self.picking_poses_yaml,
            "picking_poses_yaml",
        )
        self.packing_pose_config = self._load_pose_config_file(
            self.packing_poses_yaml,
            "packing_poses_yaml",
        )

    def _load_pose_config_file(self, yaml_path: str, label: str):
        """Load pose configuration from a YAML file"""
        if not yaml_path:
            self.get_logger().warn(f"{label} parameter is empty")
            return None

        if not os.path.exists(yaml_path):
            self.get_logger().warn(f"{label} not found: {yaml_path}")
            return None

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                pose_config = yaml.safe_load(f)
            self.get_logger().info(f"✅ Loaded poses YAML: {yaml_path}")
            return pose_config
        except Exception as e:
            self.get_logger().error(f"Failed to load {label}: {e}")
            return None

    def _init_tf(self):
        """Initialize TF listener"""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _init_moveit2(self):
        """Initialize MoveIt2"""
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "link1_to_link2", "link2_to_link3", "link3_to_link4",
                "link4_to_link5", "link5_to_link6", "link6_to_link6_flange",
            ],
            base_link_name="base_link",
            end_effector_name="gripper_tcp",
            group_name="arm",
            use_move_group_action=True,
        )

    def _init_vision(self):
        """Initialize vision components"""
        self.bridge = CvBridge()
        self.aruco_detector = ArucoDetector(
            self.CAM_K, self.CAM_D, self.MARKER_SIZE_M, self.get_logger()
        )

    def _init_publishers(self):
        """Initialize publishers"""
        self.gripper_pub = self.create_publisher(Int32, "/gripper_command", 10)
        self.servo_pub = self.create_publisher(Bool, "/servo_status", 10)
        self.tcp_pose_pub = self.create_publisher(Float64MultiArray, "/PTP_tcp_pose", 10)
        self.robot_status_pub = self.create_publisher(Int8, "/robot_states", 10 )

    def _init_subscribers(self):
        """Initialize subscribers"""
        self.create_subscription(Int32, "/PTP_gripper_command", self.ptp_gripper_command_cb, 10)
        self.create_subscription(Bool, "/PTP_servo_status", self.ptp_servo_status_cb, 10)
        self.create_subscription(Float64MultiArray, "/PTP_goal_pose", self.goal_pose_cb, 10)
        self.create_subscription(CaptureImageWithCommand, "/PTP_capture_image_with_command/compressed", self.capture_image_cb, 10)
        self.create_subscription(Float64, "/order_command", self.receipt_list_cb, 10)

    def _init_state_machine(self):
        """Initialize pick/place state machine"""
        self.state_machine = PickPlaceStateMachine(
            self, self.moveit2, self.gripper_pub, self.get_logger()
        )

    def _init_service_client(self):
        """Initialize service client for image capture"""
        self.image_capture_client = self.create_client(GetSnapshot, '/capture_image')
        self.get_logger().info("📞 Image capture service client created")
        
        # Initialize pick & place handler for receipt-based automation
        self.pickup_handler = PickPlaceHandler(self)
        self.get_logger().info("📦 Pick & Place handler initialized")

    # ==================== Callback Functions ====================
    
    def ptp_gripper_command_cb(self, msg: Int32):
        """Forward gripper command to hardware"""
        self.get_logger().info(f"/PTP_gripper_command: {msg.data}")
        self.gripper_pub.publish(msg)

    def ptp_servo_status_cb(self, msg: Bool):
        """Forward servo status to hardware"""
        self.get_logger().info(f"/PTP_servo_status: {msg.data}")
        self.servo_pub.publish(msg)

    def receipt_list_cb(self, msg: Float64):
        """Forward order command to pick & place handler"""
        self.robot_state = 2  # Change to BUSY when receipt received
        self.get_logger().info(f"🔄 State changed to BUSY")
        self.pickup_handler.receipt_list_cb(msg)
        
        
    def goal_pose_cb(self, msg: Float64MultiArray):
        """Execute goal pose via MoveIt2"""
        if not msg.data or len(msg.data) < 6:
            self.get_logger().warn("Invalid goal_pose data")
            return
        
        thread = threading.Thread(target=self._execute_goal_pose, args=(msg,), daemon=True)
        thread.start()

    def _execute_goal_pose(self, msg: Float64MultiArray):
        """Execute goal pose in separate thread"""
        try:
            # Extract and convert data
            x_m, y_m, z_m = [float(msg.data[i]) / 100.0 for i in range(3)]
            roll_deg, pitch_deg, yaw_deg = [float(msg.data[i]) for i in range(3, 6)]
            roll_rad, pitch_rad, yaw_rad = map(math.radians, [roll_deg, pitch_deg, yaw_deg])
            
            # Convert to quaternion (swap roll and pitch)
            qx, qy, qz, qw = quaternion_from_euler(pitch_rad, roll_rad, yaw_rad)
            
            self.get_logger().info(f"Goal pose: pos=({x_m:.4f},{y_m:.4f},{z_m:.4f}), rpy=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})")
            
            # Execute move
            self.moveit2.move_to_pose(
                position=(x_m, y_m, z_m),
                quat_xyzw=(qx, qy, qz, qw),
                frame_id="base_link",
                target_link="gripper_tcp",
                cartesian=False,
            )
            self.get_logger().info("✅ Goal pose action sent")
        except Exception as e:
            self.get_logger().error(f"Exception in _execute_goal_pose: {e}")

    def capture_image_cb(self, msg: CaptureImageWithCommand):
        """Handle image capture and ArUco-based pick/place"""
        try:
            self.get_logger().info(f"/PTP_capture_image received (command={msg.command})")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
            
            # Save image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filepath = os.path.join(self.capture_save_dir, f"capture_{timestamp}.jpg")
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"✅ Image saved to: {filepath}")
            
            # Execute ArUco action
            if msg.command in [0, 1]:
                gripper_cmd = 0 if msg.command == 0 else 100
                self._execute_aruco_action(
                    cv_image, gripper_cmd,
                    msg.offset_x, msg.offset_y, msg.offset_z, msg.offset_z_lift
                )
        except Exception as e:
            self.get_logger().error(f"Exception in capture_image_cb: {e}")

    def _execute_aruco_action(self, cv_image, gripper_command, offset_x, offset_y, offset_z, offset_z_lift):
        """Execute ArUco detection and pick/place"""
        if self._last_tcp_pose is None:
            self.get_logger().warn("⚠️ No TCP pose available yet")
            return
        
        # Detect ArUco marker
        result = self.aruco_detector.detect(cv_image)
        if result is None:
            return
        
        corners, ids, rvecs, tvecs = result
        
        # Use first detected marker
        rvec = rvecs[0].reshape(3, 1)
        t_marker_in_cam = tvecs[0].reshape(3, 1)
        
        # Calculate target position in base frame
        target_pos = self._calculate_target_position(t_marker_in_cam, offset_x/100, offset_y/100, offset_z/100)
        
        # Calculate target orientation
        marker_axes = self.aruco_detector.get_marker_axes(rvec)
        target_quat = self._calculate_target_orientation(marker_axes, t_marker_in_cam)
        
        # Send to state machine
        self.state_machine.set_target(target_pos, target_quat, gripper_command, offset_z_lift/100)

    def _calculate_target_position(self, t_marker_in_cam, offset_x, offset_y, offset_z):
        """Calculate target position in base frame"""
        # Get current TCP pose
        x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg = self._last_tcp_pose
        x_m, y_m, z_m = x_cm/100, y_cm/100, z_cm/100
        
        # Convert RPY to quaternion, then to rotation matrix
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = quaternion_from_euler(pitch_rad, roll_rad, yaw_rad)
        
        # base->tcp transformation
        t_base_tcp = np.array([[x_m], [y_m], [z_m]], dtype=np.float64)
        R_base_tcp = quat_to_R(qx, qy, qz, qw)
        T_base_tcp = make_T(R_base_tcp, t_base_tcp)
        
        # cam->tcp transformation (calibrated/assumed)
        R_cam_tcp = Rx(self.CAM_TCP_ROLL_DEG)
        t_cam_tcp = np.array([[self.CAM_TO_TCP_DX], [self.CAM_TO_TCP_DY], [self.CAM_TO_TCP_DZ]], dtype=np.float64)
        T_cam_tcp = make_T(R_cam_tcp, t_cam_tcp)
        
        # base->cam = base->tcp @ tcp->cam
        T_tcp_cam = inv_T(T_cam_tcp)
        T_base_cam = T_base_tcp @ T_tcp_cam
        
        # Transform marker to base frame
        marker_in_cam_homo = np.vstack([t_marker_in_cam, [[1.0]]])
        marker_in_base_homo = T_base_cam @ marker_in_cam_homo
        marker_in_base = marker_in_base_homo[:3, 0]
        
        # Add offsets
        target_x = marker_in_base[0] + offset_x
        target_y = marker_in_base[1] + offset_y
        target_z = marker_in_base[2] + offset_z
        
        self.get_logger().info(f"🎯 Target: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) m")
        return (target_x, target_y, target_z)

    def _calculate_target_orientation(self, marker_axes, t_marker_in_cam):
        """Calculate target TCP orientation based on marker axes (from jetcobot_PC.py logic)"""
        marker_x_axis_cam, marker_y_axis_cam, marker_z_axis_cam = marker_axes
        
        # Get current TCP pose
        x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg = self._last_tcp_pose
        x_m, y_m, z_m = x_cm/100, y_cm/100, z_cm/100
        
        # Convert RPY to quaternion, then to rotation matrix
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = quaternion_from_euler(pitch_rad, roll_rad, yaw_rad)
        
        # base->tcp transformation
        t_base_tcp = np.array([[x_m], [y_m], [z_m]], dtype=np.float64)
        R_base_tcp = quat_to_R(qx, qy, qz, qw)
        T_base_tcp = make_T(R_base_tcp, t_base_tcp)
        
        # cam->tcp transformation (calibrated/assumed)
        R_cam_tcp = Rx(self.CAM_TCP_ROLL_DEG)
        t_cam_tcp = np.array([[self.CAM_TO_TCP_DX], [self.CAM_TO_TCP_DY], [self.CAM_TO_TCP_DZ]], dtype=np.float64)
        T_cam_tcp = make_T(R_cam_tcp, t_cam_tcp)
        
        # base->cam = base->tcp @ tcp->cam
        T_tcp_cam = inv_T(T_cam_tcp)
        T_base_cam = T_base_tcp @ T_tcp_cam
        R_base_cam = T_base_cam[:3, :3]
        
        # Transform marker axes to base frame
        marker_x_axis_base = R_base_cam @ marker_x_axis_cam
        marker_y_axis_base = R_base_cam @ marker_y_axis_cam
        marker_z_axis_base = R_base_cam @ marker_z_axis_cam
        
        # Define TCP orientation based on marker axes
        # TCP +Y axis = Marker -Z axis (approach direction)
        # TCP +X axis = Marker +X axis (right direction)
        # TCP +Z axis = Marker -Y axis (up direction relative to marker)
        tcp_x_axis = marker_x_axis_base
        tcp_y_axis = -marker_z_axis_base
        tcp_z_axis = -marker_y_axis_base
        
        # Ensure orthogonality by using cross product
        tcp_z_axis = np.cross(tcp_x_axis, tcp_y_axis)
        tcp_z_axis = tcp_z_axis / np.linalg.norm(tcp_z_axis)
        tcp_y_axis = np.cross(tcp_z_axis, tcp_x_axis)
        tcp_y_axis = tcp_y_axis / np.linalg.norm(tcp_y_axis)
        tcp_x_axis = tcp_x_axis / np.linalg.norm(tcp_x_axis)
        
        # Create rotation matrix from TCP axes
        R_base_tcp_desired = np.column_stack([tcp_x_axis, tcp_y_axis, tcp_z_axis])
        
        # Convert to Euler angles
        target_roll_rad, target_pitch_rad, target_yaw_rad = euler_from_rotation_matrix(R_base_tcp_desired)
        
        # Adjust for robot's roll-pitch swap behavior
        target_roll_for_command_rad = target_pitch_rad  # Swap: pitch -> roll
        target_pitch_for_command_rad = target_roll_rad  # Swap: roll -> pitch
        target_yaw_for_command_rad = target_yaw_rad
        
        # Convert to quaternion using the swapped Euler angles
        target_qx_cmd, target_qy_cmd, target_qz_cmd, target_qw_cmd = quaternion_from_euler(
            target_pitch_for_command_rad, target_roll_for_command_rad, target_yaw_for_command_rad
        )
        
        return (target_qx_cmd, target_qy_cmd, target_qz_cmd, target_qw_cmd)

    def publish_tcp_pose(self):
        """Publish TCP pose from TF"""
        try:
            if not self.tf_buffer.can_transform(self.base_frame, self.tcp_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
                return
            
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tcp_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Extract position (m -> cm)
            x, y, z = tf.transform.translation.x * 100, tf.transform.translation.y * 100, tf.transform.translation.z * 100
            
            # Convert quaternion to RPY
            q = tf.transform.rotation
            roll_rad, pitch_rad, yaw_rad = euler_from_quaternion((q.x, q.y, q.z, q.w))
            roll_deg, pitch_deg, yaw_deg = map(math.degrees, [roll_rad, pitch_rad, yaw_rad])
            
            # Publish
            pose_array = Float64MultiArray()
            pose_array.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
            self.tcp_pose_pub.publish(pose_array)
            
            # Store for ArUco calculation
            self._last_tcp_pose = (x, y, z, roll_deg, pitch_deg, yaw_deg)
        except Exception as e:
            error_msg = str(e)
            if "does not exist" not in error_msg and "Could not find" not in error_msg:
                self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)

    def _process_state_machine(self):
        """Process state machine periodically"""
        self.state_machine.process()
    
    def _publish_robot_status(self):
        """Publish robot status every 1 second"""
        status_msg = Int8()
        status_msg.data = self.robot_state
        self.robot_status_pub.publish(status_msg)
        
        # Log status
        status_names = {0: "INIT", 1: "IDLE", 2: "BUSY", 3: "SUCCESS", 4: "ERROR"}
        status_name = status_names.get(self.robot_state, "UNKNOWN")
        self.get_logger().debug(f"📊 Robot Status: {status_name} ({self.robot_state})")
    
    def set_robot_state(self, state: int):
        """Set robot state and log the change"""
        state_names = {0: "INIT", 1: "IDLE", 2: "BUSY", 3: "SUCCESS", 4: "ERROR"}
        state_name = state_names.get(state, "UNKNOWN")
        self.get_logger().info(f"📋 State changed: {state_name} ({state})")
        self.robot_state = state
    # =================== Service Call for Image Capture ====================
    def request_image_capture(self):
        """
        Request image capture via service call
        
        Returns:
            Future object that will contain the response with CompressedImage
        """
        # TODO: Uncomment when service is ready
        # if not self.image_capture_client.wait_for_service(timeout_sec=1.0):
        if not self.image_capture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Image capture service not available")
            return None
        
        request = GetSnapshot.Request()  # Empty request - just trigger
        
        self.get_logger().info("📸 Requesting image capture")
        future = self.image_capture_client.call_async(request)
        future.add_done_callback(self._handle_image_capture_response)
        return future
    
    def _handle_image_capture_response(self, future):
        """Handle response from image capture service"""
        try:
            response = future.result()
            self.get_logger().info("✅ Image received from service")
            
            # Convert compressed image to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(response.image, desired_encoding="bgr8")
            
            # Save image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filepath = os.path.join(self.capture_save_dir, f"service_capture_{timestamp}.jpg")
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"💾 Saved to: {filepath}")
            
            # Process ArUco detection if needed
            # Example: self._execute_aruco_action(cv_image, gripper_cmd=0, offset_x=0, offset_y=0, offset_z=-5, offset_z_lift=5)
            
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")


def main():
    rclpy.init()
    node = JetCobotPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
