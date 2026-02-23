#!/usr/bin/env python3
"""
Auto pick & place handler for receipt-based operations

Receipt format: X.YZW
- X: place_id (ones digit)
- YZW: pick_ids (decimal digits, excluding 0)

Example: 1.456
- place_id = 1
- pick_ids = [4, 5, 6]
"""
import threading
import time
import math
import cv2
import os
from datetime import datetime
from decimal import Decimal
from std_msgs.msg import Float64
from lovo_interfaces.srv import GetSnapshot


class PickPlaceHandler:
    """Handler for receipt-based pick & place operations"""
    
    def __init__(self, node):
        """Initialize the PickPlaceHandler"""
        self.node = node # JetcobotPCNode 리소스 접근
        self._is_processing = False  # 플래그로 현재 작업 중인지 관리
        self._saved_place_marker_id = None  # 첫 번째 place의 ArUco marker ID 저장
        
        # Configurable delays (seconds)
        self.position_check_delay = 0.1  # Delay between position checks during movement
        self.image_capture_delay = 1.0   # Delay before capturing image to stabilize robot
        self.second_place_delay = 1.0    # Delay before executing second place with saved data
        self.before_move_delay = 1.0
    ## ==================== Receipt Processing =====================
    # /receipt_command 토픽에서 Float64 메시지를 수신하여 영수증을 처리하는 콜백 함수
    def receipt_list_cb(self, msg: Float64):
        """
        Process receipt command from /order_command topic
        
        Args:
            msg: Float64 message containing receipt value (e.g., 1.456)
        """
        # 이미 작업 중인 경우 새 영수증 무시
        if self._is_processing: 
            self.node.get_logger().warn("⚠️ Already processing a receipt")
            return
        
        receipt_value = msg.data
        self.node.get_logger().info(f"\n📋 Received receipt: {receipt_value}")
        
        # Parse receipt: X.YZW... → place_id=X, pick_ids=[Y,Z,W,...]
        # Use Decimal to preserve all digits after the decimal point
        receipt_str = format(Decimal(str(receipt_value)), "f")
        integer_part, _, fractional_part = receipt_str.partition(".")
        place_id = int(integer_part) % 10
        
        # Extract pick_ids from all digits after the decimal point
        pick_ids = [int(d) for d in fractional_part if d.isdigit() and d != "0"]
        
        # 터미널에 파싱 결과 출력
        self.node.get_logger().info(f"{'='*60}")
        self.node.get_logger().info(f"📊 Parsing Result:")
        self.node.get_logger().info(f"   Input Value: {receipt_value}")
        self.node.get_logger().info(f"   Place ID: {place_id}")
        self.node.get_logger().info(f"   Pick IDs: {pick_ids}")
        self.node.get_logger().info(f"{'='*60}\n")
        
        if not pick_ids:
            self.node.get_logger().warn("⚠️ No valid pick IDs found")
            return
        
        # ⚡ Load pose data from YAML config before starting thread
        picking_poses_datamap = self._load_picking_data_from_yaml(
            self.node.picking_pose_config,
            "picking_poses_yaml",
        )
        packing_poses_datamap = self._load_packing_data_from_yaml(
            self.node.packing_pose_config,
            "packing_poses_yaml",
        )

        if place_id == 0 and not packing_poses_datamap:
            self.node.get_logger().warn("⚠️ Packing pose data map is empty; skipping sequence")
            return
        if place_id != 0 and not picking_poses_datamap:
            self.node.get_logger().warn("⚠️ Picking pose data map is empty; skipping sequence")
            return

        if place_id == 0:
            # Start packing sequence when place_id is 0
            thread = threading.Thread(
                target=self._execute_packing_sequence,
                args=(place_id, pick_ids, packing_poses_datamap),
                daemon=True
            )
        else:
            # Start picking sequence for non-zero place_id
            thread = threading.Thread(
                target=self._execute_picking_sequence,
                args=(place_id, pick_ids, picking_poses_datamap),
                daemon=True
            )

        thread.start()
    
    def _execute_picking_sequence(self, place_id: int, pick_ids: list, picking_poses_datamap: dict):
        """Execute pick & place sequence"""
        self._is_processing = True
        self._saved_place_marker_id = None  # 새로운 시퀀스 시작 시 초기화
        
        try:
            for idx, pick_id in enumerate(pick_ids):
                self.node.get_logger().info(f"\n{'='*60}")
                self.node.get_logger().info(f"🔄 Pick & Place {idx+1}/{len(pick_ids)}: {pick_id} → {place_id}")
                self.node.get_logger().info(f"{'='*60}")
                
                # ========== PICK PHASE ==========
                self.node.get_logger().info(f"\n📦 PICK Phase (ID {pick_id})")
                
                time.sleep(self.before_move_delay)  # Delay before moving to pick position
                # 1. Move to pick position
                if not self._move_to_position(pick_id, picking_poses_datamap):
                    self.node.get_logger().error(f"❌ Failed to move to pick position")
                    break
                
                
                # 2. Capture image, detect ArUco, execute pick
                if not self._capture_and_execute_aruco(
                    pick_id,
                    gripper_command=0,
                    pose_datamap=picking_poses_datamap,
                    check_marker_id=pick_id,
                ):
                    self.node.get_logger().error(f"❌ Failed to execute pick")
                    break
                
                # ========== PLACE PHASE ==========
                self.node.get_logger().info(f"\n📍 PLACE Phase (ID {place_id})")
                
                time.sleep(self.before_move_delay)  # Delay before moving to place position
                # 3. Move to place position
                if not self._move_to_position(place_id, picking_poses_datamap):
                    self.node.get_logger().error(f"❌ Failed to move to place position")
                    break
                
                # 4. Execute place (with or without ArUco detection)
                if idx == 0:
                    # 첫 번째 사이클: ArUco detection 수행하고 marker ID 저장
                    self.node.get_logger().info(f"🎯 First cycle: detecting ArUco and saving marker ID")
                    if not self._capture_and_execute_aruco(place_id, gripper_command=100, pose_datamap=picking_poses_datamap, save_marker_id=True, pick_id=pick_id):
                        self.node.get_logger().error(f"❌ Failed to execute place")
                        break
                else:
                    # 이후 사이클: 저장된 marker ID와 같은 마커를 감지하여 새로 target 계산
                    self.node.get_logger().info(f"♻️ Cycle {idx+1}: detecting same marker ID {self._saved_place_marker_id}")
                    if not self._capture_and_execute_aruco(place_id, gripper_command=60, pose_datamap=picking_poses_datamap, check_marker_id=self._saved_place_marker_id, pick_id=pick_id):
                        self.node.get_logger().error(f"❌ Failed to execute place")
                        break
                
                self.node.get_logger().info(f"✅ Completed {pick_id} → {place_id}")
            
                # All tasks completed - move to standby position (pose 8)
                self.node.get_logger().info(f"\n🏁 Moving to standby position...")
                time.sleep(self.before_move_delay + 1.0)  # Delay before moving to standby
                if not self._move_to_position(8, picking_poses_datamap):
                    self.node.get_logger().warn(f"⚠️ Failed to move to standby position")
            
            self.node.get_logger().info(f"\n🎉 All tasks completed!")
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Error: {e}")
        finally:
            self._is_processing = False

    def _execute_packing_sequence(self, place_id: int, pick_ids: list, packing_poses_datamap: dict):
        """Execute packing sequence when place_id is 0"""
        self._is_processing = True
        self._saved_place_marker_id = None

        try:
            for idx, pick_id in enumerate(pick_ids):
                
                # === Packing Phase ===
                self.node.get_logger().info(f"\n{'='*60}")
                self.node.get_logger().info(f"📦 PACKING Pick Phase {idx+1}/{len(pick_ids)}: ID {pick_id}")
                self.node.get_logger().info(f"{'='*60}")

                time.sleep(self.before_move_delay)

                if not self._move_to_position("picking_poses", packing_poses_datamap):
                    self.node.get_logger().error("❌ Failed to move to packing picking pose")
                    break

                if not self._capture_and_execute_aruco(
                    "picking_poses",
                    gripper_command=0,
                    pose_datamap=packing_poses_datamap,
                    check_marker_id=pick_id,
                    pick_id=pick_id,
                ):
                    self.node.get_logger().error("❌ Failed to execute packing pick")
                    break

                time.sleep(self.before_move_delay)

                if not self._move_to_position("picking_end_pose", packing_poses_datamap):
                    self.node.get_logger().warn("⚠️ Failed to move to packing picking end pose")

                self.node.get_logger().info(f"✅ Completed packing pick phase for ID {pick_id}")

                # === Place Phase ===
                self.node.get_logger().info(f"\n📍 PACKING Place Phase {idx+1}/{len(pick_ids)}: ID {pick_id}")

                time.sleep(self.before_move_delay)  # Delay before moving to place position
                if not self._move_to_position("packing_poses", packing_poses_datamap):
                    self.node.get_logger().error("❌ Failed to move to packing place pose")
                    break

                if idx == 0:
                    self.node.get_logger().info("🎯 First cycle: detecting ArUco and saving marker ID")
                    if not self._capture_and_execute_aruco(
                        "packing_poses",
                        gripper_command=100,
                        pose_datamap=packing_poses_datamap,
                        save_marker_id=True,
                        pick_id=pick_id,
                    ):
                        self.node.get_logger().error("❌ Failed to execute packing place")
                        break
                else:
                    self.node.get_logger().info(
                        f"♻️ Cycle {idx+1}: detecting same marker ID {self._saved_place_marker_id}"
                    )
                    if not self._capture_and_execute_aruco(
                        "packing_poses",
                        gripper_command=60,
                        pose_datamap=packing_poses_datamap,
                        check_marker_id=self._saved_place_marker_id,
                        pick_id=pick_id,
                    ):
                        self.node.get_logger().error("❌ Failed to execute packing place")
                        break
                
                time.sleep(self.before_move_delay)
                
                if not self._move_to_position("packing_end_pose", packing_poses_datamap):
                    self.node.get_logger().warn("⚠️ Failed to move to packing end pose")

                self.node.get_logger().info(f"✅ Completed packing place phase for ID {pick_id}")


        except Exception as e:
            self.node.get_logger().error(f"❌ Error in packing sequence: {e}")
        finally:
            self._is_processing = False
    
    # ==================== Move Function ====================
    
    def _load_picking_data_from_yaml(self, pose_config: dict, label: str) -> dict:
        """
        Load all pose data from YAML config (detection poses + offset/lift metadata)
        
        Returns:
            dict: {pose_id: {'xyz_cm': [...], 'rpy_deg': [...], 'offset_cm': [...], 'z_lift_cm': ..., 'sub_offsets': {...}}, ...}
        """
        pick_place_datamap = {}
        
        if not pose_config or 'poses' not in pose_config:
            self.node.get_logger().warn(f"⚠️ Could not load pose data: {label} not available")
            return pick_place_datamap
        
        poses = pose_config['poses']
        
        for pose_id, pose_data in poses.items():
            xyz_cm = pose_data.get('xyz_cm', [0, 0, 0])
            rpy_deg = pose_data.get('rpy_deg', [0, 0, 0])
            offset_cm = pose_data.get('offset_cm', [0, 0, 0])
            z_lift_cm = pose_data.get('z_lift_cm', 0.0)
            
            # Also check for nested offsets structure (used in pose 2)
            offsets = pose_data.get('offsets', {})
            
            pick_place_datamap[pose_id] = {
                'xyz_cm': xyz_cm,
                'rpy_deg': rpy_deg,
                'offset_cm': offset_cm,
                'z_lift_cm': z_lift_cm,
                'sub_offsets': offsets  # For pick_ids specific offsets (if pose is a place)
            }
        
        self.node.get_logger().info(f"📋 Loaded pose data for {len(pick_place_datamap)} poses")
        return pick_place_datamap

    def _load_packing_data_from_yaml(self, pose_config: dict, label: str) -> dict:
        """
        Load packing pose data from YAML config.

        Expected keys in poses:
            - picking_poses
            - picking_end_pose
            - packing_poses
            - packing_end_pose

        Returns:
            dict: {
                pose_key: {
                    'xyz_cm': [...],
                    'rpy_deg': [...],
                    'offset_cm': [...],
                    'z_lift_cm': ...,
                    'sub_offsets': {...},
                    'note': str,
                },
                ...
            }
        """
        packing_datamap = {}

        if not pose_config or 'poses' not in pose_config:
            self.node.get_logger().warn(f"⚠️ Could not load pose data: {label} not available")
            return packing_datamap

        poses = pose_config['poses']
        required_pose_keys = [
            'picking_poses',
            'picking_end_pose',
            'packing_poses',
            'packing_end_pose',
        ]

        missing_keys = [key for key in required_pose_keys if key not in poses]
        if missing_keys:
            self.node.get_logger().warn(
                f"⚠️ Missing packing pose keys in {label}: {missing_keys}"
            )

        for pose_key, pose_data in poses.items():
            xyz_cm = pose_data.get('xyz_cm', [0, 0, 0])
            rpy_deg = pose_data.get('rpy_deg', [0, 0, 0])
            offset_cm = pose_data.get('offset_cm', [0, 0, 0])
            z_lift_cm = pose_data.get('z_lift_cm', 0.0)
            offsets = pose_data.get('offsets', {})
            note = pose_data.get('note', '')

            packing_datamap[pose_key] = {
                'xyz_cm': xyz_cm,
                'rpy_deg': rpy_deg,
                'offset_cm': offset_cm,
                'z_lift_cm': z_lift_cm,
                'sub_offsets': offsets,
                'note': note,
            }

        self.node.get_logger().info(
            f"📋 Loaded packing pose data for {len(packing_datamap)} poses"
        )
        return packing_datamap
    
    def _move_to_position(self, pose_id: int | str, pose_datamap: dict) -> bool:
        """
        Move robot to pose specified by pose_id
        
        Args:
            pose_id: ID or key of pose to move to
            pose_datamap: Dict containing pose data (xyz, rpy)
        
        Returns:
            bool: True if move succeeded, False otherwise
        """
        from jetcobot_bringup.utils.transforms import quaternion_from_euler
        
        pose_key = str(pose_id)
        
        # Get pose data from map
        if pose_key not in pose_datamap:
            self.node.get_logger().error(f"❌ Pose key {pose_id} not found in data")
            return False
        
        pose_data = pose_datamap[pose_key]
        
        # Extract position and orientation
        xyz = pose_data.get('xyz_cm', [0, 0, 0])
        rpy = pose_data.get('rpy_deg', [0, 0, 0])
        
        # Convert to meters
        x_m = xyz[0] / 100.0
        y_m = xyz[1] / 100.0
        z_m = xyz[2] / 100.0
        roll_deg = rpy[0]
        pitch_deg = rpy[1]
        yaw_deg = rpy[2]
        
        # Convert to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        
        # Convert to quaternion (swap roll and pitch for robot)
        qx, qy, qz, qw = quaternion_from_euler(pitch_rad, roll_rad, yaw_rad)
        
        self.node.get_logger().info(f"🚀 Moving to Pose {pose_id}")
        self.node.get_logger().info(f"   Position: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m")
        self.node.get_logger().info(f"   Orientation: R={roll_deg:.1f}° P={pitch_deg:.1f}° Y={yaw_deg:.1f}°")
        
        try:
            # Execute move
            self.node.moveit2.move_to_pose(
                position=(x_m, y_m, z_m),
                quat_xyzw=(qx, qy, qz, qw),
                frame_id="base_link",
                target_link="gripper_tcp",
                cartesian=False,
            )
            
            # Wait for movement to complete by checking position
            target_pos = [x_m, y_m, z_m]
            position_threshold = 0.05  # 5cm tolerance
            max_wait_time = 10.0  # Maximum wait time in seconds
            start_time = time.time()
            
            while time.time() - start_time < max_wait_time:
                if self.node._last_tcp_pose is not None:
                    # Convert from cm to m
                    current_pos = [self.node._last_tcp_pose[0] / 100.0,
                                   self.node._last_tcp_pose[1] / 100.0,
                                   self.node._last_tcp_pose[2] / 100.0]
                    distance = math.sqrt(
                        (current_pos[0] - target_pos[0])**2 +
                        (current_pos[1] - target_pos[1])**2 +
                        (current_pos[2] - target_pos[2])**2
                    )
                    
                    if distance < position_threshold:
                        self.node.get_logger().info(f"✅ Reached Pose {pose_id} (distance: {distance*1000:.1f}mm)")
                        return True
                
                time.sleep(self.position_check_delay)
            
            # Timeout reached
            if self.node._last_tcp_pose is not None:
                # Convert from cm to m
                current_pos = [self.node._last_tcp_pose[0] / 100.0,
                               self.node._last_tcp_pose[1] / 100.0,
                               self.node._last_tcp_pose[2] / 100.0]
                distance = math.sqrt(
                    (current_pos[0] - target_pos[0])**2 +
                    (current_pos[1] - target_pos[1])**2 +
                    (current_pos[2] - target_pos[2])**2
                )
                self.node.get_logger().warn(f"⚠️ Timeout reaching Pose {pose_id} (distance: {distance*1000:.1f}mm)")
            else:
                self.node.get_logger().warn(f"⚠️ No TCP pose available")
            
            return True  # Continue anyway
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to move to Pose {pose_id}: {e}")
            return False
    
    # ==================== ArUco Detection & Execution ====================
    
    def _capture_and_execute_aruco(self, pose_id: int | str, gripper_command: int, pose_datamap: dict, save_marker_id: bool = False, check_marker_id: int = None, pick_id: int = None) -> bool:
        """
        Capture image, detect ArUco marker, and execute pick/place
        
        Args:
            pose_id: ID or key of pose for reference
            gripper_command: 0 for close (pick), 100 for open (place)
            pose_datamap: Pose data map containing offset information
            save_marker_id: If True, save marker ID for reuse (first place cycle)
            check_marker_id: If set, verify detected marker ID matches this value
            pick_id: Pick ID for place operations (used to get sub_offsets if available)
        
        Returns:
            bool: True if execution succeeded, False otherwise
        """
        # Small delay to ensure robot is stable before capturing image
        time.sleep(self.image_capture_delay)

        # Capture image
        cv_image = self._capture_image()
        if cv_image is None:
            return False
        
        # Detect ArUco marker
        result = self.node.aruco_detector.detect(cv_image)
        if result is None:
            self.node.get_logger().warn("⚠️ No ArUco marker detected")
            return False
        
        corners, ids, rvecs, tvecs = result
        
        # Find marker matching check_marker_id or use first detected marker
        marker_index = None
        detected_marker_id = None
        
        if check_marker_id is not None:
            # Search for marker with matching ID among all detected markers
            for idx, marker_id in enumerate(ids):
                if marker_id[0] == check_marker_id:
                    marker_index = idx
                    detected_marker_id = marker_id[0]
                    break
            
            if marker_index is None:
                self.node.get_logger().error(f"❌ Expected marker ID {check_marker_id}, but not found in detected markers: {ids.flatten().tolist()}")
                return False
        else:
            # Use first detected marker (for pick phase and first place cycle)
            marker_index = 0
            detected_marker_id = ids[0][0]
        
        rvec = rvecs[marker_index].reshape(3, 1)
        t_marker_in_cam = tvecs[marker_index].reshape(3, 1)
        
        self.node.get_logger().info(f"✅ ArUco marker detected (ID: {detected_marker_id})")
        
        # Save marker ID if requested (for first place cycle)
        if save_marker_id:
            self._saved_place_marker_id = detected_marker_id
            self.node.get_logger().info(f"💾 Saved marker ID: {self._saved_place_marker_id}")
        
        # Get offset from pose data
        pose_key = str(pose_id)
        pose_data = pose_datamap.get(pose_key, {})
        
        # Check if this is a place operation with sub_offsets for specific pick_id
        if pick_id is not None:
            sub_offsets = pose_data.get('sub_offsets', {})
            pick_key = str(pick_id)
            if pick_key in sub_offsets:
                # Use pick_id-specific offset and z_lift
                offset_cm = sub_offsets[pick_key].get('offset_cm', [0, 0, 0])
                z_lift_cm = sub_offsets[pick_key].get('z_lift_cm', 0.0)
                self.node.get_logger().info(f"📍 Using sub_offset for pick_id {pick_id}: offset={offset_cm}, z_lift={z_lift_cm}cm")
            else:
                # Fallback to default offset
                offset_cm = pose_data.get('offset_cm', [0, 0, 0])
                z_lift_cm = pose_data.get('z_lift_cm', 0.0)
        else:
            # Use default offset (for pick operations)
            offset_cm = pose_data.get('offset_cm', [0, 0, 0])
            z_lift_cm = pose_data.get('z_lift_cm', 0.0)
        
        offset_m = [offset_cm[0]/100.0, offset_cm[1]/100.0, offset_cm[2]/100.0]
        z_lift_m = z_lift_cm / 100.0
        
        # Calculate position and orientation in base frame
        try:
            marker_pos = self.node._calculate_target_position(t_marker_in_cam, offset_m[0], offset_m[1], offset_m[2]) # target position with offset
            marker_axes = self.node.aruco_detector.get_marker_axes(rvec)
            marker_orient = self.node._calculate_target_orientation(marker_axes, t_marker_in_cam)
            
            # Execute via state machine
            self.node.state_machine.set_target(marker_pos, marker_orient, gripper_command, z_lift_m)
            
            # Wait for state machine to complete
            while self.node.state_machine.is_busy():
                time.sleep(0.1)
            
            action_type = "Pick" if gripper_command == 0 else "Place"
            self.node.get_logger().info(f"✅ {action_type} action completed")
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to execute action: {e}")
            return False
    
    def _capture_image(self):
        """
        Capture image from camera service
        
        Returns:
            cv2 image or None if capture fails
        """
        self.node.get_logger().info("📸 Requesting image from camera service...")
        
        if not self.node.image_capture_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("❌ Image capture service not available")
            return None
        
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            # GetSnapshot 서비스 요청 
            request = GetSnapshot.Request() # Request 객체 생성
            self.node.get_logger().info(f"📤 Sending GetSnapshot request... (attempt {attempt}/{max_attempts})")
            future = self.node.image_capture_client.call_async(request) # 비동기로 서버로 요청 보내기
            
            # Wait for response
            timeout = 5.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.node.get_logger().error("❌ Image capture timeout")
                    future.cancel()
                    break
                time.sleep(0.1)
            
            if not future.done():
                if attempt < max_attempts:
                    time.sleep(0.2)
                continue
            
            self.node.get_logger().info(f"📥 Received response from service")
            
            try:
                response = future.result() # GetSnapshot.Response 객체에서 결과 가져오기
                
                self.node.get_logger().info(f"🔍 Response type: {type(response)}")
                self.node.get_logger().info(f"🔍 Has image attr: {hasattr(response, 'image')}")
                
                # Check if image data is valid
                if not hasattr(response, 'image') or not response.image:
                    self.node.get_logger().error("❌ Response has no image attribute or image is None")
                    if attempt < max_attempts:
                        time.sleep(0.2)
                        continue
                    return None
                    
                self.node.get_logger().info(f"🔍 Image type: {type(response.image)}")
                self.node.get_logger().info(f"🔍 Has data attr: {hasattr(response.image, 'data')}")
                
                if not hasattr(response.image, 'data') or not response.image.data:
                    self.node.get_logger().error("❌ Received empty image data from service")
                    if attempt < max_attempts:
                        time.sleep(0.2)
                        continue
                    return None
                
                self.node.get_logger().info(f"📸 Received compressed image: format={response.image.format}, size={len(response.image.data)} bytes")
                
                # Service returns grayscale image
                cv_image = self.node.bridge.compressed_imgmsg_to_cv2(
                    response.image, 
                    desired_encoding="passthrough"
                )
                
                if cv_image is None or cv_image.size == 0:
                    self.node.get_logger().error("❌ Failed to decode image")
                    if attempt < max_attempts:
                        time.sleep(0.2)
                        continue
                    return None
                
                self.node.get_logger().info(f"📸 Image captured: {cv_image.shape}")
                
                # Save captured image to file
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = os.path.expanduser("~/captured_images")
                os.makedirs(save_dir, exist_ok=True)
                filename = os.path.join(save_dir, f"capture_{timestamp}.png")
                
                if cv2.imwrite(filename, cv_image):
                    self.node.get_logger().info(f"💾 Image saved: {filename}")
                else:
                    self.node.get_logger().warn(f"⚠️ Failed to save image to {filename}")
                
                return cv_image # OpenCV 이미지로 변환하여 반환
                
            except Exception as e:
                self.node.get_logger().error(f"❌ Failed to capture image: {e}")
                import traceback
                self.node.get_logger().error(f"Traceback: {traceback.format_exc()}")
                if attempt < max_attempts:
                    time.sleep(0.2)
                    continue
                return None
        
        return None
    

