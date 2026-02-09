"""
MoveIt 제어 (경로 계획 및 실행)
"""


class MoveItControl:
    """MoveIt 제어 클래스"""
    
    def __init__(self, controller):
        """
        Args:
            controller: RobotArmController 인스턴스
        """
        self.controller = controller
        # TODO: MoveIt Python API 초기화
        # self.move_group = None
    
    def move_jog_cartesian(self, axis, delta):
        """
        카테시안 Jog 이동 (MoveIt 경로 계획)
        
        Args:
            axis (int): 축 번호 (0-5: X, Y, Z, Roll, Pitch, Yaw)
            delta (float): 이동량
        """
        if not self.controller:
            return
        
        # TODO: MoveIt으로 경로 계획 후 실행
        # 1. 현재 pose 가져오기
        # 2. delta 적용
        # 3. MoveIt plan 생성
        # 4. execute
        
        # 임시: SDK 방식으로 fallback
        from .sdk_control import SDKControl
        sdk = SDKControl(self.controller)
        sdk.move_jog_cartesian(axis, delta)
    
    def move_absolute_cartesian(self, target_coords):
        """
        절대 좌표 이동 (MoveIt 경로 계획)
        
        Args:
            target_coords (list): 목표 좌표 [X, Y, Z, Roll, Pitch, Yaw]
        """
        if not self.controller:
            return
        
        # TODO: MoveIt으로 경로 계획 후 실행
        # 1. target_coords를 Pose로 변환
        # 2. MoveIt plan 생성
        # 3. execute
        
        # 임시: SDK 방식으로 fallback
        from .sdk_control import SDKControl
        sdk = SDKControl(self.controller)
        sdk.move_absolute_cartesian(target_coords)
    
    def move_absolute_joint(self, target_angles):
        """
        절대 각도 이동 (MoveIt 경로 계획)
        
        Args:
            target_angles (list): 목표 각도 [J1, J2, J3, J4, J5, J6]
        """
        if not self.controller:
            return
        
        # TODO: MoveIt으로 경로 계획 후 실행
        # 1. target_angles 설정
        # 2. MoveIt plan 생성
        # 3. execute
        
        # 임시: SDK 방식으로 fallback
        from .sdk_control import SDKControl
        sdk = SDKControl(self.controller)
        sdk.move_absolute_joint(target_angles)
    
    def go_home(self):
        """홈 위치로 이동 (MoveIt 경로 계획)"""
        if not self.controller:
            return
        
        # TODO: MoveIt named target "home"으로 이동
        # self.move_group.set_named_target("home")
        # self.move_group.go()
        
        # 임시: SDK 방식으로 fallback
        from .sdk_control import SDKControl
        sdk = SDKControl(self.controller)
        sdk.go_home()
    
    def set_gripper(self, state):
        """
        그리퍼 제어 (직접 제어)
        
        Args:
            state (int): 0=UNGRIP, 1=GRIP
        """
        # 그리퍼는 MoveIt 없이 직접 제어
        if not self.controller:
            return
        
        self.controller.send_gripper(state)
    
    def set_servo(self, on):
        """
        서보 제어 (직접 제어)
        
        Args:
            on (bool): True=ON, False=OFF
        """
        # 서보는 MoveIt 없이 직접 제어
        if not self.controller:
            return
        
        self.controller.send_servo(on)
