"""
SDK 직접 제어 (ROS2 토픽 직접 publish)
"""


class SDKControl:
    """SDK 직접 제어 클래스"""
    
    def __init__(self, controller):
        """
        Args:
            controller: RobotArmController 인스턴스
        """
        self.controller = controller
    
    def move_jog_cartesian(self, axis, delta):
        """
        카테시안 Jog 이동 (증분 이동)
        
        Args:
            axis (int): 축 번호 (0-5: X, Y, Z, Roll, Pitch, Yaw)
            delta (float): 이동량
        """
        if not self.controller:
            return
        
        # 현재 좌표 가져오기
        current_coords = self.controller.current_coords.copy()
        
        # 증분 적용
        current_coords[axis] += delta
        
        # 좌표 명령 전송
        self.controller.publish_coords(current_coords)
    
    def move_absolute_cartesian(self, target_coords):
        """
        절대 좌표 이동
        
        Args:
            target_coords (list): 목표 좌표 [X, Y, Z, Roll, Pitch, Yaw]
        """
        if not self.controller:
            return
        
        self.controller.publish_coords(target_coords)
    
    def move_absolute_joint(self, target_angles):
        """
        절대 각도 이동
        
        Args:
            target_angles (list): 목표 각도 [J1, J2, J3, J4, J5, J6]
        """
        if not self.controller:
            return
        
        self.controller.publish_angles(target_angles)
    
    def go_home(self):
        """홈 위치로 이동"""
        if not self.controller:
            return
        
        self.controller.go_home()
    
    def set_gripper(self, state):
        """
        그리퍼 제어
        
        Args:
            state (int): 0=UNGRIP, 1=GRIP
        """
        if not self.controller:
            return
        
        self.controller.send_gripper(state)
    
    def set_servo(self, on):
        """
        서보 제어
        
        Args:
            on (bool): True=ON, False=OFF
        """
        if not self.controller:
            return
        
        self.controller.send_servo(on)
