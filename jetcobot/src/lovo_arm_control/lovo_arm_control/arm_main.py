import rclpy
import time
import math
import threading
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from trajectory_msgs.msg import JointTrajectory
from pymycobot.mycobot280 import MyCobot280


class JetCobotFullDriver(Node):
    def __init__(self):
        super().__init__('arm_driver')
        
        try:
            # /dev/ttyJETCOBOT 포트 연결 및 초기화
            self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
            self.mc.thread_lock = True
            self.mc.power_on()
            self.get_logger().info('★★★ JetCobot Real-Robot Driver Online ★★★')
        except Exception as e:
            self.get_logger().error(f'Connect Error: {e}')
        
        # [구독자] 1. GUI(PC)에서 보내는 단순 각도 제어
        self.create_subscription(Float64MultiArray, 'target_angles', self.angle_cb, 10)
        
        # [구독자] 2. MoveIt2(PC)에서 보내는 전체 궤적 제어 (순서 교정 포함)
        self.create_subscription(JointTrajectory, 'arm_controller/joint_trajectory', self.trajectory_cb, 10)
        
        # [구독자] 3. 서보 및 그리퍼 제어
        self.create_subscription(Bool, 'servo_status', self.servo_cb, 10)
        self.create_subscription(Int32, 'gripper_control', self.gripper_cb, 10)
        
        # [구독자] 4. 좌표 기반 테스트 시퀀스 실행
        self.create_subscription(Float64MultiArray, 'sequence_coords', self.sequence_cb, 10)
        
        # [구독자] 5. 단순 좌표 이동 (P1~P5 메모리 이동용)
        self.create_subscription(Float64MultiArray, 'target_coords', self.target_coords_cb, 10)
        
        # [발행자] 현재 엔코더 값을 GUI(PC)로 전송
        self.pub_current = self.create_publisher(Float64MultiArray, 'current_angles', 10)
        self.pub_coords = self.create_publisher(Float64MultiArray, 'current_coords', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        # 궤적 실행을 위한 변수
        self.trajectory = None
        self.trajectory_index = 0
        self.trajectory_timer = None
        
        # 좌표 시퀀스 실행 상태
        self.sequence_thread = None

        # Z 오프셋 보정(mm): 로봇이 +15mm 떠 있을 때 입력 좌표에서 15mm 감소
        self.z_offset_mm = 0.0#15.0

        # 좌표 제한 (필요한 축만 적용). 현재 Y축 제한: -281.45 ~ 281.45
        self.coord_limits = [None, (-281.45, 281.45), None, None, None, None]

    def _apply_z_offset(self, coords):
        """입력 좌표에 Z 오프셋을 적용"""
        adjusted = list(coords)
        adjusted[2] = float(adjusted[2]) - self.z_offset_mm
        # 좌표 제한 적용(필요한 축만)
        for idx, lim in enumerate(self.coord_limits):
            if lim is None:
                continue
            min_v, max_v = lim
            if adjusted[idx] < min_v:
                self.get_logger().warn(
                    f'좌표 제한으로 보정: index {idx}, {adjusted[idx]} -> {min_v}'
                )
                adjusted[idx] = min_v
            elif adjusted[idx] > max_v:
                self.get_logger().warn(
                    f'좌표 제한으로 보정: index {idx}, {adjusted[idx]} -> {max_v}'
                )
                adjusted[idx] = max_v
        return adjusted

    def servo_cb(self, msg):
        """서보 전원 온/오프"""
        if msg.data:
            self.mc.power_on()
            self.get_logger().info('Servo ON')
        else:
            self.mc.release_all_servos()
            self.get_logger().info('Servo OFF')

    def angle_cb(self, msg):
        """GUI에서 직접 입력한 단일 각도 지점으로 이동"""
        if len(msg.data) == 6:
            self.mc.send_angles(list(msg.data), 50)

    def trajectory_cb(self, msg):
        """MoveIt2가 계산한 궤적을 받아서 타이머 기반으로 실행"""
        if self.trajectory is not None:
            self.get_logger().warn('이미 실행 중인 궤적이 있습니다. 새 궤적을 무시합니다.')
            return
        
        self.get_logger().info('MoveIt 궤적 수신: 실행 시작')
        self.trajectory = msg
        self.trajectory_index = 0
        
        # 궤적 실행 타이머 시작 (50ms 마다 포인트 전송)
        if self.trajectory_timer is None:
            self.trajectory_timer = self.create_timer(0.05, self.trajectory_timer_cb)

    def trajectory_timer_cb(self):
        """궤적 포인트를 순차적으로 실행"""
        if self.trajectory is None:
            return
        
        if self.trajectory_index >= len(self.trajectory.points):
            self.get_logger().info('궤적 실행 완료')
            self.trajectory = None
            self.trajectory_index = 0
            if self.trajectory_timer is not None:
                self.trajectory_timer.cancel()
                self.trajectory_timer = None
            return
        
        point = self.trajectory.points[self.trajectory_index]
        angles = [math.degrees(a) for a in point.positions]
        self.mc.send_angles(angles, 50)
        self.trajectory_index += 1

    def gripper_cb(self, msg):
        """그리퍼 제어 (1: Close, 0: Open)"""
        state = 1 if msg.data == 1 else 0
        self.mc.set_gripper_state(state, 50)
        self.get_logger().info(f'Gripper Move: {state}')

    def timer_cb(self):
        """실시간 현재 각도 피드백"""
        angles = self.mc.get_angles()
        if angles and len(angles) == 6:
            msg = Float64MultiArray()
            msg.data = [float(a) for a in angles]
            self.pub_current.publish(msg)
        
        coords = self.mc.get_coords()
        if coords and len(coords) == 6:
            coord_msg = Float64MultiArray()
            coord_msg.data = [float(c) for c in coords]
            self.pub_coords.publish(coord_msg)

    def target_coords_cb(self, msg):
        """목표 좌표로 단순 이동 (P1~P5 메모리 이동용)"""
        if len(msg.data) != 6:
            self.get_logger().error('target_coords: 6개 좌표(x,y,z,rx,ry,rz)가 필요합니다.')
            return
        
        coords = self._apply_z_offset(msg.data)
        self.get_logger().info(f'목표 좌표로 이동: {coords}')
        
        try:
            self.mc.send_coords(coords, 50, 0)
        except Exception as e:
            self.get_logger().error(f'좌표 이동 중 오류: {e}')

    def sequence_cb(self, msg):
        """입력 좌표를 받아 Z↓ → X+ → Y- 이동 후 복귀"""
        if len(msg.data) != 6:
            self.get_logger().error('sequence_coords: 6개 좌표(x,y,z,rx,ry,rz)가 필요합니다.')
            return
        
        if self.sequence_thread is not None and self.sequence_thread.is_alive():
            self.get_logger().warn('좌표 시퀀스가 이미 실행 중입니다. 새 요청을 무시합니다.')
            return
        
        coords = self._apply_z_offset(msg.data)
        self.sequence_thread = threading.Thread(target=self._run_sequence, args=(coords,), daemon=True)
        self.sequence_thread.start()

    def _run_sequence(self, current_coords):
        """블로킹 동작이므로 스레드에서 실행"""
        try:
            work_coords = current_coords.copy()
            work_coords[2] -= 50
            self.get_logger().info(f'Z축을 {work_coords[2]}로 내립니다.')
            self.mc.send_coords(work_coords, 30, 0)
            time.sleep(2)
            
            x_coords = work_coords.copy()
            x_coords[0] += 20
            self.get_logger().info(f'X 좌표를 {x_coords[0]}로 이동합니다.')
            self.mc.send_coords(x_coords, 30, 0)
            time.sleep(2)
            
            y_coords = x_coords.copy()
            y_coords[1] -= 20
            self.get_logger().info(f'Y 좌표를 {y_coords[1]}로 이동합니다.')
            self.mc.send_coords(y_coords, 30, 0)
            time.sleep(2)
            
            final_coords = self.mc.get_coords()
            self.get_logger().info(f'최종 좌표: {final_coords}')
            
            self.get_logger().info('초기 위치로 복귀합니다.')
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            time.sleep(3)
            
            self.get_logger().info('초기 위치 복귀 완료')
        except Exception as e:
            self.get_logger().error(f'좌표 시퀀스 실행 중 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JetCobotFullDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()