"""Shared communication state store."""
from PyQt6.QtCore import QObject, pyqtSignal

from .models import RobotConnectionState, SystemConnectionState


class ConnectionStateStore(QObject):
    """Shared state store for communication status across tabs."""

    state_changed = pyqtSignal(dict)
    server_state_changed = pyqtSignal(object)  # bool or None
    robot_state_changed = pyqtSignal(str, dict)

    def __init__(self):
        super().__init__()
        self._state = SystemConnectionState(server_reachable=None, robots={})

    def snapshot(self):
        """Return a serializable snapshot of current state."""
        return self._state.to_dict()

    def get_server_reachable(self):
        return self._state.server_reachable

    def get_robot_state(self, robot_id: str):
        robot = self._state.robots.get(robot_id)
        if not robot:
            return RobotConnectionState().to_dict()
        return robot.to_dict()

    def _ensure_robot(self, robot_id: str):
        if robot_id not in self._state.robots:
            self._state.robots[robot_id] = RobotConnectionState()
        return self._state.robots[robot_id]

    def set_server_reachable(self, reachable):
        self._state.server_reachable = reachable
        self.server_state_changed.emit(reachable)
        self.state_changed.emit(self.snapshot())

    def set_robot_ping(self, robot_id: str, connected: bool):
        robot = self._ensure_robot(robot_id)
        robot.ping_connected = bool(connected)
        self.robot_state_changed.emit(robot_id, robot.to_dict())
        self.state_changed.emit(self.snapshot())

    def set_robot_ros(self, robot_id: str, connected: bool):
        robot = self._ensure_robot(robot_id)
        robot.ros_connected = bool(connected)
        self.robot_state_changed.emit(robot_id, robot.to_dict())
        self.state_changed.emit(self.snapshot())

    def set_robot_camera(self, robot_id: str, connected: bool):
        robot = self._ensure_robot(robot_id)
        robot.camera_connected = bool(connected)
        self.robot_state_changed.emit(robot_id, robot.to_dict())
        self.state_changed.emit(self.snapshot())
