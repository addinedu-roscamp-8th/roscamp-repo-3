"""Communication state models."""
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any


@dataclass
class RobotConnectionState:
    """Per-robot connection state snapshot."""
    ping_connected: Optional[bool] = None
    ros_connected: Optional[bool] = None
    camera_connected: Optional[bool] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SystemConnectionState:
    """Global communication state snapshot."""
    server_reachable: Optional[bool] = None
    robots: Dict[str, RobotConnectionState] = None

    def to_dict(self) -> Dict[str, Any]:
        robots = self.robots or {}
        return {
            "server_reachable": self.server_reachable,
            "robots": {robot_id: state.to_dict() for robot_id, state in robots.items()},
        }
