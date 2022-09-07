# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from __future__ import annotations

from pymavlink import mavutil

from pymavswarm.types import AgentID


class Command(mavutil.mavlink.MAVLink_mission_item_message):
    @property
    def target_agent_id(self) -> AgentID:
        return (self.target_system, self.target_component)


class Waypoint(Command):
    def __init__(
        self,
        agent_id: AgentID,
        x: float,
        y: float,
        z: float,
        hold: float = 0,
        accept_radius: float = 0,
        pass_radius: float = 0,
        yaw: float = 0,
        frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    ) -> None:
        super().__init__(
            agent_id[0],
            agent_id[1],
            0,  # seq: unsupported
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # current: unsupported
            0,  # autocontinue: unsupported
            hold,
            accept_radius,
            pass_radius,
            yaw,
            x,
            y,
            z,
        )


class ReturnToLaunch(Command):
    def __init__(self) -> None:
        super().__init__()


class Takeoff(Command):
    def __init__(self) -> None:
        super().__init__()


class Land(Command):
    def __init__(self) -> None:
        super().__init__()


class LoiterUnlim(Command):
    def __init__(self) -> None:
        super().__init__()


class LoiterTurns(Command):
    def __init__(self) -> None:
        super().__init__()


class LoiterTime(Command):
    def __init__(self) -> None:
        super().__init__()


class Spline(Command):
    def __init__(self) -> None:
        super().__init__()


class GuidedEnable(Command):
    def __init__(self) -> None:
        super().__init__()


class Delay(Command):
    def __init__(self) -> None:
        super().__init__()


class DoJump(Command):
    def __init__(self) -> None:
        super().__init__()


class ConditionDelay(Command):
    def __init__(self) -> None:
        super().__init__()


class ConditionDistance(Command):
    def __init__(self) -> None:
        super().__init__()


class ConditionYaw(Command):
    def __init__(self) -> None:
        super().__init__()


class DoChangeSpeed(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetHome(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetServo(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetRelay(Command):
    def __init__(self) -> None:
        super().__init__()


class DoRepeatServo(Command):
    def __init__(self) -> None:
        super().__init__()


class DoRepeatRelay(Command):
    def __init__(self) -> None:
        super().__init__()


class DoDigiCamConfigure(Command):
    def __init__(self) -> None:
        super().__init__()


class DoDigiCamControl(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetCamTriggDist(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetRoi(Command):
    def __init__(self) -> None:
        super().__init__()


class DoParachute(Command):
    def __init__(self) -> None:
        super().__init__()


class DoGripper(Command):
    def __init__(self) -> None:
        super().__init__()


class DoGuidedLimits(Command):
    def __init__(self) -> None:
        super().__init__()


class DoSetResumeRepeatDist(Command):
    def __init__(self) -> None:
        super().__init__()


class DoFenceEnable(Command):
    def __init__(self) -> None:
        super().__init__()
