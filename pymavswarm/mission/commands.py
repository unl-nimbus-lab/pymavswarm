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

from pymavswarm.frames import GLOBAL_RELATIVE_FRAME
from pymavswarm.types import AgentID


class Command(mavutil.mavlink.MAVLink_mission_item_message):
    def __init__(
        self,
        agent_id: AgentID,
        frame: int,
        command: int,
        param1: int | float,
        param2: int | float,
        param3: int | float,
        param4: int | float,
        x: float,
        y: float,
        z: float,
    ) -> None:
        super().__init__(
            agent_id[0],
            agent_id[1],
            0,  # seq: unsupported
            frame,
            command,
            0,  # current: unsupported
            0,  # autocontinue: unsupported
            param1,
            param2,
            param3,
            param4,
            x,
            y,
            z,
        )
        return

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
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            hold,
            accept_radius,
            pass_radius,
            yaw,
            x,
            y,
            z,
        )
        return


class Takeoff(Command):
    def __init__(
        self,
        agent_id: AgentID,
        z: float,
        pitch: float = 0,
        yaw: float = 0,
        x: float = 0,
        y: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            pitch,
            0,
            0,
            yaw,
            x,
            y,
            z,
        )
        return


class LoiterUnlim(Command):
    def __init__(
        self,
        agent_id: AgentID,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init__(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0,
            0,
            0,
            x,
            y,
            z,
        )
        return


class LoiterTurns(Command):
    def __init__(
        self,
        agent_id: AgentID,
        turns: float,
        radius: float,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init__(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            turns,
            0,
            radius,
            0,
            x,
            y,
            z,
        )
        return


class LoiterTime(Command):
    def __init__(
        self,
        agent_id: AgentID,
        time: float,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init__(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            time,
            0,
            0,
            0,
            x,
            y,
            z,
        )
        return


class ReturnToLaunch(Command):
    def __init__(self, agent_id: AgentID) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return


class Land(Command):
    def __init__(
        self,
        agent_id: AgentID,
        x: float = 0,
        y: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init__(
            agent_id, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, x, y, 0
        )
        return


class Spline(Command):
    def __init__(
        self,
        agent_id: AgentID,
        x: float,
        y: float,
        z: float,
        delay: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
        super().__init__(
            agent_id,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            delay,
            0,
            0,
            0,
            x,
            y,
            z,
        )
        return


class GuidedEnable(Command):
    def __init__(self, agent_id: AgentID, enable: bool) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
            1 if enable else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return


class Delay(Command):
    def __init__(
        self,
        agent_id: AgentID,
        delay: int,
        future_hour: int = 0,
        future_minutes: int = 0,
        future_seconds: int = 0,
    ) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_DELAY,
            delay,
            future_hour,
            future_minutes,
            future_seconds,
            0,
            0,
            0,
        )
        return


class DoJump(Command):
    def __init__(
        self, agent_id: AgentID, waypoint_number: int, repeat_number: int
    ) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_DO_JUMP,
            waypoint_number,
            repeat_number,
            0,
            0,
            0,
            0,
            0,
        )
        return


class ConditionDelay(Command):
    def __init__(self, agent_id: AgentID, delay: float) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_CONDITION_DELAY,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return


class ConditionDistance(Command):
    def __init__(self, agent_id: AgentID, distance: float) -> None:
        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE,
            distance,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return


class ConditionYaw(Command):
    def __init__(
        self,
        agent_id: AgentID,
        angle: float,
        speed: float = 0,
        relative: bool = False,
        direction: int = 1,
    ) -> None:
        if relative and direction not in [-1, 1]:
            raise ValueError(
                "Please provide a valid direction when using relative direction. "
                f"Valid directions are 1 (CW) and -1 (CCW), got {direction}."
            )

        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            angle,
            speed,
            direction if relative else 0,
            1 if relative else 0,
            0,
            0,
            0,
        )
        return


class DoChangeSpeed(Command):
    def __init__(
        self, agent_id: AgentID, speed: float, speed_type: int = 1, throttle: float = -1
    ) -> None:
        if speed_type not in [0, 1, 2, 3]:
            raise ValueError(
                "Invalid speed type provided. Valid speed types are 0, 1, 2, and 3, "
                f"got {speed_type}"
            )

        super().__init__(
            agent_id,
            0,  # frame: unused
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            speed_type,
            speed,
            throttle,
            0,
            0,
            0,
            0,
        )
        return


class DoSetHome(Command):
    def __init__(
        self,
        agent_id: AgentID,
        use_current_position: bool,
        yaw: float = 0,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        frame: int = GLOBAL_RELATIVE_FRAME,
    ) -> None:
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
