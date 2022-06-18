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

from typing import List, Optional

from pymavswarm.handlers import MessageSenders
from pymavswarm.messages.agent_message import AgentMessage


class FlightSpeedCommand(AgentMessage):
    """Message signaling a change in the flight speed of an agent."""

    AIR_SPEED = 0
    GROUND_SPEED = 1
    CLIMB_SPEED = 2
    DESCENT_SPEED = 3

    @classmethod
    def get_speed_command_types(cls) -> List[int]:
        """
        Get the supported speed configuration types.

        :return: speed configuration types
        :rtype: List[int]
        """
        return [cls.AIR_SPEED, cls.GROUND_SPEED, cls.CLIMB_SPEED, cls.DESCENT_SPEED]

    def __init__(
        self,
        speed: float,
        speed_command_type: int,
        target_system: int,
        target_component: int,
        retry: bool,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a flight speed command.

        :param speed: desired speed [m/s]
        :type speed: float

        :param speed_command_type: type of speed (e.g., air speed) to configure
        :type speed_command_type: int

        :param target_system: target system ID
        :type target_system: int

        :param target_component: target component ID
        :type target_component: int

        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type message_timeout: float, optional

        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to None
        :type optional_context_props: Optional[dict], optional
        """
        if speed_command_type not in FlightSpeedCommand.get_speed_command_types():
            raise ValueError(
                f"{speed_command_type} is not a supported speed configuration "
                "the supported speed configuration types include: "
                f"{FlightSpeedCommand.get_speed_command_types()}"
            )

        super().__init__(
            MessageSenders.FLIGHT_SPEED,
            target_system,
            target_component,
            retry,
            message_timeout=message_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            optional_context_props=optional_context_props,
        )
        self.__speed = speed
        self.__speed_command_type = speed_command_type

        return

    @property
    def speed(self) -> float:
        """
        Desired speed [m/s].

        :return: target speed
        :rtype: float
        """
        return self.__speed

    @property
    def speed_command_type(self) -> int:
        """
        Speed type to configure.

        :return: speed command type
        :rtype: int
        """
        return self.__speed_command_type

    @property
    def context(self) -> dict:
        """
        Context of the message.

        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["speed"] = self.__speed
        context["speed_type"] = self.__speed_command_type

        return context
