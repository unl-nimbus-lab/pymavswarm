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

from pymavswarm.messages import AgentMessage
from pymavswarm.messages import SupportedMessages as supported_msgs


class FlightSpeedMessage(AgentMessage):
    """
    Message signaling a change in the flight speed of an agent.
    """

    def __init__(
        self,
        speed: float,
        speed_type: int,
        target_system: int,
        target_comp: int,
        retry: bool,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        Constructor.

        :param speed: The desired speed in m/s
        :type speed: float

        :param speed_type: The type of speed (e.g., air speed) to configure.
        :type speed_type: int

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param msg_timeout: The amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type msg_timeout: float, optional

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to {}
        :type optional_context_props: dict, optional
        """
        if speed_type not in supported_msgs.flight_speed_commands.get_supported_types():
            raise ValueError(
                f"{speed_type} is not a supported speed configuration "
                "the supported speed configuration types include: "
                f"{supported_msgs.flight_speed_commands.get_supported_types()}"
            )

        super().__init__(
            "FLIGHT_SPEED",
            target_system,
            target_comp,
            retry,
            msg_timeout=msg_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            optional_context_props=optional_context_props,
        )
        self.__speed = speed
        self.__speed_type = speed_type

        return

    @property
    def speed(self) -> float:
        """
        Desired speed in m/s.

        :rtype: float
        """
        return self.__speed

    @property
    def speed_type(self) -> int:
        """
        The type of speed to configure.

        :rtype: int
        """
        return self.__speed_type

    @property
    def context(self) -> dict:
        """
        Context of the message.

        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["speed"] = self.__speed
        context["speed_type"] = self.__speed_type

        return context
