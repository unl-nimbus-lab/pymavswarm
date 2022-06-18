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


class FlightModeCommand(AgentMessage):
    """Message signaling a flight mode change on an agent."""

    STABILIZE = "STABILIZE"
    ACRO = "ACRO"
    ALT_HOLD = "ALT_HOLD"
    AUTO = "AUTO"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"
    THROW = "THROW"
    SYSTEMID = "SYSTEMID"
    GUIDED = "GUIDED"

    @classmethod
    def get_supported_flight_modes(cls) -> List[str]:
        """
        Get the list of supported flight modes.

        :return: supported flight modes.
        :rtype: List[str]
        """
        return [
            cls.STABILIZE,
            cls.ACRO,
            cls.ALT_HOLD,
            cls.AUTO,
            cls.LOITER,
            cls.RTL,
            cls.LAND,
            cls.THROW,
            cls.SYSTEMID,
            cls.GUIDED,
        ]

    def __init__(
        self,
        flight_mode: str,
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
        Create a new flight mode command.

        :param flight_mode: desired flight mode
        :type msg_type: str

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
        super().__init__(
            MessageSenders.FLIGHT_MODE,
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            state_timeout,
            state_delay,
            optional_context_props,
        )

        if flight_mode not in FlightModeCommand.get_supported_flight_modes():
            raise ValueError(
                f"{flight_mode} is not a supported flight mode. Flight modes supported "
                f"include: {FlightModeCommand.get_supported_flight_modes()}"
            )

        self.__flight_mode = flight_mode

    @property
    def flight_mode(self) -> str:
        """
        Desired flight mode.

        :rtype: str
        """
        return self.__flight_mode

    @property
    def context(self) -> dict:
        """
        Flight mode command context.

        :return: message context
        :rtype: dict
        """
        context = super().context
        context["flight_mode"] = self.__flight_mode

        return context
