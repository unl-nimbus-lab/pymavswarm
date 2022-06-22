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

from typing import Optional

from pymavswarm.handlers import MessageSenders
from pymavswarm.messages.agent_message import AgentMessage


class TakeoffCommand(AgentMessage):
    """
    Takeoff to a certain location/altitude.

    The default for this command is to execute only a takeoff command (not the full
    sequence). The independent takeoff command requires that the agent has switched to
    GUIDED mode and armed. To successfully takeoff, the takeoff command must occur
    within 15 seconds of arming. Further documentation of this can be found at the
    following link:
    https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html

    When executing the takeoff sequence, the system will execute the full takeoff
    command sequence for you:
        1. Switch to GUIDED mode
        2. Arm
        3. Takeoff
    """

    def __init__(
        self,
        alt: float,
        target_system: int,
        target_component: int,
        retry: bool,
        lat: float = 0,
        lon: float = 0,
        execute_sequence: bool = False,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a takeoff command.

        :param alt: altitude to takeoff to
        :type alt: float
        :param target_system: target system ID
        :type target_system: int
        :param target_component: target component ID
        :type target_component: int
        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool
        :param lat: latitude to takeoff to, defaults to 0
        :type lat: float, optional
        :param lon: longitude to takeoff to, defaults to 0
        :type lon: float, optional
        :param execute_sequence: execute the takeoff sequence, defaults to False
        :type execute_sequence: bool, optional
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

        :raises ValueError: invalid altitude
        """
        if alt < 0.0 or alt > 150.0:
            raise ValueError(
                "An invalid home position altitude was provided "
                f"({alt}). Valid altitude range is [0 m, 150 m]"
            )

        if execute_sequence:
            super().__init__(
                MessageSenders.TAKEOFF_SEQUENCE,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                state_timeout,
                state_delay,
                optional_context_props,
            )
        else:
            super().__init__(
                MessageSenders.TAKEOFF,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                state_timeout,
                state_delay,
                optional_context_props,
            )

        self.__altitude = alt
        self.__latitude = lat
        self.__longitude = lon

        return

    @property
    def altitude(self) -> float:
        """
        Altitude that the agent should takeoff to.

        :rtype: float
        """
        return self.__altitude

    @property
    def latitude(self) -> float:
        """
        Latitude of the takeoff waypoint.

        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the takeoff waypoint.

        :rtype: float
        """
        return self.__longitude

    @property
    def context(self) -> dict:
        """
        Context of the message.

        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["latitude"] = self.__latitude
        context["longitude"] = self.__longitude
        context["altitude"] = self.__altitude

        return context
