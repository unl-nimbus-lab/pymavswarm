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


class WaypointCommand(AgentMessage):
    """Desired waypoint for an agent to fly to."""

    def __init__(
        self,
        lat: float,
        lon: float,
        alt: float,
        target_system: int,
        target_component: int,
        retry: bool,
        hold: float = 0,
        accept_radius: float = 0,
        pass_radius: float = 0,
        yaw: float = 0,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a waypoint command.

        :param lat: Latitude of the waypoint
        :type lat: float
        :param lon: Longitude of the waypoint
        :type lon: float
        :param alt: Altitude of the waypoint
        :type alt: float
        :param target_system: target system ID
        :type target_system: int
        :param target_component: target component ID
        :type target_component: int
        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool
        :param hold: Time to stay at waypoint for rotary wing (ignored by fixed wing)
        :type hold: float
        :param accept_radius: If the sphere with this radius (m) is hit, the waypoint
            counts as reached
        :type accept_radius: float
        :param pass_radius: 0 to pass through the WP, if > 0 radius to pass by WP.
            Positive value for clockwise orbit, negative value for counter-clockwise
            orbit. Allows trajectory control.
        :type pass_radius: float
        :param yaw: Desired yaw angle at waypoint (rotary wing). NaN to use the current
            system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
        :type yaw: float
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
            MessageSenders.WAYPOINT,
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            state_timeout,
            state_delay,
            optional_context_props,
        )

        self.__latitude = lat
        self.__longitude = lon
        self.__altitude = alt
        self.__hold = hold
        self.__accept_radius = accept_radius
        self.__pass_radius = pass_radius
        self.__yaw = yaw

        return

    @property
    def hold(self) -> float:
        """
        Time to stay at waypoint for rotary wing (ignored by fixed wing).

        :return: hold time
        :rtype: float
        """
        return self.__hold

    @property
    def accept_radius(self) -> float:
        """
        If the sphere with this radius (m) is hit, the waypoint counts as reached.

        :return: acceptance radius
        :rtype: float
        """
        return self.__accept_radius

    @property
    def pass_radius(self) -> float:
        """
        0 to pass through the WP, if > 0 radius to pass by WP.

        Positive value for clockwise orbit, negative value for counter-clockwise orbit.
        Allows trajectory control.

        :return: pass radius
        :rtype: float
        """
        return self.__pass_radius

    @property
    def yaw(self) -> float:
        """
        Desired yaw angle at waypoint (rotary wing).

        NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint,
        yaw to home, etc.).

        :return: yaw
        :rtype: float
        """
        return self.__yaw

    @property
    def latitude(self) -> float:
        """
        Latitude of the waypoint.

        :return: waypoint latitude
        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the waypoint.

        :return: waypoint longitude
        :rtype: float
        """
        return self.__longitude

    @property
    def altitude(self) -> float:
        """
        Altitude of the waypoint.

        :return: waypoint altitude
        :rtype: float
        """
        return self.__altitude

    @property
    def context(self) -> dict:
        """
        Context of the message.

        :return: message context
        :rtype: dict
        """
        context = super().context

        # Update to include new properties
        context["yaw"] = self.__yaw
        context["pass_radius"] = self.__pass_radius
        context["accept_radius"] = self.__accept_radius
        context["hold"] = self.__hold
        context["latitude"] = self.__latitude
        context["longitude"] = self.__longitude
        context["altitude"] = self.__altitude

        return context
