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

from pymavswarm.messages import AgentCommand


class TakeoffMessage(AgentCommand):
    """
    Takeoff to a certain location/altitude.
    """

    def __init__(
        self,
        target_system: int,
        target_comp: int,
        retry: bool,
        alt: float = 3.0,
        lat: Optional[float] = None,
        lon: Optional[float] = None,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        Constructor.

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param alt: The desired takeoff altitude
        :type alt: float

        :param lat: The desired takeoff latitude (optional)
        :type lat: Optional[float], optional

        :param lon: The desired takeoff longitude (optional)
        :type lon: Optional[float], optional

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
        super().__init__(
            "TAKEOFF",
            target_system,
            target_comp,
            retry,
            message_timeout=msg_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            optional_context_props=optional_context_props,
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
