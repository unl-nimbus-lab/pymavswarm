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


class ChangeHomePositionCommand(AgentMessage):
    """
    Change the home position of an agent.

    To reset the home position to the agent's current position, leave the latitude,
    longitude, and altitude coordinates as (0, 0, 0).
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        retry: bool,
        lat: float = 0,
        lon: float = 0,
        alt: float = 0,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
    ) -> None:
        """
        Create a command to change the home position of an agent.

        :param target_system: target system ID
        :type target_system: int
        :param target_component: target component ID
        :type target_component: int
        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool
        :param lat: The latitude of the home position, defaults to 0
        :type lat: float, optional
        :param lon: The longitude of the home position, defaults to 0
        :type lon: float, optional
        :param alt: The altitude of the home position, defaults to 0
        :type: alt: float, optional
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
        """
        if lat == 0 and lon == 0 and alt == 0:
            super().__init__(
                MessageSenders.RESET_HOME_TO_CURRENT,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                state_timeout,
                state_delay,
            )
        else:
            super().__init__(
                MessageSenders.RESET_HOME_POSITION,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                state_timeout,
                state_delay,
            )

        self.__altitude = alt
        self.__latitude = lat
        self.__longitude = lon

        return

    @property
    def altitude(self) -> Optional[float]:
        """
        Home position altitude [m].

        :return: home position altitude
        :rtype: Optional[float]
        """
        return self.__altitude

    @property
    def latitude(self) -> Optional[float]:
        """
        Home position latitude.

        :return: home position latitude
        :rtype: Optional[float]
        """
        return self.__latitude

    @property
    def longitude(self) -> Optional[float]:
        """
        Home position longitude.

        :return: home position longitude
        :rtype: Optional[float]
        """
        return self.__longitude
