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

# type: ignore[no-redef]
# pylint: disable=function-redefined,unused-argument

import logging
import time
from typing import Tuple

from pymavswarm import Connection
from pymavswarm.handlers.senders import Senders
from pymavswarm.plugins.hrl_plugin.messages import HRLCommand


class HrlSenders(Senders):
    """Message senders for the HRL plugin."""

    HRL_COMMAND = "HRL_COMMAND"

    def __init__(
        self, logger_name: str = "hrl-senders", log_level: int = logging.INFO
    ) -> None:
        """
        Create a new HRL senders object.

        :param logger_name: logger name, defaults to "hrl-senders"
        :type logger_name: str, optional

        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(logger_name, log_level)

        @self._send_message(HrlSenders.HRL_COMMAND)
        @self._timer()
        def sender(
            message: HRLCommand,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Send an HRL command to the swarm.

            :param message: arming message
            :type message: HRLCommand

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
            """
            # Reset target
            connection.mavlink_connection.target_system = message.target_system
            connection.mavlink_connection.target_component = message.target_component

            # Send flight mode
            connection.mavlink_connection.mav.named_value_int_send(
                int(time.time()), str.encode("hrl-state-arg"), message.hrl_command
            )

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response
