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

import logging
from typing import Callable

import pymavswarm.utils as swarm_utils
from pymavswarm.types import MessageHandler


class Receivers:
    """Base class used to implement message receivers."""

    def __init__(
        self, logger_name: str = __name__, log_level: int = logging.INFO
    ) -> None:
        """
        Make a new receivers object.

        :param logger_name: name of the logger, defaults to __name__
        :type logger_name: str, optional
        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        self._logger = swarm_utils.init_logger(logger_name, log_level=log_level)
        self.__receivers: dict[str, list[MessageHandler]] = {}

        return

    @property
    def receivers(self) -> dict[str, list[MessageHandler]]:
        """
        Methods used to handle incoming messages.

        :return: message receivers
        :rtype: dict[str, list[MessageHandler]]
        """
        return self.__receivers

    def add_message_handler(self, message: str, callback: MessageHandler) -> None:
        """
        Add a handler for the specified message.

        :param message: message to add the handler to
        :type message: str
        :param callback: callback function to call on message reception
        :type callback: MessageHandler
        """
        if message not in self.__receivers:
            self.__receivers[message] = []

        if callback not in self.__receivers[message]:
            self.__receivers[message].append(callback)

        return

    def _receive_message(self, message: str) -> Callable:
        """
        Create a receiver for a MAVLink message.

        Decorator used to create a receiver for a mavlink message
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python

        :param message: The type of message to watch for
        :type message: list | str
        :return: decorator
        :rtype: Callable
        """

        def decorator(function: MessageHandler):
            self.add_message_handler(message, function)

        return decorator
