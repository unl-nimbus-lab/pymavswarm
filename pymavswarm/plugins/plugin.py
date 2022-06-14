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

import logging
from typing import Callable, Union


class Plugin:
    """
    Template for a plugin.
    """

    def __init__(
        self, logger_name: str = "plugin", log_level: int = logging.INFO
    ) -> None:
        """
        Constructor.
        """
        self._logger = self.__init_logger(logger_name, log_level=log_level)
        self.__senders = {}
        self.__receivers = {}

        return

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels

        :param name: The name of the logger
        :type name: str

        :param log_level: The log level to display, defaults to logging.INFO
        :type log_level: int, optional

        :return: A newly configured logger
        :rtype: logging.Logger
        """
        logging.basicConfig()
        logger = logging.getLogger(name)
        logger.setLevel(log_level)
        return logger

    @property
    def senders(self) -> dict:
        """
        Methods responsible for sending mavlink messages.

        :rtype: dict
        """
        return self.__senders

    @property
    def receivers(self) -> dict:
        """
        Methods responsible for receiving mavlink messages

        :rtype: dict
        """
        return self.__receivers

    def _send_message(self, msg: Union[list, str]) -> Callable:
        """
        Decorator used to create a sender for a mavlink message

        :param msg: The message type to connect to the sender
        :type msg: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if msg not in self.__senders:
                self.__senders[msg] = []

            if function not in self.__senders[msg]:
                self.__senders[msg].append(function)

        return decorator

    def _receive_message(self, msg: Union[list, str]) -> Callable:
        """
        Decorator used to create a listener for a mavlink message
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python

        :param msg: The type of message to watch for
        :type msg: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if msg not in self.__receivers:
                self.__receivers[msg] = []

            if function not in self.__receivers[msg]:
                self.__receivers[msg].append(function)

        return decorator
