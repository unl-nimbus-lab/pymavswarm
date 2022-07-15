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
import os
from datetime import datetime


def init_logger(name: str, log_level: int = logging.INFO) -> logging.Logger:
    """
    Initialize the logger with the desired debug levels.

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


class FileLogger:
    """File logging handler."""

    def __init__(self, filename: str | None = None) -> None:
        """
        Create a new file logger.

        :param filename: name of the file to write to, defaults to None
        :type filename: str | None, optional
        """
        log_dir = os.path.join(os.getcwd(), "logs")

        if not os.path.isdir(log_dir):
            os.mkdir(log_dir)

        if filename is None:
            filename = os.path.join(
                log_dir, f"{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.log"
            )
        else:
            filename = os.path.join(log_dir, filename)

        self.__log_file = open(filename, "w")  # type: ignore

        self.__log_file.write("timestamp,system_id,component_id,message_type,message\n")

        return

    def __call__(
        self,
        timestamp: int,
        system_id: int,
        component_id: int,
        message_type: str,
        message: dict,
    ) -> None:
        """
        Add a log to the log file.

        :param timestamp: timestamp that the message was received
        :type timestamp: int
        :param system_id: system ID of the agent that sent the message
        :type system_id: int
        :param component_id: component ID of the agent that sent the message
        :type component_id: int
        :param message_type: MAVLink message type
        :type message_type: str
        :param message: MAVLink message
        :type message: dict
        """
        self.__log_file.write(
            f"{timestamp},{system_id},{component_id},{message_type},{message}\n"
        )

        return
