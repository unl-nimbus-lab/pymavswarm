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
from typing import Any

from pymavlink import mavutil

from pymavswarm.utils import init_logger


class Connection:
    """Handle interaction with the network and the MAVLink connection."""

    def __init__(
        self,
        log_level: int = logging.INFO,
    ) -> None:
        """
        Construct a new Connection object.

        :param log_level: log level of the connection logger, defaults to
            logging.INFO
        :type debug: int, optional
        """
        self.__logger = init_logger(__name__, log_level=log_level)

        self.__mavlink_connection = None
        self.__connected = False

        return

    @property
    def mavlink_connection(self) -> Any | None:
        """
        Mavlink connection.

        :return: mavlink connection
        :rtype: Any
        """
        return self.__mavlink_connection

    @property
    def connected(self) -> bool:
        """
        Mavlink connection status.

        :return: connection status
        :rtype: bool
        """
        return self.__connected

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int,
        source_component: int,
        connection_attempt_timeout: float,
    ) -> bool:
        """
        Establish a MAVLink connection.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: serial port to attempt connection on
        :type port: str
        :param baudrate: serial connection baudrate
        :type baudrate: int
        :param source_system: system ID for the source system
        :type source_system: int
        :param source_component: component ID for the source system
        :type source_component: int
        :param connection_attempt_timeout: maximum time taken to establish a connection
        :type connection_attempt_timeout: float
        :return: flag indicating whether connection was successful
        :rtype: bool
        """
        self.__logger.info(
            f"Attempting to establish a new MAVLink connection at {port} with "
            f"baudrate {baudrate}."
        )

        # Create a new mavlink connection
        self.__mavlink_connection = mavutil.mavlink_connection(
            port,
            baud=baudrate,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
            input=False,
        )

        if self.__mavlink_connection is None:
            self.__logger.error("Failed to establish a MAVLink connection.")
            return False

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.__mavlink_connection.wait_heartbeat(
            timeout=connection_attempt_timeout
        )

        if not resp:
            self.__logger.error("Failed to establish a MAVLink connection.")
            self.__mavlink_connection = None
            connected = False
        else:
            self.__logger.info("Successfully established a MAVLink connection.")
            connected = True

        # Update the internal connection state
        self.__connected = connected

        return connected

    def disconnect(self) -> None:
        """Close the connection and stop all threads."""
        # Stop the main loop of the threads
        self.__connected = False

        # Shutdown the mavlink connection
        if self.__mavlink_connection is not None:
            self.__mavlink_connection.close()

        self.__mavlink_connection = None

        return

    def __str__(self) -> str:
        """
        Print connection information in a human-readable format.

        :return: connection information
        :rtype: str
        """
        context = {"connected": self.__connected}

        if self.__mavlink_connection is not None:
            context["source_system"] = self.__mavlink_connection.source_system
            context["source_component"] = self.__mavlink_connection.source_component

        return f"Connection: {context}"
