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
import threading
import time
from typing import Any, Optional

from pymavlink import mavutil

import pymavswarm.utils as swarm_utils


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
        self.__logger = swarm_utils.init_logger(__name__, log_level=log_level)

        self.__mavlink_connection = None
        self.__connected = False
        self.__source_system: Optional[int] = None
        self.__source_component: Optional[int] = None

        # Threads
        self.__heartbeat_thread = threading.Thread(target=self.__send_heartbeat)
        self.__heartbeat_thread.daemon = True

        return

    @property
    def mavlink_connection(self) -> Any:
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

    @property
    def source_system(self) -> Optional[int]:
        """
        System ID of the source system.

        :return: system ID
        :rtype: Optional[int]
        """
        return self.__source_system

    @property
    def source_component(self) -> Optional[int]:
        """
        Component ID of the source system.

        :return: component ID
        :rtype: Optional[int]
        """
        return self.__source_component

    def __send_heartbeat(self) -> None:
        """Send a GCS heartbeat to the network."""
        while self.__connected and self.__mavlink_connection is not None:
            self.__mavlink_connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )

            # Send a heartbeat at the recommended 1 Hz interval
            time.sleep(1)

        return

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int = 255,
        source_component: int = 0,
        connection_attempt_timeout: float = 2.0,
    ) -> bool:
        """
        Establish a MAVLink connection.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: port over which a connection should be established
        :type port: str
        :param baud: baudrate that a connection should be established with
        :type baud: int
        :param source_system: system ID of the connection, defaults to 255
        :type source_system: int, optional
        :param source_component: component ID of the connection, defaults to 0
        :type source_component: int, optional
        :param connection_attempt_timeout: maximum amount of time allowed to attempt
            to establish a connection, defaults to 2.0 [s]
        :type connection_attempt_timeout: float, optional
        :return: whether or not the connection attempt was successful
        :rtype: bool
        """
        # Create a new mavlink connection
        self.__mavlink_connection = mavutil.mavlink_connection(
            port,
            baud=baudrate,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

        if self.__mavlink_connection is None:
            return False

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.__mavlink_connection.wait_heartbeat(
            timeout=connection_attempt_timeout
        )

        if not resp:
            self.__mavlink_connection = None
            connected = False
        else:
            connected = True
            self.__heartbeat_thread.start()

        # Update the internal connection state
        self.__connected = connected

        return connected

    def disconnect(self) -> None:
        """Close the connection and stop all threads."""
        # Stop the main loop of the threads
        self.__connected = False

        # Join system threads
        if self.__heartbeat_thread is not None:
            self.__heartbeat_thread.join()

        # Shutdown the mavlink connection
        if self.__mavlink_connection is not None:
            self.__mavlink_connection.close()

        return
