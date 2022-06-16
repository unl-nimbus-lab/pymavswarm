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

"""Utility functions used in the pymavswarm system."""

import logging
import time
from typing import Optional, Tuple

from pymavlink import mavutil

from pymavswarm import Connection


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


def ack_message(
    packet_type: str, connection: Connection, timeout: float = 1.0
) -> Tuple[bool, Optional[dict]]:
    """
    Ensure that a distributed message is acknowledged.

    :param message_type: The type of message that should be acknowledged
    :type message_type: str

    :param timeout: The acceptable time period before the acknowledgement is
        considered timed out, defaults to 1.0
    :type timeout: float, optional

    :return: acknowledgement success, message received indicating success (if any)
    :rtype: Tuple[bool, Any]
    """
    # Attempt to acquire the mutex
    if not connection.read_message_mutex.acquire(timeout=1.0):
        return False, None

    # Flag indicating whether the message was acknowledged
    ack_success = False

    # Message received converted to a dictionary
    message = None

    # Start acknowledgement timer
    start_t = time.time()

    while time.time() - start_t < timeout:
        try:
            message = connection.mavlink_connection.recv_match(
                type=packet_type, blocking=False
            )
            message = message.to_dict()

            if message["mavpackettype"] == packet_type:
                ack_success = True
                break
        except Exception:
            # This is a catch-all to ensure that the system continues attempting
            # acknowledgement
            continue

    # Continue reading status messages
    connection.read_message_mutex.release()

    return ack_success, message
